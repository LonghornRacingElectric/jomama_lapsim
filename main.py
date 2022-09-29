import pandas as pd
import numpy as np

def main():
    easy_driver = Racecar("TEMP", 48/12)

    endurance_track = Track("endurance_michigan_2019", 120, 100)
    autocross_track = None #Track("autocross_michigan_2019.csv", best_time = 120)
    
    # If a new racing line is needed (i.e. trackwidth changes), enable these lines
    #endurance_track.generate_racing_line(easy_driver.front_trackwidth)
    #autocross_track.generate_racing_line(easy_driver.front_trackwidth)

    results, points = Competition(endurance_track, autocross_track, easy_driver).run()

    results[0].to_csv("results/endurance_michigan_2019-easy_driver.csv")
    #results[1].to_csv("results/endurance_michigan_2019-easy_driver.csv")

    print(f"Points: {points}")

class Competition:
    def __init__(self, endurance, autocross, racecar):
        self.racecar = racecar

        self.endurance_sim = Simulation(endurance, self.racecar)
        self.autocross_sim = autocross
        self.skidpad_sim = None
        self.acceleration_sim = None

    def run(self):
        result_endurance, time_e = self.endurance_sim.run()
        results_autocross, time_ax = None, None #Simulation(self.autocross, self.racecar).run()
        results_skidpad, time_s = None, None #Simulation(self.skidpad, self.racecar).run()
        results_acceleration, time_a = None, None #Simulation(self.acceleration, self.racecar).run()
        
        points = self.points(time_e, time_ax, time_s, time_a)

        return [result_endurance, results_autocross, results_skidpad, results_acceleration], points

    def points(self, time_e, time_ax, time_s, time_a):
        best = self.endurance_sim.track.best_time
        worst = self.endurance_sim.track.worst_time
        return 400

class Simulation:
    def __init__(self, track, car):
        self.track = track
        self.car = car

        self.time = 0
        self.results = None

    def run(self):
        # get turn radii from raceline points
        track_points = curvature(self.track.path_points())

        # find dist traveled for each point on track
        track_points["dist"] = 0
        previous_row = track_points.iloc[1]
        for i, row in track_points.iloc[1:].iterrows():
            track_points.loc[i,"dist"] = ((row["x"] - previous_row["x"])**2 + (row["y"] - previous_row["y"])**2)**0.5
            previous_row = row

        # TODO: temp fix
        track_points = track_points.drop(track_points.shape[0]-1)
        track_points = track_points.drop(0)
        track_points = track_points.drop(1)

        initial_forward_sim_df = self.__forward_sim(track_points.copy(deep = True), 20, False) # TODO: starting speed?
        starting_vel = initial_forward_sim_df["vel"].iloc[initial_forward_sim_df.shape[1]]
        reverse_sim_df = self.__forward_sim(track_points.copy(deep = True), starting_vel, True)
        forward_sim_df = self.__forward_sim(track_points.copy(deep = True), starting_vel, False)
    
        ### COMBINE RESULTS
        time, rows = 0, []
        columns = [*reverse_sim_df.columns, "time"]

        for index, row in track_points.iterrows():
            reverse_row = reverse_sim_df.loc[index]
            forward_row = forward_sim_df.loc[index]
            if (forward_row["vel"] - reverse_row["vel"]) < 0:
                time += forward_row.loc["delta_t"]
                rows.append(np.array([*forward_row, time]))
            else:
                time += reverse_row.loc["delta_t"]
                reverse_row.loc["ax"]*=-1
                rows.append(np.array([*reverse_row, time]))
        
        self.results = pd.DataFrame(rows, columns=columns)
        self.time = self.results["time"].max()
        return self.results, self.time

    def __forward_sim(self, df, starting_vel, is_reverse):
        accel_func = self.car.deccel if is_reverse else self.car.accel
        gravity = 32.2
        vel = starting_vel
        for x in ["delta_t", "delta_vel", "vel", "ay", "ax"]:
            df[x] = 0

        for i, row in (df[::-1] if is_reverse else df).iterrows():
            # max speed through segment
            vmax = min(self.car.max_vel, self.car.max_vel_corner(row["R"]))

            AY = self.car.lateral(self.car.max_vel) # accel capabilities
            df.loc[i,"ay"] = min((vel*60**2/5280)**2/row["R"]/gravity, AY) # actual accel
            if vel < vmax:
                df.loc[i,"ax"] = accel_func(vel, df.loc[i,"ay"])
                df.loc[i,"delta_t"] = max(np.roots([0.5*gravity*df.loc[i,"ax"], vel, -row["dist"]]))
                df.loc[i,"delta_vel"] = min(gravity*df.loc[i,"ax"]*df.loc[i,"delta_t"], vmax-vel)
                df.loc[i,"vel"] = vel + df.loc[i,"delta_vel"]
                vel = df.loc[i,"vel"]
            else:
                # car maxed out, no more accel
                vel = vmax
                df.loc[i,"vel"] = vel
                df.loc[i,"delta_t"] = row["dist"]/vel
                df.loc[i,"ax"], df.loc[i,"delta_vel"] = 0, 0
        return df

def curvature(track_df):
    # radius of curvature and curvature vector for 2D or 3D curve
    # X - x,y column array
    # [L, R, k] = Cumulative arc length, radius of curvature, curvature vector
    track_df["L"], track_df["R"], track_df["kx"], track_df["ky"], track_df["kz"] = 0, 0, 0, 0, 0
    previous_row = track_df.iloc[-1]
    previous_coords_row = track_df.iloc[-1][["x", "y", "z"]]
    for i, row in track_df.iloc[:-1].iterrows():
        coords_row = row[["x", "y", "z"]]
        track_df.loc[i,"R"], _, k = circumcenter(coords_row, previous_coords_row, track_df.iloc[i+1, :][["x", "y", "z"]])
        track_df.loc[i,"kx"], track_df.loc[i,"ky"], track_df.loc[i,"kz"] = k
        track_df.loc[i,"L"] = previous_row["L"] + np.linalg.norm(coords_row - previous_coords_row)
        previous_row, previous_coords_row = row, coords_row
    
    N = track_df.shape[0]
    track_df.loc[N-1,"R"], _, k = circumcenter(previous_coords_row, track_df[["x", "y", "z"]].iloc[N-1, :], track_df.iloc[0, :][["x", "y", "z"]])
    track_df.loc[N-1,"kx"], track_df.loc[N-1,"ky"], track_df.loc[N-1,"kz"] = k
    track_df.loc[N-1,"L"] = track_df["L"].iloc[N-2] + np.linalg.norm(track_df[["x", "y", "z"]].iloc[N-1, :] - previous_coords_row)
    return track_df

def circumcenter(A,B,C):
    # center and radius of the circumscribed circle for the triangle ABC
    # R - Radius
    # M - 3D coordinate vector for center
    # k - vector of length 1/r in the direction from A to M
    D = np.cross(B-A, C-A)
    b = np.linalg.norm(A-C)
    c = np.linalg.norm(A-B)
    E = np.cross(D, B-A)
    F = np.cross(D, C-A)
    G = (b**2*E-c**2*F)/np.linalg.norm(D)**2/2 if np.linalg.norm(D) != 0 else np.array([0,0,0])
    M = A + G
    R = np.linalg.norm(G)
    k = G if R == 0 else G/R**2
    return R, M, k

class Track:
    def __init__(self, file_name, best_time, worst_time, load_cached_racing_line = True):
        #self.cones = pd.read_csv("tracks/" + file_name)
        self.racing_line = pd.read_csv("racing_lines/" + file_name + "-racing_line.csv") if load_cached_racing_line else None
        self.best_time = best_time
        self.worst_time = worst_time

    def generate_racing_line(self, ggv):
        pass # TODO: Robert?

    def path_points(self):
        return self.racing_line

class Racecar:
    def __init__(self, ggv_file, front_trackwidth):
        #self.file = pd.read_csv(ggv_file) #TODO!
        self.front_trackwidth = front_trackwidth
        self.max_vel = 65

    def accel(self, vel, lateral = 0):
        accel_max = 0.75
        return (accel_max**2 - lateral**2*accel_max**2/self.lateral(0)**2)**0.5

    def deccel(self, vel, lateral = 0):
        deccel_max = 2
        return (deccel_max**2 - lateral**2*deccel_max**2/self.lateral(0)**2)**0.5

    def lateral(self, vel):
        return 1.8

    def max_vel_corner(self, radius):
        if radius > 1000: # TODO
            return self.max_vel
        vel = (self.lateral(0) * 32.2 * radius) ** 0.5 * 5280/60**2
        return vel
    

if __name__ == "__main__":
    main()