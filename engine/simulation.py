import numpy as np
import pandas as pd
import time as timelib
import math

ENABLE_PROFILING = True # use this to measure times

class Simulation:
    def __init__(self, title, car, track, is_skidpad = False):
        self.title = title
        self.track = track
        self.car = car
        self.is_skidpad = is_skidpad
        self.logLines = []

        self.time = 0
        self.results = None
        self.reverse_sim_results = None
        self.forward_sim_results = None
    
    def log(self, *args):
        self.logLines.append(" ".join(map(str, args)))
    
    def begin_profiling(self):
        self.start_time = timelib.time()
        self.log("")
        self.log("===", self.title, "Sim", "===")
    
    def profiling_checkpoint(self, name):
        end_time = timelib.time()
        elapsed_time = end_time - self.start_time
        self.log(name.ljust(24), round(elapsed_time, 3), "s")
        self.start_time = end_time
    
    def finish_profiling(self):
        if ENABLE_PROFILING:
            print("\n".join(self.logLines))

    def run(self):
        self.begin_profiling()

        if self.is_skidpad:
            return pd.DataFrame(), self.__skidpad_sim()

        # get turn radii from raceline points
        track_points = curvature(self.track.path_points()).astype('float64')
        self.profiling_checkpoint("curvature")

        # find dist traveled for each point on track
        track_points["dist"], track_points["pos"], length_traveled = 0.0, 0.0, 0.0
        p0 = track_points.loc[0, "x"], track_points.loc[0, "y"]
        for i in range(1, len(track_points)):
            p1 = track_points.loc[i, "x"], track_points.loc[i, "y"]
            dist = math.dist(p0, p1)
            length_traveled += dist
            track_points.loc[i, "dist"] = dist
            track_points.loc[i, "pos"] = length_traveled
            p0 = p1
        self.profiling_checkpoint("distances")

        for x in ["delta_t", "delta_vel", "vel", "ay", "ax", "power_into_inverter", "motor_torque", "motor_efficiency"]:
            track_points[x] = 0

        initial_forward_sim_df = self.__forward_sim(track_points.copy(deep = True), 20, False)
        if self.track.track_type == "endurance":
            starting_velocity = initial_forward_sim_df["vel"].iloc[initial_forward_sim_df.shape[1]]
            ending_velocity = starting_velocity
        else:
            starting_velocity = 0
            ending_velocity = initial_forward_sim_df["vel"].iloc[initial_forward_sim_df.shape[1]]
        self.profiling_checkpoint("initial forward sim")

        self.reverse_sim_results = self.__forward_sim(track_points.copy(deep = True), ending_velocity, True)
        self.profiling_checkpoint("reverse sim")

        self.forward_sim_results = self.__forward_sim(track_points.copy(deep = True), starting_velocity, False)
        self.profiling_checkpoint("forward sim")
    
        ### COMBINE RESULTS
        time, rows = 0, []
        for index, row in track_points.iterrows():
            reverse_row = self.reverse_sim_results.loc[index]
            forward_row = self.forward_sim_results.loc[index]
            if (forward_row["vel"] - reverse_row["vel"]) < 0:
                time += forward_row.loc["delta_t"]
                rows.append(np.array([*forward_row, time]))
            else:
                time += reverse_row.loc["delta_t"]
                rows.append(np.array([*reverse_row, time]))
        
        self.results = pd.DataFrame(rows, columns=[*self.reverse_sim_results.columns, "time"])
        self.time = self.results["time"].max()
        self.profiling_checkpoint("combine sims")

        self.finish_profiling()

        return self.results, self.time
    
    def __skidpad_sim(self):
        path_radius = 7.75 + self.car.params.front_track/2 + .1524
        speed = self.car.max_vel_corner(path_radius) # speed to possible to take
        time = path_radius * 2 * np.pi / speed
        return time

    def __forward_sim(self, track, starting_vel, is_reverse):
        accel_func = self.car.accel_backward if is_reverse else self.car.accel_forward
        vel = starting_vel

        for i, row in (track[::-1] if is_reverse else track).iterrows():
            vmax = self.car.max_vel_corner(row["R"])
            track.loc[i ,"max_vel"] = vmax
            AY = self.car.lateral(self.car.max_vel()) # accel capabilities
            track.loc[i,"ay"] = min(vel**2/row["R"], AY) if row["R"] != 0 else 0 # actual accel
            if vel < vmax:
                track.loc[i,"ax"], track.loc[i,"power_into_inverter"], track.loc[i, "motor_torque"], track.loc[i ,"motor_efficiency"] = accel_func(vel, track.loc[i,"ay"])
                roots = np.roots([0.5*track.loc[i,"ax"], vel, -row["dist"]])
                track.loc[i,"delta_t"] = min(roots) if min(roots) > 0 else max(roots)
                track.loc[i,"delta_vel"] = min(track.loc[i,"ax"]*track.loc[i,"delta_t"], vmax-vel)
                track.loc[i,"vel"] = vel + track.loc[i,"delta_vel"]
                vel = track.loc[i,"vel"]
            else:
                # car maxed out, no more accel
                vel = vmax
                track.loc[i,"vel"] = vel
                track.loc[i,"delta_t"] = row["dist"]/vel
                track.loc[i,"ax"], track.loc[i,"delta_vel"] = 0, 0
        if is_reverse:
            track["ax"]*=-1
        return track

def curvature(track_df):
    # radius of curvature and curvature vector for 2D or 3D curve
    # X - x,y column array
    # [L, R, k] = Cumulative arc length, radius of curvature, curvature vector

    track_df["L"], track_df["R"], track_df["kx"], track_df["ky"], track_df["kz"] = 0, 0, 0, 0, 0
    coords_df = track_df[["x", "y", "z"]]

    previous_row = track_df.iloc[-1]
    previous_coords, this_coords = coords_df.iloc[-1], coords_df.iloc[0]

    for i, row in track_df.iloc[:-1].iterrows():
        next_coords = coords_df.iloc[i+1]
        track_df.loc[i,"R"], _, k = circumcenter(this_coords, previous_coords, next_coords)
        track_df.loc[i,"kx"], track_df.loc[i,"ky"], track_df.loc[i,"kz"] = k
        track_df.loc[i,"L"] = previous_row["L"] + np.linalg.norm(this_coords - previous_coords)
        previous_row = row
        previous_coords, this_coords = this_coords, next_coords
    
    N = track_df.shape[0]
    track_df.loc[N-1,"R"], _, k = circumcenter(previous_coords, coords_df.iloc[N-1], coords_df.iloc[0])
    track_df.loc[N-1,"kx"], track_df.loc[N-1,"ky"], track_df.loc[N-1,"kz"] = k
    track_df.loc[N-1,"L"] = track_df["L"].iloc[N-2] + np.linalg.norm(coords_df.iloc[N-1] - previous_coords)
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