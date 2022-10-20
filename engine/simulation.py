import numpy as np
import pandas as pd

class Simulation:
    def __init__(self, car, track, is_skidpad = False):
        self.track = track
        self.car = car
        self.is_skidpad = is_skidpad

        self.time = 0
        self.results = None
        self.reverse_sim = None

    def run(self):
        if self.is_skidpad:
            return None, self.__skidpad_sim()

        # get turn radii from raceline points
        track_points = self.track.path_points()

        # find dist traveled for each point on track
        track_points["dist"], track_points["pos"], length_traveled = 0, 0, 0
        previous_row = track_points.iloc[1]
        for i, row in track_points.iloc[1:].iterrows():
            dist = ((row["x"] - previous_row["x"])**2 + (row["y"] - previous_row["y"])**2)**0.5
            track_points.loc[i,"dist"] = dist
            previous_row, length_traveled = row, length_traveled + dist
            track_points.loc[i,"pos"] = length_traveled

        # TODO: temp fix - make it work without this
        track_points = track_points.drop(track_points.shape[0]-1)
        track_points = track_points.drop(0)
        track_points = track_points.drop(1)

        initial_forward_sim_df = self.__forward_sim(track_points.copy(deep = True), 20, False) # TODO: starting speed?
        starting_vel = initial_forward_sim_df["vel"].iloc[initial_forward_sim_df.shape[1]]
        reverse_sim_df = self.__forward_sim(track_points.copy(deep = True), starting_vel, True)
        self.reverse_sim = reverse_sim_df
        forward_sim_df = self.__forward_sim(track_points.copy(deep = True), starting_vel, False)
    
        ### COMBINE RESULTS
        time, rows = 0, []
        for index, row in track_points.iterrows():
            reverse_row = reverse_sim_df.loc[index]
            forward_row = forward_sim_df.loc[index]
            if (forward_row["vel"] - reverse_row["vel"]) < 0:
                time += forward_row.loc["delta_t"]
                rows.append(np.array([*forward_row, time]))
            else:
                time += reverse_row.loc["delta_t"]
                rows.append(np.array([*reverse_row, time]))
        
        self.results = pd.DataFrame(rows, columns=[*reverse_sim_df.columns, "time"])
        self.time = self.results["time"].max()
        return self.results, self.time
    
    def __skidpad_sim(self):
        path_radius = 7.75 + self.car.params.front_track/2 + .1524
        speed = self.car.max_vel_corner(path_radius) # speed to possible to take
        time = path_radius * 2 * np.pi / speed
        return time

    def __forward_sim(self, df, starting_vel, is_reverse):
        accel_func = self.car.deccel if is_reverse else self.car.accel
        vel = starting_vel
        for x in ["delta_t", "delta_vel", "vel", "ay", "ax"]:
            df[x] = 0

        for i, row in (df[::-1] if is_reverse else df).iterrows():
            vmax = self.car.max_vel_corner(row["curvature"])
            AY = self.car.lateral(self.car.params.max_vel) # accel capabilities
            df.loc[i,"ay"] = min(vel**2/row["curvature"], AY) if row["curvature"] != 0 else 0 # actual accel
            if vel < vmax:
                df.loc[i,"ax"] = accel_func(vel, df.loc[i,"ay"])
                roots = np.roots([0.5*df.loc[i,"ax"], vel, -row["dist"]])
                df.loc[i,"delta_t"] = max(roots) if df.loc[i,"ax"] >= 0 else min(roots)
                df.loc[i,"delta_vel"] = min(df.loc[i,"ax"]*df.loc[i,"delta_t"], vmax-vel)
                df.loc[i,"vel"] = vel + df.loc[i,"delta_vel"]
                vel = df.loc[i,"vel"]
            else:
                # car maxed out, no more accel
                vel = vmax
                df.loc[i,"vel"] = vel
                df.loc[i,"delta_t"] = row["dist"]/vel
                df.loc[i,"ax"], df.loc[i,"delta_vel"] = 0, 0
        if is_reverse:
            df["ax"]*=-1
        return df