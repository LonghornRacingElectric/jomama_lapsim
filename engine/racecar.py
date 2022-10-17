from operator import mul
from engine.magic_moment_method.ggv_generator import ggv_generator
import pandas as pd
from scipy.optimize import fsolve

class Racecar:
    def __init__(self, racecar, existing_ggv_file = None):
        self.ggv = pd.read_csv(existing_ggv_file) if existing_ggv_file else None
        self.params = racecar
        self.additional_compute_columns = ["motor_angular_speed", ""]

    def regenerate_GGV(self, sweep_range, mesh):
        self.ggv = ggv_generator(self.params, sweep_range, mesh)
    
    def save_ggv(self, file_target):
        self.ggv.to_csv(file_target)

    def accel(self, vel, lateral, is_forward = True):
        # Find closest velocities
        closest_vels = self.__get_nearest_velocities(vel)

        # if velocity is exact
        if closest_vels[1] is None:
            return self.__interpolate_long_accel(vel, lateral, is_forward)
        
        # for nearest velocities, find long acceleration for given lateral acceleration
        upper_vel_accel = self.__interpolate_long_accel(closest_vels[1], lateral, is_forward)
        lower_vel_accel = self.__interpolate_long_accel(closest_vels[0], lateral, is_forward)

        # do weighted averaging of long accel by closeness to velocity
        weight = abs(vel - closest_vels[0])/(abs(vel - closest_vels[0]) + abs(vel - closest_vels[1]))
        long_accel_interpolated = lower_vel_accel * (1 - weight) + upper_vel_accel * weight

        return long_accel_interpolated

    def __interpolate_long_accel(self, vel, lateral_accel, is_forward):
        # step 1: find all intersection segments
        velocity_slice_hull = self.ggv[self.ggv["s_dot"] == vel]
        exactmatch = velocity_slice_hull[velocity_slice_hull["vehicle_accelerations_NTB_1"] == lateral_accel]
        intersections = []
        if not exactmatch.empty:
            return exactmatch["vehicle_accelerations_NTB_0"]
        else:
            previous_point = velocity_slice_hull.iloc[-1]
            for _, point in velocity_slice_hull.iterrows():
                # check for intersection
                if ((previous_point["vehicle_accelerations_NTB_1"] > lateral_accel and point["vehicle_accelerations_NTB_1"] < lateral_accel) or
                    (previous_point["vehicle_accelerations_NTB_1"] < lateral_accel and point["vehicle_accelerations_NTB_1"] > lateral_accel)):
                    y2, y1 = previous_point["vehicle_accelerations_NTB_1"], point["vehicle_accelerations_NTB_1"]
                    x2, x1 = previous_point["vehicle_accelerations_NTB_0"], point["vehicle_accelerations_NTB_0"]
                    m = (y2-y1)/(x2-x1)
                    b = y1 - m * x1
                    y_in = lateral_accel
                    # y = mx + b
                    x_in = (y_in - b)/m
                    intersections.append(x_in)
                if len(intersections) > 1:
                    break
                previous_point = point
        # step 2: interpolate from intersection points, calculated long accel given lateral accel
        if is_forward:
            if len(intersections) == 0:
                return velocity_slice_hull["vehicle_accelerations_NTB_1"].max()
            return max(intersections)
        else:
            if len(intersections) == 0:
                return velocity_slice_hull["vehicle_accelerations_NTB_1"].min()
            return min(intersections)

    def __get_nearest_velocities(self, vel):
        exactmatch = self.ggv[self.ggv["s_dot"] == vel]
        if not exactmatch.empty:
            return [vel, None]
        else:
            lower_than = []
            greater_than = []
            for swept_vel in self.ggv["s_dot"].unique():
                if vel < swept_vel:
                    greater_than.append(swept_vel)
                else:
                    lower_than.append(swept_vel)
            lower_vel = max(lower_than) if len(lower_than) > 0 else min(greater_than)
            upper_vel = min(greater_than) if len(greater_than) > 0 else max(lower_than)
        return [lower_vel, upper_vel]

    def deccel(self, vel, lateral):
        return self.accel(vel, lateral, False) * -1

    def lateral(self, vel):
        # Find closest velocities
        closest_vels = self.__get_nearest_velocities(vel)
        if closest_vels[1] is None:
            return self.ggv[self.ggv["s_dot"] == vel]["vehicle_accelerations_NTB_1"].max()
        # get max lateral acceleration for those velocities
        max_lateral_accels = [self.ggv[self.ggv["s_dot"] == swept_vel]["vehicle_accelerations_NTB_1"].max() for swept_vel in closest_vels]
        # weight max lateral accel based on closeness of velocity
        weight = abs(vel - closest_vels[0])/(abs(vel - closest_vels[0]) + abs(vel - closest_vels[1]))
        lateral_accel_interpolated = max_lateral_accels[0] * (1 - weight) + max_lateral_accels[1] * weight
        return lateral_accel_interpolated

    def max_vel_corner(self, radius):
        if radius > 90 or radius == 0:
            return self.params.max_vel
        def vel_solver(x):
            velocity = x[0]
            return velocity**2/radius-self.lateral(velocity)
        vel_corner = fsolve(vel_solver, [14])[0]
        return vel_corner if vel_corner < self.params.max_vel else self.params.max_vel