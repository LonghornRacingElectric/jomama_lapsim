from operator import mul
from engine.magic_moment_method.ggv_generator import ggv_generator
import pandas as pd

class Racecar:
    def __init__(self, racecar, existing_ggv_file = None):
        self.ggv = pd.read_csv(existing_ggv_file) if existing_ggv_file else None
        self.params = racecar

    def regenerate_GGV(self, sweep_range, mesh):
        self.ggv = ggv_generator(self.params, sweep_range, mesh)
    
    def save_ggv(self, file_target):
        self.ggv.to_csv(file_target)

    def accel(self, vel, lateral):
        # Find closest velocities
        closest_vels = self.__get_nearest_velocities(vel)

        # if velocity is exact
        if closest_vels[1] is None:
            velocity_slice = self.ggv[self.ggv["s_dot"] == vel]
            return self.__interpolate_long_accel(velocity_slice, lateral, True)
        
        # for nearest velocities, find long acceleration for given lateral acceleration

        max_long_accels = [1, 2]

        # do weighted averaging of long accel by closeness to velocity
        weight = abs(vel - closest_vels[0])/(abs(vel - closest_vels[0]) + abs(vel - closest_vels[1]))
        long_accel_interpolated = max_long_accels[0] * (1 - weight) + max_long_accels[1] * weight

        return long_accel_interpolated

    def __interpolate_long_accel(self, velocity_slice_hull, lateral_accel, is_forward):
        # step 1: find all intersection segments
        exactmatch = velocity_slice_hull[velocity_slice_hull["vehicle_accelerations_NTB_1"] == lateral_accel]
        if not exactmatch.empty:
            return exactmatch["vehicle_accelerations_NTB_0"]
        else:
            previous_point = velocity_slice_hull.iloc[-1]
            for index, point in velocity_slice_hull.iterrows():
                # check for intersection
                if ((previous_point["vehicle_accelerations_NTB_1"] > lateral_accel and point["vehicle_accelerations_NTB_1"] < lateral_accel) or
                    (previous_point["vehicle_accelerations_NTB_1"] < lateral_accel and point["vehicle_accelerations_NTB_1"] > lateral_accel)):
                    
            
        # step 2: inerpolate from intersection points, calculated long accel given lateral accel

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
            lower_vel = max(lower_than)
            upper_vel = min(greater_than)
        return [lower_vel, upper_vel]

    def deccel(self, vel, lateral = 0):
        deccel_max = self.params.temp_deccel_max
        return (deccel_max**2 - lateral**2*deccel_max**2/self.lateral(15)**2)**0.5

    def lateral(self, vel):
        # Find closest velocities
        closest_vels = self.__get_nearest_velocities(vel)
        if closest_vels[1] is None:
            return self.ggv[self.ggv["s_dot"] == vel]["vehicle_accelerations_NTB_1"].max() / 9.81
        # get max lateral acceleration for those velocities
        max_lateral_accels = [self.ggv[self.ggv["s_dot"] == swept_vel]["vehicle_accelerations_NTB_1"].max() for swept_vel in closest_vels]
        # weight max lateral accel based on closeness of velocity
        weight = abs(vel - closest_vels[0])/(abs(vel - closest_vels[0]) + abs(vel - closest_vels[1]))
        lateral_accel_interpolated = max_lateral_accels[0] * (1 - weight) + max_lateral_accels[1] * weight
        return lateral_accel_interpolated / 9.81

    def max_vel_corner(self, radius):
        if radius > 1000 or radius == 0:
            return self.params.max_vel
        vel = (self.lateral(15) * 9.81 * radius) ** 0.5
        return vel