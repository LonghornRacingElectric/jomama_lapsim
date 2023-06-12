from engine.magic_moment_method.solver_sweeper import generate_GGV
import pandas as pd
from scipy.optimize import fsolve

class Racecar:
    def __init__(self, racecar, existing_ggv_file = None):
        if existing_ggv_file:
            self.ggv = pd.read_csv(existing_ggv_file)
            self.prepare_GGV()
        else:
            self.ggv = pd.DataFrame()
        self.params = racecar
        try:
            self.initial_guesses = pd.read_csv("results/GGV_initial_guesses.csv")
        except:
            self.initial_guesses = None

    def recreate_initial_guesses(self, sweep_range, mesh):
        _, df = generate_GGV(self.params, sweep_range, mesh, return_all_points = True)
        columns_to_save = ["heave", "x_double_dot", "y_double_dot", "yaw_acceleration", "roll", "pitch",
                         "front_left_tire_slip_ratio", "front_right_tire_slip_ratio", "rear_left_tire_slip_ratio", "rear_right_tire_slip_ratio",
                         "s_dot", "body_slip", "steered_angle", "torque_request", "is_left_diff_bias"]
        self.initial_guesses = df[columns_to_save]
        self.initial_guesses.to_csv("results/GGV_initial_guesses.csv")
        
    def regenerate_GGV(self, sweep_range, mesh):
        self.ggv = generate_GGV(self.params, sweep_range, mesh, initial_guesses = self.initial_guesses)
        self.prepare_GGV()
    
    def save_ggv(self, file_target):
        self.ggv.to_csv(file_target)

    def prepare_GGV(self):
        self.fast_ggv = {}
        for vel in self.ggv["s_dot"].unique():
            # prepare half-slices
            vel_slice = self.ggv[self.ggv["s_dot"] == vel]
            vel_slice_accel = vel_slice[vel_slice["vehicle_accelerations_NTB_0"] > 0].sort_values(by=["vehicle_accelerations_NTB_1"]).reset_index(drop=True)
            vel_slice_decel = vel_slice[vel_slice["vehicle_accelerations_NTB_0"] <= 0].sort_values(by=["vehicle_accelerations_NTB_1"]).reset_index(drop=True)

            # overlap half-slices for edge interpolation
            if vel_slice_accel.iloc[0]["vehicle_accelerations_NTB_1"] < vel_slice_decel.iloc[0]["vehicle_accelerations_NTB_1"]:
                vel_slice_decel.loc[-1] = vel_slice_accel.iloc[0].copy()
                vel_slice_decel.index += 1
                vel_slice_decel.sort_index(inplace=True) 
            else:
                vel_slice_accel.loc[-1] = vel_slice_decel.iloc[0].copy()
                vel_slice_accel.index += 1
                vel_slice_accel.sort_index(inplace=True) 
            if vel_slice_accel.iloc[-1]["vehicle_accelerations_NTB_1"] > vel_slice_decel.iloc[-1]["vehicle_accelerations_NTB_1"]:
                vel_slice_decel.loc[len(vel_slice_decel)] = vel_slice_accel.iloc[-1].copy()
            else:
                vel_slice_accel.loc[len(vel_slice_accel)] = vel_slice_decel.iloc[-1].copy()
            
            vel_slice_accel = vel_slice_accel.reset_index(drop=True)
            vel_slice_decel = vel_slice_decel.reset_index(drop=True)
            
            # store half-slices and max in dictionary
            self.fast_ggv[vel] = {
                True: vel_slice_accel,
                False: vel_slice_decel,
                "max": vel_slice["vehicle_accelerations_NTB_1"].max()
            }

    def max_vel(self):
        return self.params.max_motor_speed * self.params.rear_tire_radius / self.params.diff_radius * self.params.motor_radius

    def __accel(self, vel, lateral, is_forward = True):
        # Find closest velocities
        closest_vels = self.__get_nearest_velocities(vel)

        # if velocity is exact
        if closest_vels[1] is None:
            return self.__interpolate_long_accel(vel, lateral, is_forward)
        
        # for nearest velocities, find long acceleration for given lateral acceleration
        upper_vel_accel, upper_power, upper_torque, u_e = self.__interpolate_long_accel(closest_vels[1], lateral, is_forward)
        lower_vel_accel, lower_power, lower_torque, l_e = self.__interpolate_long_accel(closest_vels[0], lateral, is_forward)

        # do weighted averaging of long accel by closeness to velocity
        weight = abs(vel - closest_vels[0])/(abs(vel - closest_vels[0]) + abs(vel - closest_vels[1]))
        long_accel_interpolated = lower_vel_accel * (1 - weight) + upper_vel_accel * weight
        power_interpolated = lower_power * (1 - weight) + upper_power * weight
        torque_interpolated = lower_torque * (1 - weight) + upper_torque * weight
        efficiency_interpolated = l_e * (1 - weight) + u_e * weight

        return long_accel_interpolated, power_interpolated, torque_interpolated, efficiency_interpolated
    
    def accel_forward(self, vel, lateral):
        return self.__accel(vel, lateral, True)
    
    def accel_backward(self, vel, lateral):
        accel, _, _, _ = self.__accel(vel, lateral, False)
        return accel * -1, 0, 0, 0

    def __interpolate_long_accel(self, vel, lateral_accel, is_forward):
        def decompose(point):
            return point["vehicle_accelerations_NTB_0"], point["power_into_inverter"], point["motor_torque"], point["motor_efficiency"]

        ggv_half_slice: pd.DataFrame = self.fast_ggv[vel][is_forward]

        lat_accels: pd.Series[float] = ggv_half_slice["vehicle_accelerations_NTB_1"]
        i_upper = lat_accels.searchsorted(lateral_accel)
        
        # check if out of bounds
        if i_upper == 0 or i_upper == len(lat_accels):
            i = min(i_upper, len(lat_accels) - 1)
            return decompose(ggv_half_slice.iloc[i])

        # check for exact match
        if lat_accels.iloc[i_upper] == lateral_accel:
            return decompose(ggv_half_slice.iloc[i_upper])
        
        # interpolate between two nearest points
        i_lower = i_upper - 1
        point_lower = ggv_half_slice.iloc[i_lower]
        point_upper = ggv_half_slice.iloc[i_upper]
        lat_lower = point_lower["vehicle_accelerations_NTB_1"]
        lat_upper = point_upper["vehicle_accelerations_NTB_1"]
        weight = (lateral_accel - lat_lower) / (lat_upper - lat_lower)
        point_inter = (point_lower * (1-weight)) + (point_upper * weight)
        return decompose(point_inter)

    def __get_nearest_velocities(self, vel):
        if vel in self.fast_ggv:
            return [vel, None]
        else:
            lower_than = []
            greater_than = []
            for swept_vel in self.fast_ggv.keys():
                if vel < swept_vel:
                    greater_than.append(swept_vel)
                else:
                    lower_than.append(swept_vel)
            lower_vel = max(lower_than) if len(lower_than) > 0 else min(greater_than)
            upper_vel = min(greater_than) if len(greater_than) > 0 else max(lower_than)
        return [lower_vel, upper_vel]

    def lateral(self, vel):
        if vel < 0:
            return 0
        # Find closest velocities
        closest_vels = self.__get_nearest_velocities(vel)
        if closest_vels[1] is None:
            return self.fast_ggv[vel]["max"]
        # get max lateral acceleration for those velocities
        max_lateral_accels = [self.fast_ggv[swept_vel]["max"] for swept_vel in closest_vels]
        # weight max lateral accel based on closeness of velocity
        weight = abs(vel - closest_vels[0])/(abs(vel - closest_vels[0]) + abs(vel - closest_vels[1]))
        lateral_accel_interpolated = max_lateral_accels[0] * (1 - weight) + max_lateral_accels[1] * weight
        return lateral_accel_interpolated

    def max_vel_corner(self, radius):
        try:
            if radius > 80 or radius == 0:
                return self.max_vel()
            # TODO: FIX THIS BULLSHIT
            if radius < 7:
                radius = 7
            def vel_solver(x):
                velocity = x[0]
                return velocity**2/radius-self.lateral(velocity)
            vel_corner = fsolve(vel_solver, [radius])[0]
        except:
            print(radius)
            raise Exception
        return vel_corner if vel_corner < self.max_vel() else self.max_vel()