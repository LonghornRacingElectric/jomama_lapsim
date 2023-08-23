import engine
import engine.magic_moment_method.vehicle_params as vehicles
import numpy as np
import pandas as pd

def main():
    # NOTE: this requires a GGV to be pre generated (second input)
    lapsim_racecar = engine.Racecar(vehicles.Concept2024(motor_directory="engine/magic_moment_method/vehicle_params/Eff208.csv"))
    #lapsim_racecar.ggv = pd.read_csv("results/GGV.csv")
    endurance_track = engine.Track("racing_lines/en_mi_2019.csv", 1195, track_type="endurance")
    autocross_track = engine.Track("racing_lines/ax_mi_2019.csv", 42.2)
    skidpad_times = 4.54
    accel_times = 2.95

    # NOTE: guesstimation based from TTC on maximum tire saturation slip angle
    sweep_range = {"body_slip": (-10 * np.pi / 180, 10 * np.pi / 180),
            "steered_angle" : (-18 * np.pi / 180, 18 * np.pi / 180),
            "velocity" : (3, 30),
            "torque_request": (-1, 1),
            "is_left_diff_bias": (True, False)}
    mesh_size = 15 # NOTE: MAKE SURE THIS IS ODD


    ##########################
    ### Example: Sweeping! ###

    #masses = np.array([400, 450, 500, 550, 600]) - 22*4 * .454
    # power_limit = [80000, 70000, 60000]
    # results_df = pd.DataFrame(columns=["power_limit", "points", "times", "endurance_battery_capacity"])
    # for index, p_l in enumerate(power_limit):
    #     lapsim_racecar.params.power_limit = p_l
    #     lapsim_racecar.regenerate_GGV(sweep_range, mesh_size)

    #     # If a new racing line is needed (i.e. trackwidth changes), enable these lines
    #     #endurance_track.regenerate_racing_line(lapsim_racecar.params.front_trackwidth)
    #     #autocross_track.regenerate_racing_line(lapsim_racecar.params.front_trackwidth)

    #     p_l_results, points, times = engine.Competition(lapsim_racecar, endurance_track, autocross_track,
    #                         skidpad_times, accel_times).run()
    #     energy = sum(p_l_results[0]["delta_t"] * p_l_results[0]["power_into_inverter"])
    #     print(times)
    #     print(points)
    #     results_df.loc[index] = [p_l, points, times, energy]
    #     p_l_results[0].to_csv(f"results/p_l_{str(p_l)}-endurance-concept_2023.csv")
    # results_df.to_csv("results/p_l_sweep-concept_2023.csv")


    ####################################
    ### Example: One-off simulations ###

    #lapsim_racecar.ggv = pd.read_csv("results/GGV.csv")
    # lapsim_racecar.regenerate_GGV(sweep_range, mesh_size)
    # lapsim_racecar.save_ggv("results/GGV.csv")
    #endurance_track.regenerate_racing_line(lapsim_racecar.params.front_trackwidth)
    #autocross_track.regenerate_racing_line(lapsim_racecar.params.front_trackwidth)
    # comp = engine.Competition(lapsim_racecar, endurance_track, autocross_track,
    #                        skidpad_times, accel_times)
    # results, points, times = comp.run()
    # results[0].to_csv("results/endurance_michigan_2019-concept_2023.csv")
    # results[1].to_csv("results/autocross_michigan_2019-concept_2023.csv")
    # results[3].to_csv("results/acceleration-concept_2023.csv")
    # # comp.autocross_sim.forward_sim_results.to_csv("results/forward-autocross_michigan_2019-concept_2023.csv")
    # # comp.autocross_sim.reverse_sim_results.to_csv("results/reverse-autocross_michigan_2019-concept_2023.csv")
    # print(times)
    # print(points)



    # 228 gear ratio sweep for pack voltages
    gear_ratio = [4, 4.153846154, 4.307692308, 4.461538462, 4.615384615, 4.769230769, 4.923076923, 5.076923077, 5.230769231, 5.384615385]
    # pack_voltage = 4.2*6*23 #cell voltage * segments * cells in series per segment [V]
    lapsim_racecar.params.max_motor_speed = 7000*(2*np.pi/60) # [rad/s]
    # ***make sure car mass and other parameters for this sweep are correct in the parameter file of the vehicle/configuration used!!
    results_df = pd.DataFrame(columns=["gear_ratio", "points", "times", "endurance_battery_capacity"])
    for index, g_r in enumerate(gear_ratio):
        lapsim_racecar.params.diff_radius = g_r
        lapsim_racecar.params.max_vel = (lapsim_racecar.params.max_motor_speed/g_r)*lapsim_racecar.params.rear_tire_radius
        lapsim_racecar.regenerate_GGV(sweep_range, mesh_size)

        # If a new racing line is needed (i.e. trackwidth changes), enable these lines
        #endurance_track.regenerate_racing_line(lapsim_racecar.params.front_trackwidth)
        #autocross_track.regenerate_racing_line(lapsim_racecar.params.front_trackwidth)

        g_r_results, points, times = engine.Competition(lapsim_racecar, endurance_track, autocross_track,
                            skidpad_times, accel_times).run()
        energy = sum(g_r_results[0]["delta_t"] * g_r_results[0]["power_into_inverter"])
        print(times)
        print(points)
        results_df.loc[index] = [g_r, points, times, energy]
        g_r_results[0].to_csv(f"results/g_r_{str(g_r)}-endurance-concept_2024.csv")
    results_df.to_csv("results/g_r_sweep-concept_2024.csv")



#     # 208 gear ratio sweep for both pack voltages
#     gear_ratio = [3.75, 3.833333333, 3.916666667, 4, 4.083333333, 4.166666667, 4.25,
# 4.333333333, 4.416666667, 4.5, 4.583333333, 4.666666667, 4.75, 4.833333333, 4.916666667,
# 5, 5.083333333, 5.166666667, 5.25, 5.333333333, 5.416666667, 5.5, 5.583333333, 5.666666667,
# 5.75]
#     pack_voltage = 3.6*108 #3.6V*108s
#     lapsim_racecar.params.max_motor_speed = 6000 * (2 * np.pi / 60)
#     lapsim_racecar.params.max_torque = 140
#     lapsim_racecar.params.mass_sprung = 486 * (0.4359) - 2 * lapsim_racecar.params.mass_unsprung_front - 2 * lapsim_racecar.params.mass_unsprung_rear + lapsim_racecar.params.driver_mass # kg
#     results_df = pd.DataFrame(columns=["gear_ratio", "points", "times", "endurance_battery_capacity"])
#     for index, g_r in enumerate(gear_ratio):
#         lapsim_racecar.params.diff_radius = g_r
#         lapsim_racecar.params.max_vel = (lapsim_racecar.params.max_motor_speed/g_r)*lapsim_racecar.params.rear_tire_radius
#         lapsim_racecar.regenerate_GGV(sweep_range, mesh_size)

#         # If a new racing line is needed (i.e. trackwidth changes), enable these lines
#         #endurance_track.regenerate_racing_line(lapsim_racecar.params.front_trackwidth)
#         #autocross_track.regenerate_racing_line(lapsim_racecar.params.front_trackwidth)

#         g_r_results, points, times = engine.Competition(lapsim_racecar, endurance_track, autocross_track,
#                             skidpad_times, accel_times).run()
#         energy = sum(g_r_results[0]["delta_t"] * g_r_results[0]["power_into_inverter"])
#         print(times)
#         print(points)
#         results_df.loc[index] = [g_r, points, times, energy]
#         g_r_results[0].to_csv(f"results/g_r_208_{str(g_r)}-endurance-concept_2023.csv")
#     results_df.to_csv("results/g_r_208_sweep-concept_2023.csv")



# 228 gear ratio sweep for both pack voltages
    # pow_lim = [71000,68000,65000,62000]
    # gear_ratio = 3.75
    # pack_voltage = 3.6*108 #3.6V*108s
    # lapsim_racecar.params.max_motor_speed = (pack_voltage/500) * 5500 * (2 * np.pi / 60)
    # lapsim_racecar.params.max_torque = 230
    # lapsim_racecar.params.mass_sprung = 490 * (0.4359) - 2 * lapsim_racecar.params.mass_unsprung_front - 2 * lapsim_racecar.params.mass_unsprung_rear + lapsim_racecar.params.driver_mass # kg
    # lapsim_racecar.params.diff_radius = gear_ratio
    # lapsim_racecar.params.max_vel = (lapsim_racecar.params.max_motor_speed/lapsim_racecar.params.diff_radius)*lapsim_racecar.params.rear_tire_radius
    # results_df = pd.DataFrame(columns=["power_limit", "points", "times", "endurance_battery_capacity"])
    # for index, p_l in enumerate(pow_lim):
    #     lapsim_racecar.params.power_limit = p_l
    #     lapsim_racecar.regenerate_GGV(sweep_range, mesh_size)

    #     # If a new racing line is needed (i.e. trackwidth changes), enable these lines
    #     #endurance_track.regenerate_racing_line(lapsim_racecar.params.front_trackwidth)
    #     #autocross_track.regenerate_racing_line(lapsim_racecar.params.front_trackwidth)

    #     p_l_results, points, times = engine.Competition(lapsim_racecar, endurance_track, autocross_track,
    #                         skidpad_times, accel_times).run()
    #     energy = sum(p_l_results[0]["delta_t"] * p_l_results[0]["power_into_inverter"])
    #     print(times)
    #     print(points)
    #     results_df.loc[index] = [p_l, points, times, energy]
    #     p_l_results[0].to_csv(f"results/p_l_228_15mesh_{str(p_l)}-endurance-concept_2023.csv")
    # results_df.to_csv("results/p_l_228_sweep_15mesh_-concept_2023.csv")


# # 228 min pack volatage RMS power sim
#     gear_ratio = 3.75
#     pack_voltage = 3.0*108 #3.6V*108s
#     lapsim_racecar.params.max_motor_speed = (pack_voltage/500) * 5500 * (2 * np.pi / 60)
#     lapsim_racecar.params.max_torque = 230
#     lapsim_racecar.params.mass_sprung = 490 * (0.4359) - 2 * lapsim_racecar.params.mass_unsprung_front - 2 * lapsim_racecar.params.mass_unsprung_rear + lapsim_racecar.params.driver_mass # kg
#     lapsim_racecar.params.diff_radius = gear_ratio
#     lapsim_racecar.params.power_limit = 80000
#     lapsim_racecar.params.max_vel = (lapsim_racecar.params.max_motor_speed/lapsim_racecar.params.diff_radius)*lapsim_racecar.params.rear_tire_radius
#     results_df = pd.DataFrame(columns=["points", "times", "endurance_battery_capacity"])

#     lapsim_racecar.regenerate_GGV(sweep_range, mesh_size)
#     lapsim_racecar.save_ggv("results/GGV.csv")
#     # If a new racing line is needed (i.e. trackwidth changes), enable these lines
#     #endurance_track.regenerate_racing_line(lapsim_racecar.params.front_trackwidth)
#     #autocross_track.regenerate_racing_line(lapsim_racecar.params.front_trackwidth)

#     RMS_results, points, times = engine.Competition(lapsim_racecar, endurance_track, autocross_track,
#                         skidpad_times, accel_times).run()
#     energy = sum(RMS_results[0]["delta_t"] * RMS_results[0]["power_into_inverter"])
#     print(times)
#     print(points)
#     results_df = [points, times, energy]
#     RMS_results[0].to_csv(f"results/RMS_228_15mesh_endurance-concept_2023.csv")
#     results_df.to_csv("results/RMS_228_sweep_15mesh_concept_2023.csv")



if __name__ == "__main__":
    main()