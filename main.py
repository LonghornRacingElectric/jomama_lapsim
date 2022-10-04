import pandas as pd
import numpy as np
import engine

def main():
    easy_driver = engine.Racecar("Concept2023")

    endurance_track = engine.Track("endurance_michigan_2019", 1681.963)
    autocross_track = engine.Track("autocross_michigan_2019", 63.236)
    skidpad_times = 5.0497
    accel_times = 4.004

    ### Example: Sweeping! ###
    
    
    masses = [400, 500, 600] * .454
    results_df = pd.DataFrame(columns=["mass", "points", "endurance_time"])
    for index, mass in enumerate(masses):
        #easy_driver.params.temp_accel_max = accel
        easy_driver.regenerate_GGV()

        # If a new racing line is needed (i.e. trackwidth changes), enable these lines
        #endurance_track.regenerate_racing_line(easy_driver.params.front_trackwidth)
        #autocross_track.regenerate_racing_line(easy_driver.params.front_trackwidth)

        _, points, times = engine.Competition(easy_driver, endurance_track, autocross_track,
                             skidpad_times, accel_times).run()
        print(times)
        results_df.loc[index] = [mass, points, times[0]]
    results_df.to_csv("results/mass_sweep-easy_driver.csv")


    ### Example: One-off simulations ###

    # results, points, times = engine.Competition(easy_driver, endurance_track, autocross_track,
    #                          skidpad_times, accel_times).run()
    # results[0].to_csv("results/endurance_michigan_2019-easy_driver.csv")
    # results[1].to_csv("results/autocross_michigan_2019-easy_driver.csv")
    # print(times)
    # print(points)

    

if __name__ == "__main__":
    main()