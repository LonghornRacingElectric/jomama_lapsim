import pandas as pd
import numpy as np
import engine

def main():
    easy_driver = engine.Racecar("")

    endurance_track = engine.Track("endurance_michigan_2019", 1, 1)
    autocross_track = engine.Track("autocross_michigan_2019", 1, 1)
    skidpad_times = [1, 1]
    accel_times = [1, 1]

    ### Example: Sweeping! ###
    
    # cg_heights = [12,13,14]
    # results_df = pd.DataFrame(columns=["cg_height","points", "endurance_time"])
    # for index, cg_height in enumerate(cg_heights):
    #     easy_driver.params.cg_height = cg_height
    #     easy_driver.regenerate_GGV()

    #     # If a new racing line is needed (i.e. trackwidth changes), enable these lines
    #     #endurance_track.regenerate_racing_line(easy_driver.params.front_trackwidth)
    #     #autocross_track.regenerate_racing_line(easy_driver.params.front_trackwidth)

    #     _, points, times = engine.Competition(endurance_track, autocross_track, easy_driver).run()
    #     results_df.loc[index] = [cg_height, points, times[0]]
    # results_df.to_csv("results/cg_height_sweep-easy_driver.csv")


    ### Example: One-off simulations ###

    results, points, times = engine.Competition(easy_driver, endurance_track, autocross_track,
                             skidpad_times, accel_times).run()
    results[0].to_csv("results/endurance_michigan_2019-easy_driver.csv")
    results[1].to_csv("results/autocross_michigan_2019-easy_driver.csv")
    print(times)
    

if __name__ == "__main__":
    main()