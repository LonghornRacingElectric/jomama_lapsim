import engine

class Competition:
    def __init__(self, racecar, endurance, autocross, skidpad_times = None, accel_times = None):
        self.racecar = racecar

        accel_track = engine.Track("Acceleration", best_time = accel_times[0], worst_time = accel_times[1])
        skidpad_track = engine.Track(best_time = skidpad_times[0], worst_time = skidpad_times[1])
        
        self.endurance_sim = engine.Simulation(self.racecar, endurance)
        self.autocross_sim = engine.Simulation(self.racecar, autocross)
        self.skidpad_sim = engine.Simulation(self.racecar, skidpad_track, is_skidpad = True)
        self.acceleration_sim = engine.Simulation(self.racecar, accel_track)

    def run(self):
        print("Running Endurance Sim")
        result_endurance, time_e = self.endurance_sim.run()
        print("Running Autocross Sim")
        results_autocross, time_ax = self.autocross_sim.run()
        print("Running Skidpad Sim")
        results_skidpad, time_s = self.skidpad_sim.run()
        print("Running Acceleration Sim")
        results_acceleration, time_a = self.acceleration_sim.run()
        
        times = [time_e, time_ax, time_s, time_a]
        points = self.points(*times)

        return [result_endurance, results_autocross, results_skidpad, results_acceleration], points, times

    def points(self, time_e, time_ax, time_s, time_a):
        # TODO
        return 400