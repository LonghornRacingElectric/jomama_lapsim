import engine

class Competition:
    def __init__(self, endurance, autocross, racecar):
        self.racecar = racecar

        self.endurance_sim = engine.Simulation(endurance, self.racecar)
        self.autocross_sim = engine.Simulation(autocross, self.racecar)
        self.skidpad_sim = None
        self.acceleration_sim = None

    def run(self):
        print("Running Endurance Sim")
        result_endurance, time_e = self.endurance_sim.run()
        print("Running Autocross Sim")
        results_autocross, time_ax = self.autocross_sim.run()
        print("Running Skidpad Sim")
        results_skidpad, time_s = None, None #Simulation(self.skidpad, self.racecar).run()
        print("Running Acceleration Sim")
        results_acceleration, time_a = None, None #Simulation(self.acceleration, self.racecar).run()
        
        times = [time_e, time_ax, time_s, time_a]
        points = self.points(*times)

        return [result_endurance, results_autocross, results_skidpad, results_acceleration], points, times

    def points(self, time_e, time_ax, time_s, time_a):
        # TODO
        return 400