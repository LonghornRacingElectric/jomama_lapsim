import engine

class Competition:
    def __init__(self, endurance, autocross, racecar):
        self.racecar = racecar

        self.endurance_sim = engine.Simulation(endurance, self.racecar)
        self.autocross_sim = autocross
        self.skidpad_sim = None
        self.acceleration_sim = None

    def run(self):
        result_endurance, time_e = self.endurance_sim.run()
        results_autocross, time_ax = None, None #Simulation(self.autocross, self.racecar).run()
        results_skidpad, time_s = None, None #Simulation(self.skidpad, self.racecar).run()
        results_acceleration, time_a = None, None #Simulation(self.acceleration, self.racecar).run()
        
        times = [time_e, time_ax, time_s, time_a]
        points = self.points(*times)

        return [result_endurance, results_autocross, results_skidpad, results_acceleration], points, times

    def points(self, time_e, time_ax, time_s, time_a):
        best = self.endurance_sim.track.best_time
        worst = self.endurance_sim.track.worst_time
        return 400