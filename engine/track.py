import pandas as pd

class Track:
    def __init__(self, file_name, best_time, worst_time, load_cached_racing_line = True):
        #self.cones = pd.read_csv("tracks/" + file_name)
        self.racing_line = pd.read_csv("racing_lines/" + file_name + "-racing_line.csv") if load_cached_racing_line else None
        self.best_time = best_time
        self.worst_time = worst_time

    def regenerate_racing_line(self, ggv):
        pass # TODO: Robert?

    def path_points(self):
        return self.racing_line