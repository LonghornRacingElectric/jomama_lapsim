import pandas as pd

class Track:
    def __init__(self, file_name = None, best_time = None, worst_time = None, load_cached_racing_line = True):
        #self.cones = pd.read_csv("tracks/" + file_name)
        if file_name and load_cached_racing_line:
            self.racing_line = pd.read_csv("racing_lines/" + file_name + "-racing_line.csv")
        self.best_time = best_time

    def regenerate_racing_line(self, ggv):
        pass # TODO: Robert?

    def path_points(self):
        return self.racing_line