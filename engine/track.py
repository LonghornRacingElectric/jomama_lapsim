import pandas as pd

class Track:
    def __init__(self, cached_racing_line = None, best_time = None):
        #self.cones = pd.read_csv("tracks/" + file_name)
        if cached_racing_line:
            self.racing_line = pd.read_csv(cached_racing_line)
        self.best_time = best_time

    def regenerate_racing_line(self, ggv):
        pass # TODO: Robert?

    def path_points(self):
        return self.racing_line