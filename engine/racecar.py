import types

class Racecar:
    def __init__(self, param_file):
        #self.file = pd.read_csv(ggv_file) #TODO!
        self.max_vel = 29 # m / s
        self.params = types.SimpleNamespace()
        self.params.front_trackwidth = 1.2192
        self.params.temp_accel_max = 1.2
        self.params.temp_deccel_max = 1.8
        self.params.temp_lateral_max = 1.75


    def regenerate_GGV(self):
        pass

    def accel(self, vel, lateral = 0):
        accel_max = self.params.temp_accel_max
        return (accel_max**2 - lateral**2*accel_max**2/self.lateral(0)**2)**0.5

    def deccel(self, vel, lateral = 0):
        deccel_max = self.params.temp_deccel_max
        return (deccel_max**2 - lateral**2*deccel_max**2/self.lateral(0)**2)**0.5

    def lateral(self, vel):
        return self.params.temp_lateral_max

    def max_vel_corner(self, radius):
        if radius > 1000 or radius == 0: # TODO
            return self.max_vel
        vel = (self.lateral(0) * 9.81 * radius) ** 0.5
        return vel