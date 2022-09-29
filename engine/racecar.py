import types

class Racecar:
    def __init__(self, param_file):
        #self.file = pd.read_csv(ggv_file) #TODO!
        self.max_vel = 65
        self.params = types.SimpleNamespace()

    def regenerate_GGV(self):
        pass

    def accel(self, vel, lateral = 0):
        accel_max = 0.75
        return (accel_max**2 - lateral**2*accel_max**2/self.lateral(0)**2)**0.5

    def deccel(self, vel, lateral = 0):
        deccel_max = 2
        return (deccel_max**2 - lateral**2*deccel_max**2/self.lateral(0)**2)**0.5

    def lateral(self, vel):
        return 1.8

    def max_vel_corner(self, radius):
        if radius > 1000: # TODO
            return self.max_vel
        vel = (self.lateral(0) * 32.2 * radius) ** 0.5 * 5280/60**2
        return vel