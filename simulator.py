from robot import Robot
import matplotlib.pyplot as plt

class Terrain:
    def get_distance(self, x, y, phi):
        # TODO
        return 1.0



class Sim:
    def __init__(self):
        self.timeStep = 0.01
        self.robotList = []




class BlindSim():
    def __init__(self):
        self.dt = 0.1
        self.robotList = [Robot()]

    def step(self,robot):
            start_internal_pos = robot.get_robot_position()
            robot.move()
            end_internal_pos = robot.get_robot_position()
            print("Start ",start_internal_pos)
            print("End ",end_internal_pos)
            return end_internal_pos

    def no_env_sim(self):
        history = []
        for n in range(20000):
            for i,r in enumerate(self.robotList):
                h = self.step(r)
                history.append(h)
        self.plot_path(history)

    def plot_path(self,history):
        xs = [h["x"] for h in history]
        ys = [h["y"] for h in history]
        fig, ax = plt.subplots()  # Create a figure containing a single axes.
        ax.plot(xs, ys)  # Plot some data on the axes
        plt.show()



bsim = BlindSim()
bsim.no_env_sim()
