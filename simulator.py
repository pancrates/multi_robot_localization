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
            com_vel = robot.move()
            robot.MCL.apply_motion_model(com_vel["u"],com_vel["w"])
            particles = robot.MCL.get_particles()
            end_internal_pos = robot.get_robot_position()
            print("Start ",start_internal_pos)
            print("End ",end_internal_pos)
            return {"new_position": end_internal_pos, "particles":particles}

    def no_env_sim(self):
        history = []
        for n in range(100):
            for i,r in enumerate(self.robotList):
                h = self.step(r)
                history.append(h)
        self.plot_path(history)

    def plot_path(self,history):
        xs = [h["new_position"]["x"] for h in history]
        ys = [h["new_position"]["y"] for h in history]
        particle_history = [h["particles"] for h in history]
        fig, ax = plt.subplots()  # Create a figure containing a single axes.
        ax.plot(xs, ys)  # Plot some data on the axes
        for ps in particle_history:
            print(ps)
            x_positions = [p.x for p in ps]
            y_positions = [p.y for p in ps]
            ax.scatter(x_positions, y_positions,s=1)  # Plot some data on the axes
        plt.show()



bsim = BlindSim()
bsim.no_env_sim()
