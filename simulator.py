from robot import Robot
import matplotlib.pyplot as plt

terrain_data = [
    [0,0,0,0,0,0,0,0,0,0],
    [0,0,0,0,0,1,0,1,0,0],
    [0,0,1,0,0,1,0,1,0,0],
    [0,0,1,0,0,1,0,1,0,0],
    [0,0,1,0,0,1,0,1,0,0],
    [0,0,1,0,0,1,0,1,0,0],
    [0,0,1,0,0,1,0,1,0,0],
    [0,0,1,0,0,1,0,1,0,0],
    [0,0,1,1,1,1,0,1,0,0],
    [0,0,0,0,0,0,0,0,0,0],
]

class Terrain:
    def __init__(self, grid):
        self.grid = np.array(grid)

    def get_distance(self, x, y, phi):
        grid = self.grid
        width, height = grid.shape
        dx, dy = math.cos(phi), math.sin(phi)
        dsx = 1 if dx > 0 else -1
        dsy = 1 if dy > 0 else -1
        tx = math.floor(x) if dx > 0 else (math.ceil(x) - 1)
        ty = math.floor(y) if dy > 0 else (math.ceil(y) - 1)
        dtx = (tx - x + (1 if dx > 0 else 0)) / dx if dx != 0 else np.inf
        dty = (ty - y + (1 if dy > 0 else 0)) / dy if dy != 0 else np.inf
        ddtx = dsx / dx if dx != 0 else np.inf
        ddty = dsy / dy if dy != 0 else np.inf
        t = 0
        while tx >= 0 and tx < width and ty >= 0 and ty < height:
            if grid[ty,tx]:
                break
            if dtx < dty:
                tx += dsx
                t += dtx
                dty -= dtx
                dtx = ddtx
            else:
                ty += dsy
                t += dty
                dtx -= dty
                dty = ddty
        return t



class Sim:
    def __init__(self):
        self.timeStep = 0.01
        self.robotList = []


class BlindSim():
    def __init__(self):
        self.dt = 0.1
        self.robotList = [Robot()]
        self.history = []
    def step(self,robot):
            start_internal_pos = robot.get_robot_position()
            com_vel = robot.move()
            robot.MCL.apply_motion_model(com_vel["u"],com_vel["w"])
            particles = robot.MCL.get_particles()
            end_internal_pos = robot.get_robot_position()
            print("Start ",start_internal_pos)
            print("End ",end_internal_pos)
            return {"new_position": end_internal_pos, "particles":particles}

    #    def no_env_sim(self):
    #        history = []
    #        for n in range(100):
    #            for i,r in enumerate(self.robotList):
    #                h = self.step(r)
    #                history.append(h)
    #        self.plot_path(history)


    def no_env_sim(self):
        for i,r in enumerate(self.robotList):
            self.history.append({"rid":i,"moves":[],"particles":[]})
        for n in range(100):
            for i,r in enumerate(self.robotList):
                h = self.step(r)
                self.history[i]["moves"].append(h["new_position"])
                self.history[i]["particles"].append(h["particles"])
        print(self.history[0]["moves"])
        self.plot_path()


    def plot_path(self):
        fig, ax = plt.subplots()  # Create a figure containing a single axes.
        for i,r in enumerate(self.robotList):
            xs = [h["x"] for h in self.history[i]["moves"]]
            ys = [h["y"] for h in self.history[i]["moves"]]
            print(xs)
            particle_history = [h[i]["particles"] for h in self.history]
            ax.plot(xs, ys)  # Plot some data on the axes
            for ps in particle_history:
                print(ps)
                x_positions = [p.x for p in ps]
                y_positions = [p.y for p in ps]
                ax.scatter(x_positions, y_positions,s=1)  # Plot some data on the axes
        
        plt.show()



bsim = BlindSim()
bsim.no_env_sim()
