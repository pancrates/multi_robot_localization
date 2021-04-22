from robot import Robot
import matplotlib.pyplot as plt
import numpy as np
import copy
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
        self.robotList = [Robot(),Robot()]
        self.robotPositions = [{"x":-3,"y":-1,"phi":np.pi},{"x":3,"y":2,"phi":2}]
        self.history = []
        self.detenction_range = 50

    def calc_relative_positions(self,index):
        source = self.robotPositions[index]
        relative_positions = []
        for i, pos in enumerate(self.robotPositions):
            if i == index:
                continue
            else:
                dx = (source["x"]-pos["x"])
                dy = (source["y"]-pos["y"])
                r = np.sqrt(dx**2 + dy**2)
                theta = np.arctan2(dy,dx) - pos["phi"]
                relative_positions.append( {"from":i,"to":index,"r":r,"theta":theta})
        return relative_positions

    def move_step(self,index):
            robot = self.robotList[index]
            start_internal_pos = robot.get_robot_position()
            cmd_vel = robot.move()
            end_internal_pos = robot.get_robot_position()
            #print("Start ",start_internal_pos)
            #print("End ",end_internal_pos)
            return {"cmd_vel":cmd_vel, "new_position": end_internal_pos}


    def update_absolute_position(self,i):
            robot= self.robotList[i]
            print(self.robotPositions[i])
            deltas = robot.kinematics(robot.cmd_vel["u"],robot.cmd_vel["w"])
            self.robotPositions[i]["x"]   +=  deltas["dx"] + np.random.normal(loc=0,scale=0.1)
            self.robotPositions[i]["y"]   +=  deltas["dy"] + np.random.normal(loc=0,scale=0.1)
            self.robotPositions[i]["phi"] +=  deltas["dPhi"] + np.random.normal(loc=0,scale=0.1)
            print(self.robotPositions[i])

    def localize_step(self,index):
            robot = self.robotList[index]
            ### Motion model
            robot.MCL.apply_motion_model(robot.cmd_vel["u"],robot.cmd_vel["w"])
            particles = robot.MCL.get_particles()

            ### bearing and distance
            relative_positions = self.calc_relative_positions(index)
            neigbours = list(filter(lambda x: x["r"] <= self.detenction_range,relative_positions))
            dmsgs = []
            for i, pos in enumerate(neigbours):
                dmsg = copy.deepcopy(pos)
                dmsg["particles"] = self.robotList[pos["from"]].MCL.get_particles()
                print("particles ",len(dmsg["particles"]))
                dmsgs.append(dmsg)
            robot.MCL.apply_detection_model(dmsgs)

            particles = robot.MCL.get_particles()
            return {"neigbours":neigbours, "particles":particles}

    def no_env_sim(self):
        for i,r in enumerate(self.robotList):
            self.history.append({"rid":i,"moves":[],"particles":[],"cmd_vel":[],"neigbours":[],"positions":[]})
        for n in range(100):
            for i,r in enumerate(self.robotList):
                ### Move 
                h = self.move_step(i)
                self.update_absolute_position(i)
                self.history[i]["moves"].append(h["new_position"])
                self.history[i]["cmd_vel"].append(h["cmd_vel"])
                self.history[i]["positions"].append(copy.deepcopy(self.robotPositions[i]))
            for i,r in enumerate(self.robotList):
                l = self.localize_step(i)
                self.history[i]["neigbours"].append(l["neigbours"])
            for i,r in enumerate(self.robotList):
                particles = r.MCL.simple_sampling()
                self.history[i]["particles"].append(l["particles"])
       # print(self.history[0]["moves"])
        self.plot_path()


    def plot_path(self):
        fig, ax = plt.subplots()  # Create a figure containing a single axes.
        for i,r in enumerate(self.robotList):
            absolute_xs = [h["x"] for h in self.history[i]["positions"]]
            absolute_ys = [h["y"] for h in self.history[i]["positions"]]
            #print(xs)
            belief_xs = [h["x"] for h in self.history[i]["moves"]]
            belief_ys = [h["y"] for h in self.history[i]["moves"]]
            #print(xs)

            ax.plot(absolute_xs, absolute_ys)  # Plot some data on the axes
            ax.plot(belief_xs, belief_ys, linestyle = ':')  # Plot some data on the axes
            particle_history = [h for h in self.history[i]["particles"]]
            for ps in particle_history:
                #print(ps)
                x_positions = [p.x for p in ps]
                y_positions = [p.y for p in ps]
                ws =  [1 for p in ps]
                #print(ws)
                ax.scatter(x_positions, y_positions,s=ws)  # Plot some data on the axes

        plt.show()



bsim = BlindSim()
bsim.no_env_sim()
