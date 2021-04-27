from robot import Robot, Particle, MCL
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


def norm_ang(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

class Sim:
    def __init__(self):
        self.timeStep = 0.01
        self.robotList = []


class BlindSim():
    def __init__(self):
        self.dt = 0.1
        self.robotPositions = [{"x":1,"y":1,"phi":np.pi/3},{"x":-1,"y":-1,"phi":-np.pi-0.002},{"x":-1,"y":-2,"phi":-np.pi-0.002}]
        self.robotList = [Robot(self.robotPositions[0]),Robot(),Robot()]
        self.history = []
        self.detenction_range = 500

    def calc_relative_positions(self,index):
        source = self.robotPositions[index]
        relative_positions = []
        for i, pos in enumerate(self.robotPositions):
            if i == index:
                continue
            else:
                pos["phi"] = norm_ang(pos["phi"])
                source["phi"] = norm_ang(source["phi"])
                dx = (pos["x"]-source["x"])
                dy = (pos["y"]-source["y"])
                r = np.sqrt(dx**2 + dy**2)
                theta = norm_ang(np.arctan2(dy,dx) - pos["phi"])
                theta2 = norm_ang(np.arctan2(-dy,-dx) - source["phi"])
                relative_positions.append( {"from":i,"to":index,"r":r,"theta":theta,"theta2":theta2})
                if(len(relative_positions)) > 2:
                    exit()
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
            deltas = robot.kinematics(robot.cmd_vel["u"],robot.cmd_vel["w"],phi=self.robotPositions[i]["phi"])
            self.robotPositions[i]["x"]   +=  deltas["dx"] + np.random.normal(loc=0,scale=0.01)
            self.robotPositions[i]["y"]   +=  deltas["dy"] + np.random.normal(loc=0,scale=0.01)
            self.robotPositions[i]["phi"] +=  norm_ang(deltas["dPhi"] + np.random.normal(loc=0,scale=0.01))
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
            return {"neigbours":neigbours, "particles":particles,"dmsgs":dmsgs}


    def reciprocal_sample(self,index):
            robot = self.robotList[index]
            relative_positions = self.calc_relative_positions(index)
            neigbours = list(filter(lambda x: x["r"] <= self.detenction_range,relative_positions))
            dmsgs = []
            for i, pos in enumerate(neigbours):
                dmsg = copy.deepcopy(pos)
                dmsg["particles"] = self.robotList[pos["from"]].MCL.get_particles()
                print("particles ",len(dmsg["particles"]))
                dmsgs.append(dmsg)
            robot.MCL.reciprocal_sample(dmsgs)
            particles = robot.MCL.get_particles()
            print("PART LEN ",len(particles), type(particles[0]))
            return  particles




    def no_env_sim(self):
        for i,r in enumerate(self.robotList):
            self.history.append({"rid":i,"moves":[],"particles":[],"cmd_vel":[],"neigbours":[],"positions":[],"corrected":[]})
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
                #self.history[i]["particles"].append(l["particles"])
                #r.MCL.get_prob_map(l["dmsgs"])
            for i,r in enumerate(self.robotList):
                #particles = r.MCL.simple_sampling()
                #particles = self.reciprocal_sample(i)
                if i == 0:
                     particles = r.MCL.simple_sampling()
                #    r.MCL.correct_position(self.robotPositions[i])
                #    particles = copy.deepcopy(r.MCL.particles)
                else:   particles = self.reciprocal_sample(i)
                self.history[i]["particles"].append(particles)

       # print(self.history[0]["moves"])
        self.plot_path()


    def get_corrected_location(self):
        cluster_helper =  MCL(1)
        mean_real = np.zeros((2,len(self.history[0]["positions"])),)
        mean_cloud = np.zeros((2,len(self.history[0]["positions"])))
        for i,r in enumerate(self.robotList):
            absolute_xs = [h["x"] for h in self.history[i]["positions"]]
            absolute_ys = [h["y"] for h in self.history[i]["positions"]]
            mean_real=mean_real+np.vstack((np.array(absolute_xs),np.array(absolute_ys)))
            p_means = [ cluster_helper.cluster_mean(h) for h in self.history[i]["particles"]]
            p_mean_x = [ mean['mean_x'] for mean in p_means]
            p_mean_y = [ mean['mean_y'] for mean in p_means]
            mean_cloud=mean_cloud+np.vstack((np.array(p_mean_x),np.array(p_mean_y)))

        translation = mean_real/len(self.robotList) - mean_cloud/len(self.robotList)
        for i,r in enumerate(self.robotList):
            absolute_xs = [h["x"] for h in self.history[i]["positions"]]
            absolute_ys = [h["y"] for h in self.history[i]["positions"]]
            robot_real = np.vstack((np.array(absolute_xs),np.array(absolute_ys)))
            p_means = [ cluster_helper.cluster_mean(h) for h in self.history[i]["particles"]]
            p_mean_x = [ mean['mean_x'] for mean in p_means]
            p_mean_y = [ mean['mean_y'] for mean in p_means]
            robot_cloud=np.vstack((np.array(p_mean_x),np.array(p_mean_y)))
            robot_final = robot_real-robot_cloud-translation
            #self.history[i]["corrected"].append({"fixed_x":robot_final[0],"fixed_y":robot_final[1]})
            self.history[i]["corrected"].append(robot_final)

    def plot_path(self):
        cluster_helper =  MCL(1)
        fig, ax = plt.subplots()  # Create a figure containing a single axes.
        #mean cez vse robote na vsak timestep 
        # translation_vec =  mead_real - mean_cloud 
        # for each robot real-infer-translateion_vector
        mean_cloud = np.zeros((2,len(self.history[0]["positions"])))
        for i,r in enumerate(self.robotList):
            absolute_xs = [h["x"] for h in self.history[i]["positions"]]
            absolute_ys = [h["y"] for h in self.history[i]["positions"]]
            ax.plot(absolute_xs, absolute_ys)  # Plot some data on the axes
            #print(xs)
            belief_xs = [h["x"] for h in self.history[i]["moves"]]
            belief_ys = [h["y"] for h in self.history[i]["moves"]]
            ax.plot(belief_xs, belief_ys, linestyle = ':')  # Plot some data on the axes
            #print(xs)

            particle_history = [h for h in self.history[i]["particles"]]
            p_means = [ cluster_helper.cluster_mean(h) for h in self.history[i]["particles"]]
            p_mean_x = [ mean['mean_x'] for mean in p_means]
            p_mean_y = [ mean['mean_y'] for mean in p_means]
            print("Len mean x ",len(p_mean_x))
            print("Len particle_history ",len(particle_history))
            print("Len cloud  ",(mean_cloud).shape)
            mean_cloud=mean_cloud+np.vstack((np.array(p_mean_x),np.array(p_mean_y)))

            for ps in particle_history:
                #print(ps)
                x_positions = [p.x for p in ps]
                y_positions = [p.y for p in ps]
                mean = MCL(1).cluster_mean(ps)
                ws =  [1 for p in ps]
                #print(ws)
                cmap = { 0:'k',1:'b',2:'y',3:'g',4:'r' }
                ax.scatter(x_positions, y_positions,s=ws,color=cmap[i])  # Plot some data on the axes
                ax.scatter(mean["mean_x"], mean["mean_y"],s=10,color=("r" if i==1 else "m"))  # Plot some data on the axes
            self.get_corrected_location()

            for i,r in enumerate(self.robotList):
                print(type(self.history[i]["corrected"]))
                print((self.history[i]["corrected"][0]))
                #ax.scatter(self.history[i]["corrected"][0][0], self.history[i]["corrected"][0][1],s=10,color=cmap[i+2])  # Plot some data on the axes


        plt.show()


bsim = BlindSim()
bsim.no_env_sim()
