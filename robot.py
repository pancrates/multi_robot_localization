import numpy as np
import scipy
from scipy import stats
import math
import copy
import random
def ROUND(a):
   return int(a + 0.5)


class Robot:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.phi = 0
        self.distances = None
        self.naigbours = []
        #self.grid = np.zeros((1000,1000)) ## 0 unknown, -1 empty , 1 full
        self.MCL = MCL(10)
        self.cmd_vel = {"u":0,"w":0}


    def update_sensor_readings(self,readings):
        self.distances = readings

    #def update_grid(self):


    def update_naigbours(self,naigbours):
        self.naigbours = naigbours


    def get_robot_position(self):
        return {"x":self.x,"y":self.y,"phi":self.phi}


    def move(self):
        cmd_vel =  self.wander()
        dPosition = self.kinematics(cmd_vel["u"],cmd_vel["w"])
        self.x = self.x + dPosition["dx"]
        self.y = self.y + dPosition['dy']
        self.phi = self.phi + dPosition["dPhi"]
        if self.phi > np.pi:
            self.phi = -2*np.pi + self.phi

        if self.phi < -np.pi:
            self.phi = 2*np.pi + self.phi
        self.cmd_vel = cmd_vel
        return cmd_vel

    def kinematics(self,u,w):
        dx =  math.cos(w)*u
        dy =  math.sin(w)*u
        dPhi = w
        return {"dx":dx,"dy":dy,"dPhi":dPhi}

    def wander(self):
        #u = self.cmd_vel["u"] + np.random.normal(loc=0.5,scale=0.5)
        #w = self.cmd_vel["w"] + np.random.normal(loc=0, scale=np.pi/2)

        u = np.random.normal(loc=0.5,scale=0.5)
        w = np.random.normal(loc=0, scale=np.pi/2)

        return {"u":u,"w":w}

    #def distance_and_bearing(self,dmsg): 



class Particle:
    def __init__(self,particle_id,x=0,y=0,phi=0):
        self.x = x
        self.y = y
        self.phi = phi
        self.w = 1
        self.id = particle_id

    def kinematics(self,u,w):
        dx = math.cos(w)*u
        dy = math.sin(w)*u
        dPhi = w
        return {"dx":dx,"dy":dy,"dPhi":dPhi}

    def update_position(self,u,w):
        deltas = self.kinematics(u,w)
        self.x = self.x     + deltas["dx"] + np.random.normal(loc=0,scale=0.1)
        self.y = self.y     + deltas["dy"] + np.random.normal(loc=0,scale=0.1)
        self.phi = self.phi + deltas["dPhi"] + np.random.normal(loc=0,scale=0.3)


    ### Here the ray trace callback traditinaly works with the robot generated map
    def update_weight(self,readings,ray_trace_callback,sigma=0.1):
        w = 1
        expected_readings = ray_tracing_callback(self.x,self.y,self.phi)
        for i, r in enumerate(readings):
            prob = scipy.stats.norm.pdf(r,loc=expected_readings[i],scale=sigma)
            w*=prob
        self.w = w
        return w

    def update_detection_weight(self,dmsgs):
        if len(dmsgs) == 0:
            print("no neigbours")
            return 1

        host_r = dmsgs[0]["to"]
        print("PARTICLE ",self.id,"of R ",host_r,"working on particles from ",dmsgs[0]["from"])
        w = 1
        for i,dmsg in enumerate(dmsgs):
            w*= self.weight_of_message(dmsg)
        return w

    def weight_of_message(self,dmsg):
        w = 0
        for i,p in enumerate(dmsg["particles"]):
            dx = (p.x-self.x)
            dy = (p.x-self.y)
            dr = np.sqrt(dx**2 + dy**2) - dmsg["r"]
            dTheta = np.arctan2(dy,dx) - (p.phi + dmsg["theta"])
            #dphi = np.pi - p.phi - self.phi + 
            sample = np.array([dr,dTheta])
            prob = stats.multivariate_normal.pdf(sample,mean=np.array([0,0]),cov=100)
            #print(prob)
            w+=prob
        return w/len(dmsg["particles"])

class MCL:
    def __init__(self,particleN=10):
        self.particle_count = particleN
        self.particles = []
        for n in range(particleN):
            self.particles.append(Particle(n))


    def apply_motion_model(self,u,w):
        for i,p in enumerate(self.particles):
            p.update_position(u,w)

    def get_particles(self):
        return copy.deepcopy( self.particles)

    def apply_sensor_model(self,readings,ray_trace_callback):
        sum_w = 0
        for i,p in enumerate(self.particles):
            w = p.update_weight(readings,ray_trace_callback)
            sum_w +=w
        for p in self.particles:
            p.w = p.w/sum_w

    def apply_detection_model(self,dmsgs):
        sum_w = 0
        for i,p in enumerate(self.particles):
            w = p.update_detection_weight(dmsgs)
            sum_w +=w
        print("SUM w ",sum_w)
        for p in self.particles:
            p.w *= w/sum_w

    def simple_sampling(self):
        sum_w = 0
        for i,p in enumerate(self.particles):
            sum_w += p.w
        weights =[p.w for p in self.particles]
        print(weights)
        new_ps = copy.deepcopy(random.choices(self.particles, weights=weights, k=len(weights)))
        print(new_ps)
        for i,p in enumerate(new_ps):
            p.id=i
            p.w=1
        self.particles = new_ps
        return self.particles

#r = Robot()
#r.drawDDA(2,5,10,200)



