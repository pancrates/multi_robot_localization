import numpy as np
import scipy
from scipy import stats
import math
import copy
import random
def norm_ang(angle):
   return (angle + np.pi) % (2 * np.pi) - np.pi

class Robot:
    def __init__(self,pos=None):
        if pos==None:
            self.x = 0
            self.y = 0
            self.phi = 0
        else:
            self.x = pos['x']
            self.y = pos['y']
            self.phi = pos['phi']

        self.distances = None
        self.naigbours = []
        #self.grid = np.zeros((1000,1000)) ## 0 unknown, -1 empty , 1 full
        self.MCL = MCL(20,pos)
        self.cmd_vel = {"u":0,"w":0}


    def update_sensor_readings(self,readings):
        self.distances = readings

    #def update_grid(self):


    def update_naigbours(self,naigbours):
        self.naigbours = naigbours


    def get_robot_position(self):
        return {"x":self.x,"y":self.y,"phi":self.phi}


    def move(self):
        #cmd_vel =  self.wander()
        cmd_vel =  self.straight()
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

    def kinematics(self,u,w,phi=None):
        if phi==None:
            phi =self.phi
        dx =  math.cos(phi)*u
        dy =  math.sin(phi)*u
        dPhi = w
        return {"dx":dx,"dy":dy,"dPhi":dPhi}

    def wander(self):
        #u = self.cmd_vel["u"] + np.random.normal(loc=0,scale=0.5)
        #w = self.cmd_vel["w"] + np.random.normal(loc=0,scale=np.pi/2)
        u = np.random.normal(loc=0.5,scale=0.5)
        w = np.random.normal(loc=0, scale=np.pi/2)
        return {"u":u,"w":w}

    def straight(self):
        #u = self.cmd_vel["u"] + np.random.normal(loc=0,scale=0.5)
        #w = self.cmd_vel["w"] + np.random.normal(loc=0,scale=np.pi/2)
        u = 0.1 #np.random.normal(loc=0.5,scale=0.5)
        w = 0 #np.random.normal(loc=0, scale=np.pi/2)
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
        dx = math.cos(self.phi)*u
        dy = math.sin(self.phi)*u
        dPhi = w
        return {"dx":dx,"dy":dy,"dPhi":dPhi}

    def update_position(self,u,w):
        deltas = self.kinematics(u,w)
        self.x = self.x     + deltas["dx"] + np.random.normal(loc=0,scale=0.1)
        self.y = self.y     + deltas["dy"] + np.random.normal(loc=0,scale=0.1)
        self.phi = self.phi + deltas["dPhi"] + np.random.normal(loc=0,scale=0.1)


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
            p.phi = norm_ang(p.phi)
            self.phi = norm_ang(self.phi)
            dx = (p.x-self.x)
            dy = (p.x-self.y)
            dr = np.sqrt(dx**2 + dy**2) - dmsg["r"]
            dTheta = norm_ang(np.arctan2(dy,dx) - (p.phi + dmsg["theta"]))
            dPhi = norm_ang(np.pi - p.phi - self.phi + dmsg["theta"] - dmsg["theta2"])
            sample = np.array([dr,dTheta,dPhi])
            s=0.1
            prob = stats.multivariate_normal.pdf(sample,mean=np.array([0,0,0]),cov=np.array([[s,0,0],[0,s,0],[0,0,4*s]]))

            #print(prob)
            w+=prob
        return w/len(dmsg["particles"])

class MCL:
    def __init__(self,particleN=10,pos=None):
        self.particle_count = particleN
        self.particles = []
        for n in range(particleN):
            if pos == None:
                x = np.random.uniform(low=-3,high=3)
                y = np.random.uniform(low=-3,high=3)
                phi = np.random.uniform(low=-np.pi,high=np.pi)
                self.particles.append(Particle(n,x=x,y=y,phi=phi))
            else:
                x = pos['x'] + np.random.normal(loc=0,scale=0.1)
                y = pos['y'] + np.random.normal(loc=0,scale=0.1)
                phi = pos['phi'] + np.random.normal(loc=0,scale=0.1)
                self.particles.append(Particle(n,x=x,y=y,phi=phi))


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
        if len(dmsgs) ==0:
            print("apply_detection_model NO MESSAGES")
            return 1
        for i,p in enumerate(self.particles):
            w = p.update_detection_weight(dmsgs)
            print("New particle w: ",w)
            sum_w +=w
            p.w *= w

    def simple_sampling(self):
        sum_w = 0
        for i,p in enumerate(self.particles):
            sum_w += p.w

        weights =[p.w for p in self.particles]
        ids = range(len(weights))
        print("len w",len(weights))
        print("len ids",len(ids))
        #print(weights)
        new_ids = random.choices(ids, weights=weights, k=len(weights))
        print("weights ",weights)
        print("IDS ",new_ids)
        new_particles = []
        for i,p in enumerate(new_ids):
            new_p = copy.deepcopy(self.particles[p])
            new_p.id =i
            new_p.w=1
            #if i <= (len(weights)/10):
            #    new_p = Particle(i,x=np.random.uniform(low=-15,high=15),y=np.random.uniform(low=-15,high=15),phi=np.random.uniform(low=-np.pi,high=np.pi))
            new_particles.append(new_p)
        self.particles = new_particles
        return self.particles


    ### FIX increases number of paricles
    def reciprocal_sample(self,dmsgs):
        new_particles = []
        for i,dmsg in enumerate(dmsgs):
            for j,p in enumerate(dmsg['particles']):
                mean = self.cluster_mean(particles=dmsg["particles"])
                #r2_x = mean["mean_x"]
                #r2_y = mean["mean_y"]
                #r2_phi = mean["mean_phi"]
                r2_x = p.x
                r2_y = p.y
                r2_phi = p.phi
                thetaR = dmsg["theta"]
                thetaA = norm_ang(r2_phi+thetaR)
                new_x = r2_x+np.cos(thetaA)
                new_y = r2_y+np.sin(thetaA)
                new_phi = norm_ang(np.pi- thetaA)
                new_particles.append(Particle(j,x=new_x,y=new_y,phi=new_phi))  
        new_ps = random.choices(new_particles, k=len(self.particles))
        self.particles = copy.deepcopy(new_ps)

    def cluster_mean(self,particles=None):
        if particles == None:
            particles = self.particles
        mean_x =0
        mean_y =0
        mean_phi=0
        for i,p in enumerate(particles):
            mean_x+=p.x
            mean_y+=p.y
            mean_phi+=p.phi
        n =len(particles)
        return {"mean_x":mean_x/n,"mean_y":mean_y/n,"mean_phi":mean_phi/n}

#r = Robot()
#r.drawDDA(2,5,10,200)



