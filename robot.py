import numpy as np
import math


def ROUND(a):
   return int(a + 0.5)


class Robot:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.phi = 0
        self.distances = None
        self.naigbours = []
        self.grid = np.zeros((1000,1000)) ## 0 unknown, -1 empty , 1 full


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

    def kinematics(self,u,w):
        dx = math.cos(w)*u
        dy = math.sin(w)*u
        dPhi = w
        return {"dx":dx,"dy":dy,"dPhi":dPhi}

    def wander(self):
        u = np.random.uniform(low=-np.pi,high=np.pi)
        w = np.random.uniform(low=-np.pi,high=np.pi)
        return {"u":u,"w":w}

    def drawDDA(self,x1,y1,x2,y2):
        x,y = x1,y1
        length = (x2-x1) if (x2-x1) > (y2-y1) else (y2-y1)
        dx = (x2-x1)/float(length)
        dy = (y2-y1)/float(length)
        print ('x = %s, y = %s' % (((ROUND(x),ROUND(y)))))
        for i in range(length):
            x += dx
            y += dy
        print ('x = %s, y = %s' % (((ROUND(x),ROUND(y)))))




#r = Robot()
#r.drawDDA(2,5,10,200)



