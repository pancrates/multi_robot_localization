import numpy as np
import math 

class Robot:
    def __init__(self,position={x:0,y:0,phi:0}):
        self.position = position
        self.distances = None
        self.grid = np.zeros((1000,1000)) ## 0 unknown, -1 empty , 1 full
        
    
    def update_sensor_readings(self,readings):
        self.distances = readings
    
    #def update_grid(self):
    
    
    def tileCoords(cellSize, x, y):
        return (math.floor(x / cellSize) + 1, math.floor(y / cellSize) + 1)
    
    

    def ROUND(a):
        return int(a + 0.5)

    def drawDDA(x1,y1,x2,y2):
        x,y = x1,y1
        length = (x2-x1) if (x2-x1) > (y2-y1) else (y2-y1)
        dx = (x2-x1)/float(length)
        dy = (y2-y1)/float(length)
        print ('x = %s, y = %s' % (((ROUND(x),ROUND(y)))))
        for i in range(length):
            x += dx
            y += dy
        print ('x = %s, y = %s' % (((ROUND(x),ROUND(y)))))





drawDDA(2,5,10,20)


    
