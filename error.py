import cv2
import numpy as np
from random import random
import math

def getSimplePathError(path, robotPath, N=1):
    totalError = 0.0
    for i in range(len(path) - 1):
        totalError += getSimpleError(path[i], path[i+1], robotPath[i])[2]
    
    totalError += getSimpleError(path[i], path[1], robotPath[i])[2]    
    return totalError/N

def getSimpleError(startPoint, endPoint, robotPos):
    start, end, robot= np.array(startPoint), np.array(endPoint), np.array(robotPos)
    middle = (start + end)/ 2.0
    #cv2.circle(src, (int(middle[0]),int(middle[1])), 3, (255, 255, 255), 1)
    errorS = np.sqrt(((start - robot) ** 2).sum(0))
    errorE = np.sqrt(((end - robot) ** 2).sum(0))
    errorM = np.sqrt(((middle - robot) ** 2).sum(0))
    return errorS, errorE, errorM
    
def tD2I(tup):
    return (int(tup[0]), int(tup[1]))

def getPerimeterError(startPoint, endPoint, robotPath, src=''):
    path = [startPoint] + robotPath + [endPoint]
    pathNp = np.array(path)
    totalError = 0.0
    for i in range(len (pathNp) - 1):
        totalError += np.sqrt(((pathNp[i] - pathNp[i+1]) ** 2).sum(0))
        if src is not '':
            cv2.arrowedLine(src, tD2I(path[i]), tD2I(path[i+1]), (255,255,255))
    
    return totalError
    

if __name__ == '__main__':

    height, width = 480, 640
    radius = 150
    angleShift = math.radians(20)
    src = np.zeros(shape=(height, width, 3))
    center = (width/2, height/2)
    cv2.circle(src, center, radius, (0,0,255),1)
    path = []
    robotPath = []
    
    for i in np.arange(0,  angleShift * 2, angleShift):
        x = (width/2 + math.cos(i) * radius)
        y = (height/2 + math.sin(i) * radius) 
        cv2.circle(src, (int(x), int(y)), 1, (255, 0, 0), 1)
        
        xn = (width/2 + math.cos(i + angleShift/4.0) * radius)  
        yn = (height/2 + math.sin(i + angleShift/4.0) * radius)  
        xr = xn + (random() - 0.5) * radius/2.5
        yr = yn + (random() - 0.5) * radius/2.5
        robotPath.append((xr, yr))
        cv2.circle(src, (int(xr), int(yr)), 1, (0, 255, 0), 1)
        
        xn = (width/2 + math.cos(i + angleShift/2.0) * radius)  
        yn = (height/2 + math.sin(i + angleShift/2.0) * radius)  
        xr = xn + (random() - 0.5) * radius/2.5
        yr = yn + (random() - 0.5) * radius/2.5
        robotPath.append((xr, yr))
        cv2.circle(src, (int(xr), int(yr)), 1, (0, 255, 0), 1)
        
        path.append((x,y))
        
    print (getPerimeterError(path[0], path[1], robotPath, src))
    
    cv2.namedWindow("ErrorWin", cv2.WINDOW_NORMAL)
    cv2.imshow("ErrorWin",src);
    cv2.waitKey(0);
    cv2.destroyAllWindows();