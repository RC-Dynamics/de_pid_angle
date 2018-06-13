import math
import collections as col
class PID:
    def __init__(self, kp, ki, kd, vl):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.vl = vl
        self.lastError = 0
        self.errorSum = 0
        self.leftSpeed = self.vl
        self.rightSpeed = self.vl
        self.dq = col.deque(maxlen = 5)

    def getError(self, rx, ry, rtheta, x, y):
        r2line = math.degrees(math.atan2(y - ry, x - rx))
        if(r2line < 0):
            r2line = 360 + r2line
        error = r2line - rtheta
        if(error < -180):
            error = 360 + error
        if(error > 180):
            error = error - 360
        return error
        
   
    def calcSpeeds(self, rx, ry, rtheta, x, y):
        error = self.getError(rx, ry, rtheta, x ,y)
        correction = kp * error + ki * self.errorSum + kd * (error - self.lastError)
        self.leftSpeed = self.vl + correction
        self.rightSpeed = self.vl - correction
        self.lastError = error
        self.dq.append(error)
        self.errorSum = sum(self.dq)
    
    def getLeftSpeed():
        return self.leftSpeed
    
    def getRightSpeed():
        return self.rightSpeed

def main():
    pidAngle = PID(1, 0, 0, 0)
    for i in range(36):
        ang = i*10
        print ("{}  {}".format(pidAngle.getError(0.0, 0.0, 90, math.cos(math.radians(ang)), math.sin(math.radians(ang))), ang))

if __name__ == "__main__":
    main()

