import math
import collections as col
class PIDAngle:
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
        # print ("line: {0:.2f}  robot: {1:.2f}  erro:{2:.2f}".format(r2line, rtheta, error))
        return error
        
    def setVl(self, vl):
        self.vl = vl

    def setK(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
   
    def calcSpeeds(self, rx, ry, rtheta, x, y):
        error = self.getError(rx, ry, rtheta, x ,y)
        correction = self.kp * error + self.ki * self.errorSum + self.kd * (error - self.lastError)
        self.leftSpeed = self.vl + correction
        self.rightSpeed = self.vl - correction
        self.lastError = error
        self.dq.append(error)
        self.errorSum = sum(self.dq)
        return error
    
    def getLeftSpeed(self):
        return self.leftSpeed
    
    def getRightSpeed(self):
        return self.rightSpeed

def main():
    pidAngle = PID(1, 0, 0, 0)
    for i in range(36):
        ang = i*10
        print ("{}  {}".format(pidAngle.getError(0.0, 0.0, 90, math.cos(math.radians(ang)), math.sin(math.radians(ang))), ang))

# if __name__ == "__main__":
#     main()

