import socket
import json
import time
from robot import Robot
from path import Path
from pid import PIDAngle


def manhattan_distance(pt1, pt2):
	return abs(pt1[0] - pt2[0]) + abs(pt1[1] - pt2[1]) 



HOST = '127.0.0.1'        # Local host
PORT = 50007              # Arbitrary port

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(1)

print ('Waiting for connection...')
conn, addr = s.accept()
robot = Robot()
pid = PIDAngle(0.1, 0, 3, 20)
path = Path(40)


print ('Connected by client', addr)

data = conn.recv(1024)
recData = json.loads(data)
robot.setCoord(recData['robot0']['x'],recData['robot0']['y'],recData['robot0']['z'])
x, y, theta = robot.getCoord()
robot.setVel(0,0)
d = robot.getVel()
conn.send(d.encode())


robotPositions = []
startPoints = []
endPoints = []
startPoints.append((125, 65))
restartFlag = False
while 1: #DEA
	restartFlag = False
	countInt = 0 
	
	path.circleDiscretization(50)
	for i in path.getPath():
		
		start_time = time.time()
		print(path.pointDistance(i[0], i[1], x, y))
		
		robotPositions.append([])
		
		endPoints.append((i[0], i[1]))
		
		# pass start and end point of segment -> Carlos
		while path.pointDistance(i[0], i[1], x, y) > 5 and time.time() - start_time < 1:
			data = conn.recv(1024)
			recData = json.loads(data)
			robot.setCoord(recData['robot0']['x'],recData['robot0']['y'],recData['robot0']['z'])		
			x, y, theta = robot.getCoord()


			if(path.pointDistance(125, 65, x, y) <= 0.1 and countInt > 0):
				print("Restart!")
				restartFlag = True
				break

			robotPositions[countInt].append((x, y))
			erro = pid.calcSpeeds(x, y, 360-theta, i[0], i[1])
			robot.setVel(pid.getLeftSpeed(),pid.getRightSpeed())
			
			# pass point of robot in this segment -> Carlos

			#print("robo: {0:.2f}  {1:.2f} -- obj:{2:.2f}  {3:.2f} - erro: {4:.2f} - dist: {5:.2f}".format(x, y, i[0], i[1], erro, path.pointDistance(i[0], i[1], x, y)))
			
			#Sending vel to simulation
			d = robot.getVel()
			conn.send(d.encode())
		if restartFlag == True:
			break
		startPoints.append((i[0], i[1]))
		countInt += 1
	#print((startPoints[0]))
	#print((endPoints[0]))
	#print(robotPositions[0])
	#break

conn.close()
print ('Server closed.')


