import socket
import json
import time
from robot import Robot
from path import Path
from pid import PIDAngle

def restartRobot():
	data = conn.recv(1024)
	recData = json.loads(data.decode('utf-8'))
	robot.setCoord(recData['robot0']['x'],recData['robot0']['y'],recData['robot0']['z'])
	x, y, theta = robot.getCoord()
	robot.setVel(0,0)
	d = robot.getVel()
	conn.send(d.encode())
	return x,y,theta
def updateRobot(i):
	data = conn.recv(1024)
	recData = json.loads(data.decode('utf-8'))
	robot.setCoord(recData['robot0']['x'],recData['robot0']['y'],recData['robot0']['z'])		
	x, y, theta = robot.getCoord()
	robotPositions[countInt].append((x, y))
	erro = pid.calcSpeeds(x, y, 360-theta, i[0], i[1])
	robot.setVel(pid.getLeftSpeed(),pid.getRightSpeed())
	d = robot.getVel()
	conn.send(d.encode())
	return x,y,theta




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

robotPositions = []
startPoints = []
endPoints = []
startPoints.append((125, 65))
restartFlag = False
path.circleDiscretization(50)

while 1: #DEA a todo gene 
	x, y, theta = restartRobot()
	restartFlag = False
	countInt = 0 
	print ("chegggou")
	# -> 
	
	for i in path.getPath():
		
		start_time = time.time()
		# print(path.pointDistance(i[0], i[1], x, y))
		
		robotPositions.append([])
		
		endPoints.append((i[0], i[1]))
		print(countInt)
		
		# pass start and end point of segment -> Carlos
		while path.pointDistance(i[0], i[1], x, y) > 5 and time.time() - start_time < 0.4:
			#print("robo: {0:.2f}  {1:.2f} -- obj:{2:.2f}  {3:.2f} - erro: {4:.2f} - dist: {5:.2f}".format(x, y, i[0], i[1], erro, path.pointDistance(i[0], i[1], x, y)))
			x,y,theta = updateRobot(i)

			if(path.pointDistance(125, 65, x, y) <= 0.1 and countInt > 0):
				print("Restart!")
				restartFlag = True
				break

		
		if restartFlag == True:
			break
		startPoints.append((i[0], i[1]))
		countInt += 1
	print("Path complete!")
	while (path.pointDistance(125, 65, x, y) >= 0.3):
		x,y,theta = updateRobot(i)
		pass
	#while(path.pointDistance(125, 65, x, y) <= 0.1 and countInt > 0):
#		print("Wait For Restart")#
#		pass
#	break
	#print((startPoints[0]))
	#print((endPoints[0]))
	#print(robotPositions[0])
	#break

conn.close()
print ('Server closed.')

