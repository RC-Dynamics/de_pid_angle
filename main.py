import socket
import json
from robot import Robot
from path import Path
from pid import PIDAngle

HOST = '127.0.0.1'        # Local host
PORT = 50007              # Arbitrary port

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(1)

print ('Waiting for connection...')
conn, addr = s.accept()
robot = Robot()
pid = PIDAngle(1, 0, 0, 30)
path = PathError(40)


print ('Connected by client', addr)
while 1:
	data = conn.recv(1024)
	if not data: break
	# print ('Received data from client', repr(data), '...send it back...')
	
	#Put DE HERE
	for i in path.getPath():
		recData = json.loads(data)
		robot.setCoord(recData['robot0']['x'],recData['robot0']['y'],recData['robot0']['z'])
		
		x, y, theta = robot.getCoord()
		pid.calcSpeeds(x, y, theta, i[0], i[1])
		robot.setVel(pid.getLeftSpeed(),pid.getRightSpeed())
		
		#Sending vel to simulation
		d = robot.getVel()
		conn.send(d.encode())

conn.close()
print ('Server closed.')