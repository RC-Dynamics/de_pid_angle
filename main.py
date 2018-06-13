import socket
import json
from robot import Robot

HOST = '127.0.0.1'        # Local host
PORT = 50007              # Arbitrary port

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(1)

print ('Waiting for connection...')
conn, addr = s.accept()
robot = Robot()

print ('Connected by client', addr)
while 1:
	data = conn.recv(1024)
	if not data: break
	# print ('Received data from client', repr(data), '...send it back...')
	recData = json.loads(data)
	robot.setCoord(recData['robot0']['x'],recData['robot0']['y'],recData['robot0']['z'])
	
	#Put DE HERE
	robot.setVel(0,100)

	#Sending vel to simulation
	d = robot.getVel()
	conn.send(d.encode())

conn.close()
print ('Server closed.')