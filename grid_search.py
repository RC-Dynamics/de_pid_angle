import socket
import json
import time
from robot import Robot
from path import Path
from pid import PIDAngle
from datetime import datetime
import matplotlib.pyplot as plt


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
	#robotPositions[countInt].append((x, y))
	erro = pid.calcSpeeds(x, 130-y, 360-theta, i[0], 130-i[1])
	# print ("t: {0:.2f}  e: {1:.2f}  x:{2:.2f}   y:{3:.2f}  rx:{4:.2f}  ry:{5:.2f}".format(theta, erro, i[0], i[1], x, y))
	robot.setVel(pid.getLeftSpeed(),pid.getRightSpeed())
	d = robot.getVel()
	conn.send(d.encode())
	return x,y,theta





######################################################################################
import numpy as np 
import random

#from fitness_ackley import fitness_ackley as fitnessFunction
from tqdm import tqdm

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

#while 1: #DEA a todo gene 
def fitnessFunction(kp, ki, kd):
	path.circleDiscretization(50)

	#	print("Trying: Kp: {:0.2f} Ki: {:0.2f} Kd: {:0.2f}".format(kp, ki, kd))
	fitness_error = 0.0
	fitness_error2 = 0.0
	pid.setK(kp, ki, kd)
	robotPositions = []
	startPoints = (125, 65)

	x, y, theta = restartRobot()
	
	while (path.pointDistance(125, 65, x, y) >= 0.3):
		x,y,theta = restartRobot()
	
	x, y, theta = restartRobot()
	countInt = 0 
	pid.setK(kp, ki, kd)
	#print ("chegggou")
	# -> 
	flag = False
	for i in path.getPath():
		start_time = time.time()
		endPoints = (i[0], i[1])
		robotPositions = []
		while path.pointDistance(i[0], i[1], x, y) > 5 and time.time() - start_time < 0.05:

			x,y,theta = updateRobot(i)
			robotPositions.append((x, y))

		fitness_error2 += (path.getPerimeterError(startPoints, endPoints, robotPositions)**2)
		fitness_error += (path.getPerimeterError(startPoints, endPoints, robotPositions))
		
		if (path.pointDistance(125, 65, x, y) >= 0.3 and countInt < 10):
			flag = True

		if (path.pointDistance(125, 65, x, y) <= 0.3 and countInt > 10 and flag):
			print("Error: Robot Reset")
	
		startPoints = (i[0], i[1])
		countInt += 1
		#print (countInt)
	#print("Path complete!, with Fitness = ", fitness_error)
	if countInt != len(path.getPath()):
		print("ERROR: Count Interation Error {}".format(countInt))
	if(fitness_error < 250):
		print("Warring: Fitness error too low! {}".format(fitness_error))
		print("Warring: Set Fitness to Inf")
		fitness_error = 99999999

	return fitness_error, fitness_error2
	#while (path.pointDistance(125, 65, x, y) >= 0.3):
	#	x,y,theta = updateRobot(i)
	#	pass
	#while(path.pointDistance(125, 65, x, y) <= 0.1 and countInt > 0):
#		print("Wait For Restart")#
#		pass
#	break
	#print((startPoints[0]))
	#print((endPoints[0]))
	#print(robotPositions[0])
	#break


class DEA:
	def __init__(self, NP=15, D=2, MaxGen=500, CR=0.90, F=0.7):
		"""
		Attributes:
			NP = Number of Population
			D = Dimentions
			MaxGen = Max generations
			CR = Crossover and Mutation rate
			F = Scaling Factor       
		"""
		self.NP = NP
		self.D = D
		self.MaxGen = MaxGen
		self.CR = CR
		self.F = F
		self.kpMax, self.kdMax = 2, 10
		self.timeStamp = (datetime.now())  
		self.logName = '_'.join(str(x) for x in (self.timeStamp.year,self.timeStamp.month, self.timeStamp.day, self.timeStamp.minute))
		
		self.__init_csv()
		self.__init__population()
		

	def __init__population(self):
		self.population = np.zeros((self.NP, self.D), dtype=np.float)
	
	def __get_fitness(self, genotype):
		return fitnessFunction(genotype[0], 0.0,genotype[1]);

	def __init_csv(self):
		with open ('log/log_grid_'+self.logName+'_population.csv', 'a') as log:
			log.write("Kp\tKd\tFitness\tFitness**2\t\n")

	def forward(self):
		# Mutation and Cross Over
		KpAxis = []
		KdAxis = []
		fitAxis = []
		t = tqdm(np.arange(0, 5, 0.25))

		for kp in t:
			for kd in np.arange(0, 5, 0.25):
				fitness = self.__get_fitness(np.array([kp, kd]))
				KpAxis.append(kp)
				KdAxis.append(kd)
				fitAxis.append(fitness)
				with open ('log/log_grid_'+self.logName+'_population.csv', 'a') as log:
					log.write("{:0.4f}\t{:0.4f}\t{:0.4f}\t{:0.4f}\t\n".format(kp, kd, fitness[0], fitness[1]))
				t.set_description("PID: {:0.4f} {:0.4f}, Fitness: {:0.4f}, Fitness**2: {:0.4f}".format(kp, kd, fitness[0], fitness[1]))

		fig = plt.figure()
		ax = fig.gca(projection='3d')
		ax.plot_trisurf(KpAxis, KdAxis, fitAxis, cmap="jet")
		plt.show()


						
			



#######################################################################################

if __name__ == '__main__':
	dea = DEA(NP=10, MaxGen=200)
	dea.forward()

	#print (np.hstack((dea.population, dea.fitness)))

conn.close()
print ('Server closed.')

