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

	print("Trying: Kp: {:0.2f} Ki: {:0.2f} Kd: {:0.2f}".format(kp, ki, kd))
	fitness_error = 0.0
	pid.setK(kp, ki, kd)
	robotPositions = []
	startPoints = (125, 65)

	x, y, theta = restartRobot()
	
	while (path.pointDistance(125, 65, x, y) >= 0.3):
		x,y,theta = restartRobot()
	
	x, y, theta = restartRobot()

	restartFlag = False
	countInt = 0 
	print ("chegggou")
	# -> 
	
	for i in path.getPath():
		
		start_time = time.time()
		# print(path.pointDistance(i[0], i[1], x, y))
		
		
		
		endPoints = (i[0], i[1])
		print (endPoints)
		
		# pass start and end point of segment -> Carlos
		while path.pointDistance(i[0], i[1], x, y) > 5 and time.time() - start_time < 0.4:
			#print("robo: {0:.2f}  {1:.2f} -- obj:{2:.2f}  {3:.2f} - erro: {4:.2f} - dist: {5:.2f}".format(x, y, i[0], i[1], erro, path.pointDistance(i[0], i[1], x, y)))
			x,y,theta = updateRobot(i)

			if(path.pointDistance(125, 65, x, y) <= 0.1 and countInt > 0):
				print("Restart!")
				restartFlag = True
				break
			robotPositions.append((x, y))
		
		if restartFlag == True:
			break

		fitness_error += path.getPerimeterError(startPoints, endPoints, robotPositions)
		startPoints = (i[0], i[1])
		countInt += 1
	print("Path complete!, with Fitness = ", fitness_error)
	
	return fitness_error
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
	def __init__(self, NP=15, D=3, MaxGen=500, CR=0.90, F=0.7):
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
		self.kpMax, self.kiMax, self.kdMax = 2, 2, 10
		self.__init__population()
		self.__init__fitness()

	def __init__population(self):
		kp = np.random.rand(self.NP, 1) * self.kpMax
		ki = np.random.rand(self.NP, 1) * self.kiMax
		kd = np.random.rand(self.NP, 1) * self.kdMax
		population = (np.hstack((kp, ki, kd)))
		self.population = np.zeros((self.NP, self.D), dtype=np.float)
		self.population[0] = (0.1, 0, 2) # Force a normal solution
		
		for i in range(1, self.NP ):
			for j in range(self.D):
				minD = np.min(population[:, j])
				maxD = np.max(population[:, j])
				self.population[i][j] = minD + np.random.rand(1) * (maxD - minD)


	def __init__fitness(self):
		self.fitness = np.zeros((self.NP, 1), dtype=np.float)
		for i in range(self.NP ):
			self.fitness[i] = fitnessFunction(self.population[i][0], self.population[i][1], self.population[i][2])

	def __get_fitness(self, genotype):
		return fitnessFunction(genotype[0],genotype[1],genotype[2]);

	def forward(self):
		# Mutation and Cross Over
		t = tqdm(range(self.MaxGen))
		for G in t:
			self.popG = np.zeros((self.NP, self.D), dtype=np.float)
			for i in range(self.NP):
				r1, r2, r3 = random.sample([x for x in range(self.NP) if x != i], 3)
				jrand = np.random.randint(self.D)

				for j in range(self.D):
					if(np.random.random(1) < self.CR) or (j == jrand):
						geneR1 = self.population[r1, j]
						geneR2 = self.population[r2, j]
						geneR3 = self.population[r3, j]
						self.popG[i,j] = geneR1 + self.F * (geneR2 - geneR3)

					else:
						self.popG[i,j] = self.population[i, j]
				# Selection
				popGFit = self.__get_fitness( self.popG[i])

				if popGFit < self.__get_fitness( self.population[i]):
					self.population[i] = self.popG[i]	
					self.fitness[i] = popGFit
				
			t.set_description("{}".format(np.min(self.fitness)))



#######################################################################################

if __name__ == '__main__':
	dea = DEA(NP=4, MaxGen=2)
	dea.forward()
	print (np.hstack((dea.population, dea.fitness)))

conn.close()
print ('Server closed.')

