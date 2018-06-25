import socket
import json
import time
from robot import Robot
from path import Path
from pid import PIDAngle
from datetime import datetime


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
	pid.setK(kp, ki, kd)
	robotPositions = []
	startPoints = (125, 65)

	x, y, theta = restartRobot()
	while (path.pointDistance(125, 65, x, y) != 0.0):
		x,y,theta = restartRobot()
	
	x, y, theta = restartRobot()

	restartFlag = False
	countInt = 0 
	pid.setK(kp, ki, kd)
	#print ("chegggou")
	warningRestart = True

	for i in path.getPath():
		robotPositions = []
		start_time = time.time()
		endPoints = (i[0], i[1])
		#print(countInt)
		while  path.pointDistance(i[0], i[1], x, y) > 1 and time.time() - start_time < 0.06:  
			x,y,theta = updateRobot(i)
			if(path.pointDistance(125, 65, x, y) <= 0.05 and countInt > 20 and warningRestart):
				#print("Warring: Possible Restart")
				warningRestart = False
				

			robotPositions.append((x, y))
		
		if restartFlag == True:
			break
		
		fitness_error += path.getPerimeterError(startPoints, endPoints, robotPositions)
		startPoints = (i[0], i[1])
		countInt += 1
	#print("Path complete!, with Fitness = ", fitness_error)
	if countInt != len(path.getPath()):
		print("ERROR: Count Interation Error {}".format(countInt))
	if(fitness_error < 250):
		print("Warring: Fitness error too low! {}".format(fitness_error))
		print("Warring: Set Fitness to Inf")
		fitness_error = 99999999
	
	restartRobot()
	time.sleep(0.2)

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
		self.kpMax, self.kiMax, self.kdMax = 4, 2, 4
		self.timeStamp = (datetime.now())  
		self.logName = '_'.join(str(x) for x in (self.timeStamp.year,self.timeStamp.month, self.timeStamp.day, self.timeStamp.hour, self.timeStamp.minute))
		
		self.__init_csv()
		self.__init__population()
		self.__init__fitness()


	def __init__population(self):
		kp = np.random.rand(self.NP, 1) * self.kpMax
		ki = np.random.rand(self.NP, 1) * self.kiMax
		kd = np.random.rand(self.NP, 1) * self.kdMax
		population = (np.hstack((kp, ki, kd)))
		self.population = np.zeros((self.NP, self.D), dtype=np.float)
		
		for i in range(0, self.NP ):
			for j in range(self.D):
				minD = np.min(population[:, j])
				maxD = np.max(population[:, j])
				self.population[i][j] = minD + np.random.rand(1) * (maxD - minD)
		
		
	def __init__fitness(self):
		self.fitness = np.zeros((self.NP, 1), dtype=np.float)
		for i in range(self.NP ):
			self.fitness[i] = fitnessFunction(self.population[i][0], self.population[i][1], self.population[i][2])
			print("PID: {}, Fitness: {}".format(self.population[i], self.fitness[i]) )

	def __get_fitness(self, genotype):
		return fitnessFunction(genotype[0],genotype[1],genotype[2]);

	def __init_csv(self):
		with open ('log/log_'+self.logName+'_epoch.csv', 'a') as log:
			log.write("Epoch\t Min\t Mean\t Best\t\n")

		with open ('log/log_'+self.logName+'_population.csv', 'a') as log:
			log.write("Kp\tKi\tKd\tFitness\t\n")

	def forward(self):
		# Mutation and Cross Over
		t = tqdm(range(self.MaxGen))
		epoch = 0
		for G in t:

			with open ('log/log_'+self.logName+'_population.csv', 'a') as log:
				log.write("Epoch: {}\n".format(epoch))
				#logWrite = np.array2string(np.hstack((self.population, self.fitness)), formatter={'float_kind':lambda x: "%.4f" % x})
				PIDList = np.hstack((self.population, self.fitness))
				for PID in PIDList:
					log.write("{:0.4f}\t{:0.4f}\t{:0.4f}\t{:0.4f}\t\n".format(PID[0], PID[1], PID[2], PID[3]))
				

			with open ('log/log_'+self.logName+'_epoch.csv', 'a') as log:
				indexMin = np.argmin(self.fitness)
				minFit = np.min(self.fitness)
				meanFit = np.mean(self.fitness)
				logWrite = np.array2string( self.population[indexMin] , formatter={'float_kind':lambda x: "%.4f" % x})
				log.write("{}\t{:.4f}\t{:.4f}\t{}\t\n".format(epoch, minFit, meanFit, logWrite))
			
			epoch += 1
		
			self.popG = np.zeros((self.NP, self.D), dtype=np.float)
			for i in range(self.NP):
				r1, r2, r3 = random.sample([x for x in range(self.NP) if x != i], 3)
				jrand = np.random.randint(self.D)

				for j in range(self.D):
					if(np.random.random(1) < self.CR) or (j == jrand):
						geneR1 = self.population[r1, j]
						geneR2 = self.population[r2, j]
						geneR3 = self.population[r3, j]
						gene = geneR1 + self.F * (geneR2 - geneR3)
						self.popG[i,j] = abs(gene)

					else:
						self.popG[i,j] = self.population[i, j]
				# Selection
				popGFit = self.__get_fitness( self.popG[i])
				t.set_description("PID: {}, Fitness: {}".format(self.popG[i],popGFit))

				if popGFit <= self.fitness[i]:
					self.population[i] = self.popG[i]	
					self.fitness[i] = popGFit


						
			



#######################################################################################

if __name__ == '__main__':
	try:
		dea = DEA(NP=10, MaxGen=200)
		dea.forward()

		print (np.hstack((dea.population, dea.fitness)))
	except:
		conn.close()
		print ('Server closed.')
	conn.close()
	print ('Server closed.')

