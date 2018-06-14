import numpy as np 
import random

from fitness_ackley import fitness_ackley as fitnessFunction
from tqdm import tqdm
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
		self.kpMax, self.kiMax, self.kdMax = 10, 2, 10
		self.__init__population()
		self.__init__fitness()

	def __init__population(self):
		kp = np.random.rand(self.NP, 1) * self.kpMax
		ki = np.random.rand(self.NP, 1) * self.kiMax
		kd = np.random.rand(self.NP, 1) * self.kdMax
		population = (np.hstack((kp, ki, kd)))
		self.population = np.zeros((self.NP, self.D), dtype=np.float)
		for i in range(self.NP ):
			for j in range(self.D):
				minD = np.min(population[:, j])
				maxD = np.max(population[:, j])
				self.population[i][j] = minD + np.random.rand(1) * (maxD - minD)

	def __init__fitness(self):
		self.fitness = np.zeros((self.NP), dtype=np.float)
		for i in range(self.NP ):
			self.fitness[i] = fitnessFunction(self.population[i])

				

	def __get_fitness(self, genotype):
		return fitnessFunction(genotype);

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

if __name__ == '__main__':
	dea = DEA()
	dea.forward()
	print (dea.population[0])


