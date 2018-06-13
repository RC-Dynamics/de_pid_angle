import numpy as np 
import random
class DEA:
	def __init__(self, NP=5, D=3, MaxGen=20, CR=0.9, F=1):
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
		# TO DO
		pass

	def __get_fitness(self, genotype):
		# TO DO
		return 1;

	def forward(self):
		# Mutation and Cross Over
		for G in range(self.MaxGen):
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
			if self.__get_fitness( self.popG[i, :]) > self.__get_fitness( self.population[i, :]):
				self.population[i] = self.popG[i]	
				

if __name__ == '__main__':
	dea = DEA()
	#dea.forward()


