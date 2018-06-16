class Robot(object):
	"""docstring for Robot"""
	def __init__(self):
		super(Robot, self).__init__()
		self.x = 0
		self.y = 0
		self.angle = 0

	def setCoord(self,x,y,angle):
		self.x = x
		self.y = y
		self.angle = angle
		
	def getCoord(self):
		return self.x,self.y,self.angle

	def setVel(self, vLeft, vRight):
		self.vLeft = vLeft
		self.vRight = vRight

	def getVel(self):
		return '{"robot0":{"left":'+str(self.vLeft)+',"right":'+str(self.vRight)+'},"robot1":{"left":0,"right":0},"robot2":{"left":0,"right":0} }'