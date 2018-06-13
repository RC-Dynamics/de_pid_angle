import math

class Circle:
    
    def __init__(self, r):
        self.radius = r

    def circleDiscretization(self, qtd_poits = 40):
        ret = []
        angle_diff = 2 * math.pi / qtd_poits
        for i in range(qtd_poits):
            point = (self.radius * math.cos(i * angle_diff), self.radius * math.sin(i * angle_diff))
            ret.append(point)
        return ret
    
    def area(self):
        return math.pi * self.radius * self.radius