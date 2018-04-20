# includes all cars' setting
import road
from road import Lane
import random


locations = {'a':(5, 0), 'b':(7,0),'c':(8, 5),'d':(8, 7),'e':(3, 8),'f':(1, 8),'g':(0, 3),'h':(0, 1)}

class Car:

	def __init__(self, road_name):

		self.speed = round(random.uniform(0.1, 0.5), 2)#0.005, 0.025
		self.location = locations[road_name]
		self.neighbors = [None, None]
		self.road_name = road_name
		self.road = Lane(road_name)
		self.deta = 0.1
		

	def run(self, delta_t):
		self.location = self.road.next_location(self.road_name, self.location, self.speed*delta_t)

	def predict_location(self, lane):

		return lane.next_location(self.location, self.deta)







