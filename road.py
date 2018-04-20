# This file includes all the lane basic setting
# 4-way 2 lanes on each way


import matplotlib.pyplot as plt
import numpy
import math
from scipy.optimize import linprog


class Lane:


	def __init__(self, name):

		self.name = name
		self.turn_roads = ['a','c','e','g']
		self.stra_roads = ['b','d','e','h']
		self.centers = {'a':(0, 0), 'c':(8, 0),'e':(8, 8),'g':(0, 8)}
		self.turn_r = 5
		self.car_list=[]



	# return the location for each car in the next time
	def next_location(self, location, delta_x):

		if self.name in self.turn_roads:

			return self.turn_road(self.centers[self.name], self.turn_r, location, delta_x, self.name)

		else:
			return self.straight_road(location, delta_x, self.name)

# define the trajectory of a given turn (left) road
# input: center(c), radium, current location (x, y), lane number: road, delta_x
# output: next location
	def turn_road (self, c, r, location, delta_x, name):

		x = location[0]
		y = location[1]
		#print('location = ', location)

		if name == 'a':
			x_next = location[0]- delta_x
			# print(r**2 - (c[0]-x_next)**2)
			# print(math.sqrt(r**2 - (c[0]-x_next)**2))
			y_next = round(math.sqrt(max(0,r**2 - (c[0]-x_next)**2))+ c[1], 2)
			return ((x_next, y_next))
		elif name == 'c':
			x_next = max(3, location[0]- delta_x)
			y_next = round(math.sqrt(max(0, r**2 - (c[0]-x_next)**2)) + c[1], 2)
			return ((x_next, y_next))
		elif name == 'e':
			x_next = location[0]+ delta_x
			y_next = round(math.sqrt(max(0, r**2 - (c[0]- x_next)**2)) + c[1],2)
			y_next = 2*c[1] - y_next
			return ((x_next, y_next))
		else:
			x_next = min(5, location[0]+ delta_x)
			y_next = round(math.sqrt(max(0, r**2 - (c[0]- x_next)**2)) + c[1],2)
			y_next = 2*c[1] - y_next
			return ((x_next, y_next))

# define the straight lane
# return the locaiton of next time
	def straight_road(self, location, delta, name):

		if name == 'b':
			return ((location[0], location[1] + delta))
		elif name == 'd':
			return ((location[0]-delta, location[1]))
		elif name == 'f':
			return ((location[0], location[1]-delta))
		else:
			return((location[0]+delta, location[1]))


	def update_cars(self):
		remove_cars = []
		for i in range(len(self.car_list)):
			car = self.car_list[i]
			car.location = self.next_location(car.location, car.speed)
			if car.location[0]>=8 or car.location[0]<=0 or car.location[1]>=8 or car.location[1]<=0:
				remove_cars.append(car)
		if len(remove_cars) > 0:

			for car in remove_cars:
				self.car_list.remove(car)
		for j in range(len(self.car_list)):
			car = self.car_list[j]
			if j-1 >=0:
				car.neighbors[0] = self.car_list[j-1]
			if j+1 < len(self.car_list):
				car.neighbors[1] = self.car_list[j+1]
		return remove_cars
























	