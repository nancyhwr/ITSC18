from multiprocessing import Process
from scipy.optimize import linprog
from car import Car
import random
from dijkstra import Graph
import dijkstra as dk
from road import Lane
from car import Car
import copy
import math
import numpy as np
 




nodes = ['a', 'b','c','d','e','f','g','h']
edges = {'a':['b','d','e'], 'b':['a','g','f'],'c':['g','f','d'], 'd': ['a','c','h'], 'e':['a','h','f'],'f':['b','c','e'],'g':['b','c','h'],'h':['d','e','g']}
stream_rates = {'a':5, 'b': 8,'c':2,'d':5,'e':2,'f':4,'g':3,'h':8}
stream_range = [15, 25] # light: 10-20; busy: 20-30; jam: >30 [30-100 [30-50]
graph = Graph(nodes, edges, stream_rates)
threshold = 70 #should be a constant value  2*stream_range[1] 100

total_duration = 12 #120  #240
minimum_waiting_time = 2 #15 
speed_same = 0.05
L = 5
conflicts ={'a':['c','f','g','h'], 'b':['c','e','d','h'],'c':['b','e','a','h'],'d':['b','g','e','f'],'e':['d','g','c','b'],
			'f':['d','a','g','h'],'g':['f','a','e','d'],'h':['f','c','a','b']}

lane_a = Lane('a')
lane_b = Lane('b')
lane_c = Lane('c')
lane_d = Lane('d')
lane_e = Lane('e')
lane_f = Lane('f')
lane_g = Lane('g')
lane_h = Lane('h')

lane_list = {'a':lane_a, 'b':lane_b,'c':lane_c,'d':lane_d,'e':lane_e,'f':lane_f,'g':lane_g,'h':lane_h}
cross_crash_threshold = 0.5  #1.3 # calcualte from two curved lane. 
straight_carsh_threshold = 0.5
DIS_THRESHOLD = 0.01

solution = {(-1, -1):[-1000, -1000], (0, -1):[-1000, -1000], (1, -1):[-1000, -1000],
				(-1, 0):[-1000, -1000], (0, 0):[-1000, -1000], (1, 0):[-1000, -1000],
				(-1, 1):[-1000, -1000], (0, 1):[-1000, -1000], (1, 1):[-1000, -1000]}

baseline_groups = [[['a','b'], ['c','d'],['e','f'],['g','h']],
[['a','d'],['h','e'],['c','f'],['g','b']],
[['a','e'],['h','d'],['e','f'],['g','b']],
[['a','e'],['h','d'],['c','g'],['f','b']],
[['h','e'],['a','d'],['b','f'],['g','c']]]


## Calculate the distances among pairs of cars on the same lane
def distance_cal(lanes, interval, speeds, stream_rates):
	distances = {}
	for lane in lanes:
		dis = (interval[lane] * speeds[lane] - 8) / (stream_rates[lane] -1)
		distances[lane]= round(dis)
	return distances


## Dynamicaly change the stream_rates 
def dynamic_change(stream_rates, stream_range):
	for key in stream_rates.keys():
		stream_rates[key] = random.randrange(stream_range[0], stream_range[1])

## the interval periods or each light 
def interval_time(group_streams, total_duration,minimum_waiting_time):

	c2 = [-e for e in group_streams]
	A2 = [[1]*len(group_streams)]
	b2 = [total_duration]
	lower = minimum_waiting_time
	#higher = total_duration/len(group_streams)
	higher  = total_duration
	bounds2 = ((lower, higher),)*len(group_streams)
	res2 = linprog(c2, A_eq = A2, b_eq = b2, bounds = bounds2) #A_eq = A2, b_eq = b2, 
	return list(res2['x'])


def distance(car1_loc, car2_loc):

	return math.sqrt((car1_loc[0] - car2_loc[0])**2 + (car1_loc[1] - car2_loc[1])**2)

## given a car, return a list of car who are gonna collapse. 
def tellCollapse(cars):
	accident = 0
	pre_accident = 0
	crash_cars = []

	for i in range(len(cars)):
		for j in range(i+1, len(cars)):
			#print(cars[i].location, cars[j].location)
			dis = distance(cars[i].predict_location(),cars[j].predict_location())
			if dis <= cross_crash_threshold and cars[i].name != cars[j].name:
				pre_accident = pre_accident + 1
				# the accident is predicted on the same lane. 
				if cars[i].road_name == cars[j].road_name:
					crash_solution = strategic1(cars[i], cars[j])
					cars[i].speed = cars[i].speed + cars[i].deta * crash_solution[0][1]
					cars[j].speed = cars[j].speed + cars[j].deta * crash_solution[0][0]
					change_speed_after(lane_list[cars[i].road_name].car_list, cars[i])
					change_speed_after(lane_list[cars[j].road_name].car_list, cars[j])

				# the accident is predicted on different lanes.
				else:
					crash_solution = strategic2(cars[i], cars[j])
					cars[i].speed = cars[i].speed + cars[i].deta * crash_solution[0][1]
					cars[j].speed = cars[j].speed + cars[j].deta * crash_solution[0][0]
					change_speed_after(lane_list[cars[i].road_name].car_list, cars[i])
					change_speed_after(lane_list[cars[j].road_name].car_list, cars[j])

	#return crash_cars, accident, pre_accident
	return pre_accident


def dec_payoff(cari):

	if cari.speed-cari.deta <= 0:
		return -1000
	else:
		return L/cari.speed - L/(cari.speed - cari.deta)


def acc_payoff(cari):

	if cari.neighbors[0] == None:
		return 1000
	else:
		if cari.speed+cari.deta  == cari.neighbors[0].speed:
			return 1000
		else:

			t = distance(cari.location, cari.neighbors[0].location)/(cari.speed+cari.deta - cari.neighbors[0].speed)
			if t == 0:
				return 1000
			else:
			#print('[location] =', cari.location,'[neighbor]=',cari.neighbors[0].location,'[t] = ', t)
				return -10/t
	
def change_speed_after(car_list, car):

	target_ibdex = car_list.index(car)
	for i in range(target_ibdex+1, len(car_list)):
		car_list[i].speed = car.speed




## strategic game in scenario-1: same lane
## return (v_j, v_i)
def strategic1(cari, carj):

	#print('strategic-1')
	result = []
	if carj.speed - carj.deta > 0:
		f_j = {-1: dec_payoff(carj), 0:0, 1: -1000}
	else:
		f_j = {-1: -1000, 0:0, 1: -1000}

	f_i = {-1: -1000, 0:0, 1: acc_payoff(cari)}

	solution[(-1, 0)] = [f_j[-1], f_i[0]]
	solution[(-1, 1)] = [f_j[-1], f_i[1]]
	solution[(0, 1)] = [f_j[0], f_i[1]]

	if solution[(-1, 0)][1] > solution[(-1, 1)][1]:
		result.append((-1, 0))
	if solution[(0, 1)][0] > solution[(-1, 1)][1]:
		result.append((0, 1))
	if solution[(-1, 1)][0] >= solution[(-1, 0)][0] and solution[(-1, 1)][1] >= solution[(0, 1)][1]:
		result.append((-1, 1))
	if len(result) > 1:

		a = np.array([result[i][0]+ result[i][0] for i in range(len(result))])
		return [result[np.argmax(a)]]

	return result  


## strategic game in scenario-2: crossing lane
## return (v_j, v_i)
def strategic2(cari, carj): 
	#print('strategic-2')
	result_i = []
	result_j = []

	f_j = {-1: dec_payoff(carj), 0:0, 1:acc_payoff(carj)}
	f_i = {-1: dec_payoff(cari), 0:0, 1:acc_payoff(cari)}

	if cari.speed == carj.speed:
		solution[(-1, -1)] = [-1000, -1000]
		solution[(1, 1)] = [-1000, -1000]

	else:
		solution[(-1, -1)] = [f_j[-1], f_i[-1]]
		solution[(1, 1)] = [f_j[1], f_i[1]]

	solution[(-1, 0)] = [f_j[-1], f_i[0]]
	solution[(-1, 1)] = [f_j[-1], f_i[1]]
	solution[(0, -1)] = [f_j[0], f_i[-1]]
	solution[(0, 1)] = [f_j[0], f_i[1]]
	solution[(1, -1)] = [f_j[1], f_i[-1]]
	solution[(1, 0)] = [f_j[1], f_i[0]]

	## find the solution:

	a = np.array([solution[(k, -1)][0] for k in [-1, 0, 1]])
	b = np.array([solution[(k, 0)][0] for k in [-1, 0, 1]])
	c = np.array([solution[(k, 1)][0] for k in [-1, 0, 1]])

	result_j.append(((np.argmax(a))-1, -1))
	result_j.append(((np.argmax(b))-1, 0))
	result_j.append(((np.argmax(c))-1, 1))

	e = np.array([solution[(-1,k)][1] for k in [-1, 0, 1]])
	f = np.array([solution[(0,k)][1] for k in [-1, 0, 1]])
	g = np.array([solution[(1,k)][1] for k in [-1, 0, 1]])
	result_i.append((-1, (np.argmax(e))-1))
	result_i.append((0, np.argmax(f)-1))
	result_i.append((1, np.argmax(g)-1))

	result = [val for val in result_j if val in result_i]

	if len(result) > 1:

		res = np.array([result[i][0]+ result[i][0] for i in range(len(result))])
		#print('[2-result]=', [result[np.argmax(res)]])
		return [result[np.argmax(res)]]
	return result

def remove_accidents(all_cars, accident_cars):

	for car in accident_cars:
		lane = car.road_name
		## Sometimes, three cars to crash happened. Two of them are on the same alne but the other is on the other lane. I deal with the same lane first
		## so when I solve the same lane, the car is possible already removed, so there won't exist anymore when I try to deal with the other lane
		## however, in the accident_cars list, that car will occur twice, so i forcely ignore this case.
		if car in lane_list[lane].car_list:	
			lane_list[lane].car_list.remove(car)
			all_cars.remove(car)
		

# calculate the probability of generating a car within a second. 
def generate_car(stream, speed):

	return 20/stream
	# s = speed * 2000
	# return (stream*s)/3600


def current_accident(all_cars):
	count = 0
	crash_cars = []
	for i in range(len(all_cars)):
		for j in range(i+1, len(all_cars)):
			if distance(all_cars[i].location, all_cars[j].location) < DIS_THRESHOLD and all_cars[i].name!= all_cars[j].name:
				count = count + 1
				if all_cars[i] not in crash_cars:
					crash_cars.append(all_cars[i])
				if all_cars[j] not in crash_cars:
					crash_cars.append(all_cars[j])		
	return count, crash_cars
	#return crash_cars


def remove_crash_cars(car_list):
	for car in car_list:
		lane = car.road_name
		lane_list[lane].car_list.remove(car)

def form_baseline(groups, stream):
	return [(groups[i], stream[str(groups[i][0])]+ stream[str(groups[i][1])]) for i in range(len(groups))]

###############################################################################################################################################

rounds = 20
experiment = 10
deno = rounds*experiment

stat_pass_cars = []
stat_accident = []
stat_pred_accident = []



for t in range(experiment):
	#print('[experiment] = ', t)
	
	dynamic_change(stream_rates,stream_range)
	i_nodes = copy.deepcopy(nodes)
	i_edges = copy.deepcopy(edges)
	i_stream_rate = copy.deepcopy(stream_rates)
	graph = Graph(i_nodes, i_edges, i_stream_rate)
	coalitions = dk.coalition(graph, i_stream_rate,threshold)	
	#coalitions = form_baseline(baseline_groups[4], i_stream_rate)
	
	group_lanes = [e[0] for e in coalitions]
	group_streams = [e[1] for e in coalitions]
	interval_times = interval_time(group_streams, total_duration,minimum_waiting_time)
	streams = {lane: stream_rates[lane] for lane in nodes}
	prob_car = {lane: stream_rates[lane]/20 for lane in nodes} #0.25

	total_number = 0
	pass_cars = 0
	accident = 0
	pre_acc = 0

	for i in range(rounds):
		
		# we calculate by every 10 seconds. 
		#dynamic_change(stream_rates,stream_range)
		for c in range(len(coalitions)):
			group = coalitions[c][0]

			for j in range(int(interval_times[c])):  #every 10 seconds

				all_cars = []
				for lane in group:
					all_cars = all_cars + lane_list[lane].car_list
				pre_acc = pre_acc + tellCollapse(all_cars)
				accident_analysis = current_accident(all_cars)
				accident = accident + accident_analysis[0]
				remove_crash_cars(accident_analysis[1])
				

				for s in range(20):  #each unite represent 0.5 second
					for lane in group:
						pass_cars = pass_cars + lane_list[lane].move_all()
						if random.uniform(0, 1) < prob_car[lane]:
							lane_list[lane].car_list.append(Car(lane, lane+str(i)))
	
					#print('accident = ', accident)
	stat_pass_cars.append(pass_cars/rounds)
	stat_accident.append(accident/rounds)
	stat_pred_accident.append(pre_acc/rounds)


print('psfa')
print('throughput_psfa_= ', stat_pass_cars)
print('acc_b_= ', stat_accident)
print('pre_accident_= ', stat_pred_accident)

