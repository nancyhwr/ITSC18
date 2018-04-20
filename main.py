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
#stream_range = [67, 100] # light: 12-30; busy: 30-67; jam: >67
stream_range = [30, 50] # light: 10-20; busy: 20-30; jam: >30 [30-100 [30-50]
graph = Graph(nodes, edges, stream_rates)
threshold = 100 #should be a constant value  2*stream_range[1]
DIS_THRESHOLD = 1
total_duration = 120  #240
minimum_waiting_time = 15  #60  120/8 = 15
speed_same = 0.05
L = 8
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


solution = {(-1, -1):[-1000, -1000], (0, -1):[-1000, -1000], (1, -1):[-1000, -1000],
				(-1, 0):[-1000, -1000], (0, 0):[-1000, -1000], (1, 0):[-1000, -1000],
				(-1, 1):[-1000, -1000], (0, 1):[-1000, -1000], (1, 1):[-1000, -1000]}

baseline = [[(['a','b'],13),(['c','d'],7),(['e','f'],6),(['g','h'],11)],
[(['a','d'],10),(['h','e'],10),(['c','f'],6),(['g','b'],11)],
[(['a','e'],7),(['h','d'],13),(['e','f'],6),(['g','b'],11)],
[(['a','e'],7),(['h','d'],13),(['c','g'],5),(['f','b'],12)],
[(['h','e'],10),(['a','d'],10),(['b','f'],12),(['g','c'],5)]]


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
			dis = distance(cars[i].predict_location(lane_list[cars[i].road_name]),cars[j].predict_location(lane_list[cars[j].road_name]))
			if dis <= cross_crash_threshold:
				pre_accident = pre_accident + 1

				if cars[i].road_name == cars[j].road_name:
					crash_solution = strategic1(cars[i], cars[j])
					if crash_solution == []:
						accident = accident+1
						crash_cars.append(cars[i])
						crash_cars.append(cars[j])
					else:
						
						cars[i].speed = cars[i].speed + cars[i].deta * crash_solution[0][0]
						cars[j].speed = cars[j].speed + cars[j].deta * crash_solution[0][1]

				else:
					crash_solution = strategic2(cars[i], cars[j])

					if crash_solution == None:
						accident = accident+1
						crash_cars.append(cars[i])
						crash_cars.append(cars[j])
					else:
						cars[i].speed = cars[i].speed + cars[i].deta * crash_solution[0][0]
						cars[j].speed = cars[j].speed + cars[j].deta * crash_solution[0][1]

	return crash_cars, accident, pre_accident


def acc_payoff(cari):

	t = distance(cari.location, cari.neighbors[0].location)/(cari.speed+cari.deta - cari.neighbors[0].speed)
	#return -10*1/t
	return -10 * math.exp(-(distance(cari.location, cari.neighbors[0].location)/(cari.speed+cari.deta - cari.neighbors[0].speed)))
	

## strategic game in scenario-1: same lane
def strategic1(cari, carj):
	result = []
	if carj.speed - carj.deta > 0:
		f_j = {-1: L/carj.speed - L/(carj.speed - carj.deta), 0: -1000, 1: -1000}
	else:
		f_j = {-1: -1000, 0: -1000, 1: -1000}
	if cari.neighbors[0] == None:
		f_i = {-1: -1000, 0: 0, 1: 1000}
	else:
		if round(cari.speed+cari.deta - cari.neighbors[0].speed, 2) == 0:
			f_i = {-1: -1000, 0: 0, 1: 1000}
		else:
			f_i = {-1: -1000, 0:0, 1: acc_payoff(cari)}
	solution[(-1, 0)] = [f_j[-1], f_i[0]]
	solution[(-1, 1)] = [f_j[-1], f_i[1]]
	solution[(0, 1)] = [f_j[0], f_i[1]]

	# pick a solution mainly from three options: (-1, 0), (-1, 1), (0, 1)

	if solution[(-1, 0)][1] > solution[(-1, 1)][1]:

		result.append((-1, 0))

	if solution[(0, 1)][0] > solution[(-1, 1)][0]:

		result.append((0, 1))

	if len(result) > 1:

		a = np.array([result[i][0]+ result[i][0] for i in range(len(result))])
		return [result[np.argmax(a)]]

	return result


## strategic game in scenario-2: crossing lane

def strategic2(cari, carj):
	result_i = []
	result_j = []
	result = []
	if cari.neighbors[0] == None and cari.speed - cari.deta > 0:
		f_i = {-1: L/cari.speed - L/(cari.speed - cari.deta) , 0: 0 , 1: 1000}
	elif cari.neighbors[0] == None:
		f_i = {-1: -1000 , 0: 0 , 1:1000}
	else:
		if round(cari.speed+cari.deta - cari.neighbors[0].speed, 2) == 0:
			f_i = {-1:-1000, 0: 0 , 1: 1000}
		else:
			#f_i = {-1:-1000, 0: 0 , 1: - 10 * math.exp(-round((distance(cari.location, cari.neighbors[0].location)/(cari.speed+cari.deta - cari.neighbors[0].speed)),2))}
			f_i = {-1: -1000, 0:0, 1: acc_payoff(cari)}
	if carj.neighbors[0] == None and carj.speed - carj.deta > 0:
		f_j = {-1: L/carj.speed - L/(carj.speed - carj.deta) , 0: 0 , 1:1000}
	elif carj.neighbors[0] == None:
		f_j = {-1: -1000 , 0: 0 , 1:1000}
	else :
		if round(carj.speed+carj.deta - carj.neighbors[0].speed, 2) == 0:
			f_j = {-1:-1000, 0: 0 , 1: 1000}
		else:
			f_j = {-1: -1000, 0:0, 1: acc_payoff(carj)}
	# have the same speed
	if abs(cari.speed - carj.speed) < speed_same:

		solution[(-1, 0)] = [f_i[-1], f_j[0]]
		solution[(-1, 1)] = [f_i[-1], f_j[1]]
		solution[(0, -1)] = [f_i[0], f_j[-1]]
		solution[(0, 1)] = [f_i[0], f_j[1]]
		solution[(1, -1)] = [f_i[1], f_j[-1]]
		solution[(1, 0)] = [f_i[1], f_j[0]]

	# speeds are different
	else:

		for key in solution.keys():
			solution[key] = [f_i[key[0]], f_j[key[1]]]
		solution[(0, 0)] = [-1000, -1000]

	## find the solution:

	a = np.array([solution[(k, -1)][0] for k in [-1, 0, 1]])
	b = np.array([solution[(k, 0)][0] for k in [-1, 0, 1]])
	c = np.array([solution[(k, 1)][0] for k in [-1, 0, 1]])

	result_i.append(((np.argmax(a))-1, -1))
	result_i.append(((np.argmax(b))-1, 0))
	result_i.append(((np.argmax(c))-1, 1))

	e = np.array([solution[(-1,k)][1] for k in [-1, 0, 1]])
	f = np.array([solution[(0,k)][1] for k in [-1, 0, 1]])
	g = np.array([solution[(1,k)][1] for k in [-1, 0, 1]])
	result_j.append((-1, (np.argmax(e))-1))
	result_j.append((0, np.argmax(f)-1))
	result_j.append((1, np.argmax(g)-1))

	result = [val for val in result_j if val in result_i]
	
	#print('[intersection]=',result )
	
	if len(result) > 1:

		res = np.array([result[i][0]+ result[i][0] for i in range(len(result))])
		#print('[AFTER]',[result[np.argmax(res)]])
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
	s = speed * 2000
	return (stream*s)/3600






###############################################################################################################################################

iteration = 10
TEST = 20

stat_pass_cars = []
stat_accident = []
stat_pred_accident = []

for t in range(TEST):

	avg_pass_cars = 0
	avg_accident = 0
	avg_pre_accident = 0


	for i in range(iteration):

	#print('[iteration] = ', iteration)
		dynamic_change(stream_rates,stream_range)
		i_nodes = copy.deepcopy(nodes)
		i_edges = copy.deepcopy(edges)
		i_stream_rate = copy.deepcopy(stream_rates)

		graph = Graph(i_nodes, i_edges, i_stream_rate)
	### Test the ID-Dijkstra ALgorithm
		coalitions = dk.coalition(graph, i_stream_rate,threshold)
	### Test the Baseline 
		#coalitions = baseline[0]
		group_lanes = [e[0] for e in coalitions]
		group_streams = [e[1] for e in coalitions]
		# print('[group_lanes] = ', group_lanes)
		# print('[streams] = ', group_streams)
		# print('[interval_times] = ', interval_time(group_streams, total_duration,minimum_waiting_time))
		interval_times = interval_time(group_streams, total_duration,minimum_waiting_time)
		#interval_times = list(interval_time(group_streams, total_duration,minimum_waiting_time)['x'])
		total_number = 0
		pass_cars = []
		accident = 0
		pre_acc = 0
		streams = {lane: stream_rates[lane] for lane in nodes}
		prob_car = {lane: generate_car(streams[lane], 0.025) for lane in nodes}


		for i in range(len(coalitions)):
		
		#print('[coalitoin] = ', i)
			group = coalitions[i][0]
			all_cars = []
		
			for j in range(int(interval_times[i])):
		
			#current_carsh(all_cars)
				for lane in group:
				## remove all the cars who already went out from the intersection. 
				## update all cars' positions
					remov_car = lane_list[lane].update_cars()
					if len(remov_car)>0:
						for i in remov_car:
							if i in all_cars:
								all_cars.remove(i)
							pass_cars.append(i)
							del i
					draw = random.uniform(0, 1)
					if draw < prob_car[lane]:
						new_car = Car(lane)
						lane_list[lane].car_list.append(new_car)
						all_cars.append(new_car)
						total_number =+ 1

			crash_result = tellCollapse(all_cars)
			remove_accidents(all_cars, crash_result[0])
			accident = accident+ crash_result[1]
			pre_acc = pre_acc + crash_result[2]

		
		avg_accident = avg_accident + accident 
		avg_pass_cars = avg_pass_cars + len(pass_cars)
		avg_pre_accident = avg_pre_accident + pre_acc

	stat_pass_cars.append(avg_pass_cars/iteration)
	stat_accident.append(avg_accident/iteration)
	stat_pred_accident.append(avg_pre_accident/iteration)


#print('[baseline5]')

print('[avg_throughput]= ', stat_pass_cars)
print('[avg_accident]= ', stat_accident)
print('[avg_pred_accident]= ', stat_pred_accident)



			






# def knuth_shuffle(items):
#     """
#     Fisher-Yates shuffle or Knuth shuffle which name is more famous.
#     Post constrain: return array of the same length of input but with random order
#     """
#     #print(items)
#     for i in range(len(items)):
#         j = random.randrange(i, len(items))
#         items[i], items[j] = items[j], items[i]
#     return items



	
