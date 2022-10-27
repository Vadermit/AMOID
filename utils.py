import geopandas as gpd
import pandas as pd
import numpy as np
import osmnx as ox
import networkx as nx
import copy
import time
import re
from heapq import heappush, heappop
from itertools import count
import pickle
import matplotlib.pyplot as plt
import logging

logger = logging
logger.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')

def list_intersection(lst1, lst2, signal_bundle_dict): 
    lst3 = [value for value in lst1 if value in lst2]
    remove_list = []
    for i in range(len(lst3)):
        try:
            if (lst3[i] in signal_bundle_dict) and (lst3[i + 1] in signal_bundle_dict[lst3[i]]):
                remove_list.append(lst3[i + 1])
        except:
            pass
    for rm in remove_list:
        lst3.remove(rm)
    return lst3

def used_stops(routes):
#     print(routes)
    used_stops = []
    for i in range(len(routes)):
        used_stops = used_stops + routes[i][2][1:]    #list(set(used_stops).union(set(routes[i][1])))
#     print(used_stops)
#     used_stops.pop(0)
    try:
        used_stops.pop()
    except:
        return []
#     print(used_stops)
#     print('--------------------------------')
    return used_stops

def stops_routes_finder(G_routing_temp, source, target, time_dict, length_dict, complex_mode = False, cutoff = None):
    if complex_mode:
        try:
            routes = list(nx.all_simple_paths(G_routing_temp, source = source, target = target, cutoff = cutoff))
            sym = 1
        except:
            return []
        # res = []
        # [res.append(x) for x in routes if x not in res]
        res = [] 
        for i in routes: 
            if i not in res:
                res.append(i)
    #     print(len(res))
        line_routes = []
        if len(res) != 0:
            for i in range(len(res)):
                route_time = 0
                route_length = 0
                for j in range(len(res[i]) - 1):
                    route_time += time_dict[res[i][j]][res[i][j + 1]]
                    route_length += length_dict[res[i][j]][res[i][j + 1]]
#                     route_time += nx.get_edge_attributes(G_routing_temp, 'travel_time')[(res[i][j], res[i][j + 1], 0)]
#                     route_length += nx.get_edge_attributes(G_routing_temp, 'length')[(res[i][j], res[i][j + 1], 0)]
                line_routes.append((route_time, route_length, res[i]))
            if len(res) >= 3:
                line_routes = sorted(line_routes, key=lambda x: x[0])[:3]
            elif len(res) == 2:
                line_routes = sorted(line_routes, key=lambda x: x[0])[:2]
            elif len(res) == 1:
                line_routes = [sorted(line_routes, key=lambda x: x[0])[0]]

    else:
        try:
            route_path = nx.algorithms.dijkstra_path(G_routing_temp, source = source, target = target, weight = 'travel_time')
#             print('okk')
            route_time = 0
            route_length = 0
            for j in range(len(route_path) - 1):
                route_time += time_dict[route_path[j]][route_path[j + 1]]
                route_length += length_dict[route_path[j]][route_path[j + 1]]
#                 route_length += nx.get_edge_attributes(G_routing_temp, 'length')[(route_path[j], route_path[j + 1], 0)]
#                 route_time += nx.get_edge_attributes(G_routing_temp, 'travel_time')[(route_path[j], route_path[j + 1], 0)]
            line_routes = [(route_time, route_length, route_path)]
        except:
#             print('sss')
            return []
    return line_routes

def line_route_finder(G_routing, line, time_dict, length_dict, no_repeat = True, complex_mode = False, cutoff = None):
#     G_eva_bus = copy.deepcopy(G_eva)
    routes = []
    sim_val = 1
    G_routing_temp = copy.deepcopy(G_routing)
    
    for j in range(len(line) - 1):
        source = line[j]
        target = line[j + 1]
#         print(source)
#         print(target)
#         path = stops_routes_finder(G_routing_temp, source, target, time_dict, length_dict, complex_mode = False)
#         print(path)
        try:
            path = stops_routes_finder(G_routing_temp, source, target, time_dict, length_dict, complex_mode = False)[0]
        except:
#             print(j)
            sim_val = 0
            break
        routes.append(path)
        if no_repeat:
            pop_stops_list = used_stops(routes)
#             print(pop_stops_list)
            G_routing_temp = G_routing.copy()
            G_routing_temp.remove_nodes_from(pop_stops_list)
    
    comp_val = 0
    if complex_mode and (sim_val == 0):
        routes = []
        comp_val = 1
        back_step = 3
        optional_routes = []
        line_d1 = copy.deepcopy(line)
        if len(line_d1) == 1:
            return [(0, line_d1)]
        max_frontier = 0
        stop = 0
        pop_stops_list = []
        while True:
    #             print(stop)
            if len(optional_routes) == stop:
                G_routing_temp = copy.deepcopy(G_routing)
                if no_repeat:
                    G_routing_temp.remove_nodes_from(pop_stops_list)
                source = line_d1[stop]
                target = line_d1[stop + 1]
                temp_routes = stops_routes_finder(G_routing_temp, source, target, time_dict, length_dict, complex_mode = True, cutoff = cutoff)
                optional_routes.append(temp_routes)

            if len(optional_routes[stop]) != 0:
    #                 print(optional_routes[stop][0])
                routes.append(optional_routes[stop][0])
                if no_repeat:
                    pop_stops_list = used_stops(routes)
                optional_routes[stop].pop(0)
                stop += 1
                if stop == len(line_d1) - 1:
                    break
                if stop > max_frontier:
                    max_frontier = stop
            else:
                try:
                    optional_routes.pop(stop)
                    stop -= 1
                    routes.pop(stop)
                    if max_frontier - stop > back_step:
                        comp_val = 0
                        break
                except:
                    comp_val = 0
                    break
    if (sim_val == 1) or (comp_val == 1):
        return routes
    else:
        return None

def geo_dist_cal(x, y, a, b):
    return np.sqrt(np.square(x - a) + np.square(y - b))

def opeartion_time_cal(G_eva, G_routing, route_list, fleet_size, traffic_signals_list, time_dict, length_dict, intersections_xy_dict, signal_bundle_dict, intersection_time_loss, stop_time_loss = 30, no_repeat = True, complex_mode = False, cutoff = None, return_dict = False):
    cir = np.zeros(len(route_list))
    approaching_time_from_departure = []
    bus_intersection_time_loss_dict = dict()
    line_lengths = []
    for i in range(len(route_list)):
        approaching_time_from_departure.append([])
        line_lengths.append([])
        if route_list[i][0] != route_list[i][-1]:  # Non-circular bus line, two-way operation
            cir[i] = 0
            routes_d1 = line_route_finder(G_routing, route_list[i], time_dict, length_dict, no_repeat = no_repeat, complex_mode = complex_mode, cutoff = cutoff)
            if routes_d1 is not None:
                routes_d2 = line_route_finder(G_routing, route_list[i][::-1], time_dict, length_dict, no_repeat = no_repeat, complex_mode = complex_mode, cutoff = cutoff)
                if routes_d2 is None:
                    return None
            else:
                return None
        else: # Circular bus line, one-way operation
            cir[i] = 1
            routes_d1 = line_route_finder(G_routing, route_list[i], time_dict, length_dict, no_repeat = no_repeat, complex_mode = complex_mode, cutoff = cutoff)
            if routes_d1 is None:
                routes_d2 = line_route_finder(G_routing, route_list[i][::-1], time_dict, length_dict, no_repeat = no_repeat, complex_mode = complex_mode, cutoff = cutoff)
                if routes_d2 is None:
                    return None
                else:
                    routes_ = routes_d2
            else:
                routes_ = routes_d1
        # Calculating bus operation time and headway
        
        if cir[i] == 0:
            approaching_time_from_departure[i].append([0])
            line_operation_time = 0
            line_length = 0
            for j in range(len(routes_d1)):
                line_operation_time = line_operation_time + routes_d1[j][0] + intersection_time_loss * len(list_intersection(routes_d1[j][2][:-1], traffic_signals_list, signal_bundle_dict))
                line_length += routes_d1[j][1]
#                 dict_key = str(routes_d1[j][2][0])+'_'+str(i + 1)+'_'+'0'+ '_to_' +str(routes_d1[j][2][-1])+'_'+str(i + 1)+'_'+'0'
                if (str(routes_d1[j][2][0])+'_'+str(i + 1)+'_'+'0') in bus_intersection_time_loss_dict:
                    bus_intersection_time_loss_dict[str(routes_d1[j][2][0])+'_'+str(i + 1)+'_'+'0'][str(routes_d1[j][2][-1])+'_'+str(i + 1)+'_'+'0'] = intersection_time_loss * len(list_intersection(routes_d1[j][2][:-1], traffic_signals_list, signal_bundle_dict))
                else:
                    bus_intersection_time_loss_dict[str(routes_d1[j][2][0])+'_'+str(i + 1)+'_'+'0'] = dict()
                    bus_intersection_time_loss_dict[str(routes_d1[j][2][0])+'_'+str(i + 1)+'_'+'0'][str(routes_d1[j][2][-1])+'_'+str(i + 1)+'_'+'0'] = intersection_time_loss * len(list_intersection(routes_d1[j][2][:-1], traffic_signals_list, signal_bundle_dict))
                if j > 1:
                    line_operation_time += stop_time_loss
                approaching_time_from_departure[i][0].append(line_operation_time)
            line_lengths[i].append(line_length)
            
            line_operation_time = 0
            line_length = 0
            approaching_time_from_departure[i].append([0])
            for j in range(len(routes_d2)):
                line_operation_time = line_operation_time + routes_d2[j][0] + intersection_time_loss * len(list_intersection(routes_d2[j][2][:-1], traffic_signals_list, signal_bundle_dict))
                line_length += routes_d2[j][1]
                if (str(routes_d2[j][2][0])+'_'+str(i + 1)+'_'+'1') in bus_intersection_time_loss_dict:
                    bus_intersection_time_loss_dict[str(routes_d2[j][2][0])+'_'+str(i + 1)+'_'+'1'][str(routes_d2[j][2][-1])+'_'+str(i + 1)+'_'+'1'] = intersection_time_loss * len(list_intersection(routes_d2[j][2][:-1], traffic_signals_list, signal_bundle_dict))
                else:
                    bus_intersection_time_loss_dict[str(routes_d2[j][2][0])+'_'+str(i + 1)+'_'+'1'] = dict()
                    bus_intersection_time_loss_dict[str(routes_d2[j][2][0])+'_'+str(i + 1)+'_'+'1'][str(routes_d2[j][2][-1])+'_'+str(i + 1)+'_'+'1'] = intersection_time_loss * len(list_intersection(routes_d2[j][2][:-1], traffic_signals_list, signal_bundle_dict))
                if j > 1:
                    line_operation_time += stop_time_loss
                approaching_time_from_departure[i][1].append(line_operation_time)
            line_lengths[i].append(line_length)
            
        else:
            line_operation_time = 0
            line_length = 0
            approaching_time_from_departure[i].append([0])
            for j in range(len(routes_)):
                line_operation_time = line_operation_time + routes_[j][0] + intersection_time_loss * len(list_intersection(routes_[j][2][:-1], traffic_signals_list, signal_bundle_dict))
                line_length += routes_[j][1]
                if (str(routes_[j][2][0])+'_'+str(i + 1)+'_'+'0') in bus_intersection_time_loss_dict:
                    bus_intersection_time_loss_dict[str(routes_[j][2][0])+'_'+str(i + 1)+'_'+'0'][str(routes_[j][2][-1])+'_'+str(i + 1)+'_'+'0'] = intersection_time_loss * len(list_intersection(routes_[j][2][:-1], traffic_signals_list, signal_bundle_dict))
                else:
                    bus_intersection_time_loss_dict[str(routes_[j][2][0])+'_'+str(i + 1)+'_'+'0'] = dict()
                    bus_intersection_time_loss_dict[str(routes_[j][2][0])+'_'+str(i + 1)+'_'+'0'][str(routes_d1[j][2][-1])+'_'+str(i + 1)+'_'+'0'] = intersection_time_loss * len(list_intersection(routes_[j][2][:-1], traffic_signals_list, signal_bundle_dict))
                if j > 1:
                    line_operation_time += stop_time_loss
                approaching_time_from_departure[i][0].append(line_operation_time)
            line_lengths[i].append(line_length)
            
    if return_dict == True:
        return approaching_time_from_departure, line_lengths, bus_intersection_time_loss_dict
    else:
        return approaching_time_from_departure

def graph_creation(G_eva, G_routing, route_list, fleet_size, traffic_signals_list, time_dict, length_dict, intersections_xy_dict, signal_bundle_dict, intersection_time_loss, stop_time_loss = 30, tolerance = 300, no_repeat = True, complex_mode = False, zero_wait = False, cutoff = None):
#     inter_stops_n = 0
    G_eva_bus = copy.deepcopy(G_eva)
    for u, v, key, data in G_eva_bus.out_edges(data=True, keys=True):
        nx.set_edge_attributes(G_eva_bus, {(u, v, key): {'bus': False}})
    wait_time = np.zeros(len(route_list))
    actual_tolerance = np.zeros(len(route_list))
    cir = np.zeros(len(route_list))
    bus_operation_length = np.zeros(len(route_list))
    stops_spacing = []
    circuity = list(np.zeros(len(route_list)))
    for i in range(len(route_list)):
#         inter_stops_n += 1
        if route_list[i][0] != route_list[i][-1]:  # Non-circular bus line, two-way operation
            cir[i] = 0
            routes_d1 = line_route_finder(G_routing, route_list[i], time_dict, length_dict, no_repeat = no_repeat, complex_mode = complex_mode, cutoff = cutoff)
            if routes_d1 is not None:
                routes_d2 = line_route_finder(G_routing, route_list[i][::-1], time_dict, length_dict, no_repeat = no_repeat, complex_mode = complex_mode, cutoff = cutoff)
                if routes_d2 is None:
                    return None
            else:
                return None
        else: # Circular bus line, one-way operation
            cir[i] = 1
            routes_d1 = line_route_finder(G_routing, route_list[i], time_dict, length_dict, no_repeat = no_repeat, complex_mode = complex_mode, cutoff = cutoff)
            if routes_d1 is None:
                routes_d2 = line_route_finder(G_routing, route_list[i][::-1], time_dict, length_dict, no_repeat = no_repeat, complex_mode = complex_mode, cutoff = cutoff)
                if routes_d2 is None:
                    return None
                else:
                    routes_ = routes_d2
            else:
                routes_ = routes_d1
        
        # Calculating bus operation time and headway
        if cir[i] == 0:
            line_operation_time = 0
            for j in range(len(routes_d1)):
                line_operation_time = line_operation_time + routes_d1[j][0] + intersection_time_loss * len(list_intersection(routes_d1[j][2][:-1], traffic_signals_list, signal_bundle_dict))
            for j in range(len(routes_d2)):
                line_operation_time = line_operation_time + routes_d2[j][0] + intersection_time_loss * len(list_intersection(routes_d2[j][2][:-1], traffic_signals_list, signal_bundle_dict))
            wait_time[i] = line_operation_time / (fleet_size[i] * 2)
        else:
            line_operation_time = 0
            for j in range(len(routes_)):
                line_operation_time = line_operation_time + routes_[j][0] + intersection_time_loss * len(list_intersection(routes_[j][2][:-1], traffic_signals_list, signal_bundle_dict))
            wait_time[i] = line_operation_time / (fleet_size[i] * 2)
        actual_tolerance[i] = min(wait_time[i], tolerance)
        wait_time[i] = wait_time[i] - actual_tolerance[i]
        
        # Calculating bus operation length and stop spacing
        if cir[i] == 0:
            for j in range(len(routes_d1)):
                stops_spacing.append(routes_d1[j][1])
                bus_operation_length[i] += routes_d1[j][1]
            source_node = route_list[i][0]
            target_node = route_list[i][-1]
            source_node_x = intersections_xy_dict[source_node]['x']
            source_node_y = intersections_xy_dict[source_node]['y']
            target_node_x = intersections_xy_dict[target_node]['x']
            target_node_y = intersections_xy_dict[target_node]['y']
            direct_dist = geo_dist_cal(source_node_x, source_node_y, target_node_x, target_node_y) / 3.280840
            circuity[i] = bus_operation_length[i] / direct_dist
        else:
            for j in range(len(routes_)):
                stops_spacing.append(routes_[j][1])
                bus_operation_length[i] += routes_[j][1]
            circuity[i] = None
            
        if zero_wait:
            wait_time[i] = 0
        
        # Creating bus layer
        if cir[i] == 0:
            G_eva_bus = add_bus_layer(i + 1, G_eva_bus, routes_d1, wait_time[i], cir[i], stop_time_loss = stop_time_loss, direction = 0)
            G_eva_bus = add_bus_layer(i + 1, G_eva_bus, routes_d2, wait_time[i], cir[i], stop_time_loss = stop_time_loss, direction = 1)
        else:
            G_eva_bus = add_bus_layer(i + 1, G_eva_bus, routes_, wait_time[i], cir[i])
        
    return G_eva_bus, wait_time, actual_tolerance, cir, stops_spacing, bus_operation_length, circuity

def add_bus_layer(line_N, G_eva, line, wait_time, cir, stop_time_loss = 30, direction = 0):
    for j in range(len(line)):
        source_node = line[j][2][0]
        target_node = line[j][2][-1]
        source_inode = str(source_node) + '_' + str(line_N) + '_' + str(direction)
        target_inode = str(target_node) + '_' + str(line_N) + '_' + str(direction)
        G_eva.add_nodes_from([source_inode, target_inode])
        if j == 0:
            _ = G_eva.add_edge(source_node, source_inode, **{'travel_time': wait_time, 'length': 0, 'bus': line_N})
            _ = G_eva.add_edge(source_inode, source_node, **{'travel_time': 0, 'length': 0, 'bus': line_N})

        if line[0][2][0] != line[-1][2][-1] or j != (len(line) - 1):
            _ = G_eva.add_edge(target_node, target_inode, **{'travel_time': wait_time, 'length': 0, 'bus': line_N})
            _ = G_eva.add_edge(target_inode, target_node, **{'travel_time': 0, 'length': 0, 'bus': line_N})
        _ = G_eva.add_edge(source_inode, target_inode, **{'travel_time': line[j][0] + stop_time_loss, 'length': line[j][1], 'bus': line_N})

    return G_eva

def bus_schedule_assign(bus_lines, line_lengths, fleet_size, approaching_time_from_departure, start_hour, end_hour, break_time = 15):
    stt = start_hour * 3600
    schedule = dict()
    daily_milage = 0
    for line_ in range(len(bus_lines)):
        approaching_times = approaching_time_from_departure[line_]
        bus_number = fleet_size[line_]
        bus_line = bus_lines[line_]
        ## Initialization: Approaching times of the first round
        # Direction 1
        for i in range(len(bus_line)):
            stop = str(bus_line[i]) + '_' + str(line_ + 1) + '_0'
            schedule[stop] = [stt + approaching_times[0][i]]
        mid_time = stt + approaching_times[0][i]
        if bus_line[0] != bus_line[-1]:
            # Direction 2
            cir = 0
            schedule[str(bus_line[i]) + '_' + str(line_ + 1) + '_1'] = [mid_time]
            for i in range(1, len(bus_line)):
                stop = str(bus_line[::-1][i]) + '_' + str(line_ + 1) + '_1'
                schedule[stop] = [mid_time + approaching_times[0][i]]
            head_way = int((approaching_times[0][-1] + approaching_times[1][-1] + break_time) / bus_number)
            daily_milage += line_lengths[line_][0] + line_lengths[line_][1]
        else:
            cir = 1
            head_way = int((approaching_times[0][-1] + break_time) / bus_number)
            daily_milage += line_lengths[line_][0]
            
        ## Schedule of the rest day
        end_time = end_hour * 3600
        while schedule[str(bus_line[0]) + '_' + str(line_ + 1) +'_0'][-1] < end_time:
            if cir == 0:
                daily_milage += line_lengths[line_][0] + line_lengths[line_][1]
            else:
                daily_milage += line_lengths[line_][0]
            for stop, sch in schedule.items():
                schedule[stop].append(sch[-1] + head_way)

    return schedule, head_way, daily_milage

def save_obj(obj, name):
    with open(name + '.pkl', 'wb') as f:
        pickle.dump(obj, f, pickle.HIGHEST_PROTOCOL)

def load_obj(name):
    with open(name + '.pkl', 'rb') as f:
        return pickle.load(f)

class Taxi():
    def __init__(self, initial_node = 97871013):
        self.free = True
        self.break_ = False
        self.end_ = False
        self.break_time = None
        self.total_free_time = 0
        self.origin_node = None
        self.destination_node = initial_node
        self.order_complete_time = 7 * 3600
        self.total_operation_time = 0
        self.current_operation_time = 0
        self.operation_distance_acc = 0

class Taxi_Fleet():
    def __init__(self, fleet_size, G_routing, intersection_time_loss, traffic_signals_list, signal_bundle_dict, time_dict = None, length_dict = None):
        self.fleet_size = fleet_size
        self.taxi_fleet = [Taxi() for i in range(fleet_size)]
        self.G_routing = G_routing
        self.intersection_time_loss = intersection_time_loss
        self.traffic_signals_list = traffic_signals_list
        self.signal_bundle_dict = signal_bundle_dict
        if time_dict is not None:
            self.time_dict = time_dict
            self.length_dict = length_dict
        else:
            self.time_dict, self.length_dict = create_dict(self.G_routing, cal_bus = False)
    
    def fleet_state_update(self, time_now):
        for i in range(self.fleet_size):
            if (self.taxi_fleet[i].free == False) and (self.taxi_fleet[i].order_complete_time < time_now):
                self.taxi_fleet[i].free = True
            if (self.taxi_fleet[i].break_ == True) and (time_now - self.taxi_fleet[i].break_time > 2700): # break at least 45 min
                self.taxi_fleet[i].break_ = False
#                 self.taxi_fleet[i].order_complete_time = None
    
    def assign_passenger_car(self, origin_node, target_node, order_time, wait_travel_time, pu_heading_time, taxi_travel_time, trip_dist, taxi_id):
        if self.taxi_fleet[taxi_id].free == True:
            current_free_time = order_time - self.taxi_fleet[taxi_id].order_complete_time
            self.taxi_fleet[taxi_id].total_free_time += current_free_time
            if current_free_time > 600: ## Time gap over 10 minutes is regard as a valid break
                self.taxi_fleet[taxi_id].current_operation_time = wait_travel_time # pu_heading_time + taxi_travel_time
                self.taxi_fleet[taxi_id].total_operation_time += wait_travel_time # pu_heading_time + taxi_travel_time
            else: # cruising
                self.taxi_fleet[taxi_id].current_operation_time += wait_travel_time + current_free_time
                self.taxi_fleet[taxi_id].total_operation_time += wait_travel_time + current_free_time
        else: # serve next ride immediately
            self.taxi_fleet[taxi_id].current_operation_time += pu_heading_time + taxi_travel_time
            self.taxi_fleet[taxi_id].total_operation_time += pu_heading_time + taxi_travel_time
        
        self.taxi_fleet[taxi_id].free = False
        self.taxi_fleet[taxi_id].origin_node = origin_node
        self.taxi_fleet[taxi_id].destination_node = target_node
        self.taxi_fleet[taxi_id].order_complete_time = order_time + wait_travel_time
        self.taxi_fleet[taxi_id].operation_distance_acc += trip_dist
        if self.taxi_fleet[taxi_id].current_operation_time > 16200: ## 4.5 hours
            self.taxi_fleet[taxi_id].break_ = 0
            self.taxi_fleet[taxi_id].break_time = self.taxi_fleet[taxi_id].order_complete_time
        if self.taxi_fleet[taxi_id].total_operation_time > 28800: ## 8 work hours
            self.taxi_fleet[taxi_id].end_ = True
    
    def wait_travel_time_cal(self, origin_node, target_node, order_time):
        if self.fleet_size == 0:
            return np.inf, np.inf, np.inf, np.inf, np.inf, None
        self.fleet_state_update(order_time)
#         wait_travel_time_list = np.zeros(self.fleet_size)
        pu_heading_time_list = np.zeros(self.fleet_size)
        pu_heading_dist_list = np.zeros(self.fleet_size)
        wait_time_list = np.zeros(self.fleet_size)
#         taxi_travel_time_list = np.zeros(self.fleet_size)
#         self.fleet_state_update(order_time)
        for i in range(self.fleet_size):
            try:
                pu_heading_time, pu_heading_dist, pu_heading_paths = multi_source_dijkstra(self.G_routing, [self.taxi_fleet[i].destination_node], targets = [origin_node], time_dict = self.time_dict, length_dict = self.length_dict)
                pu_heading_time, pu_heading_dist, pu_heading_paths = pu_heading_time[origin_node], pu_heading_dist[origin_node], pu_heading_paths[origin_node]
                pu_heading_time += self.intersection_time_loss * len(list_intersection(pu_heading_paths, self.traffic_signals_list, self.signal_bundle_dict))
            except:
                pu_heading_time = np.inf
                pu_heading_dist = np.inf
                
            pu_heading_time_list[i] = pu_heading_time
            pu_heading_dist_list[i] = pu_heading_dist
            wait_time_list[i] = pu_heading_time
            
            if self.taxi_fleet[i].free == False:
                wait_time_list[i] += self.taxi_fleet[i].order_complete_time - order_time
            
        sorted_indeces = np.argsort(wait_time_list)
        for i in range(len(sorted_indeces)):
            car_index = sorted_indeces[i]
            if (self.taxi_fleet[car_index].break_ == False) and (self.taxi_fleet[car_index].end_ == False):
                break
                
        pu_heading_dist = pu_heading_dist_list[car_index]
        pu_heading_time = pu_heading_time_list[car_index]
        wait_time = wait_time_list[car_index]
        wait_travel_time = wait_time_list[car_index]
    #         print(taxi_time)
        try:
            taxi_travel_time, taxi_travel_dist, taxi_travel_paths = multi_source_dijkstra(self.G_routing, [origin_node], targets = [target_node], time_dict = self.time_dict, length_dict = self.length_dict)
            taxi_travel_time, taxi_travel_dist, taxi_travel_paths = taxi_travel_time[target_node], taxi_travel_dist[target_node], taxi_travel_paths[target_node]
            taxi_travel_time += self.intersection_time_loss * len(list_intersection(taxi_travel_paths, self.traffic_signals_list, self.signal_bundle_dict))
            wait_travel_time += taxi_travel_time
            trip_dist = taxi_travel_dist + pu_heading_dist
            
#             taxi_time += self.intersection_time_loss * len(list_intersection(taxi_travel_paths, traffic_signals_list, signal_bundle_dict))
        except:
            wait_travel_time = np.inf
            taxi_travel_time = np.inf
            trip_dist = np.inf
        
        return wait_travel_time, wait_time, pu_heading_time, taxi_travel_time, trip_dist, car_index
    
    def daily_free_time(self):
        daily_ft = []
        for i in range(self.fleet_size):
            daily_ft.append(self.taxi_fleet[i].total_free_time)
        return daily_ft, np.mean(daily_ft)
    
    def daily_total_dist(self):
        total_dist = 0
        for i in range(self.fleet_size):
            total_dist += self.taxi_fleet[i].operation_distance_acc
        return total_dist
    
    
def time_gap_cal(schedule_list, current_time):
    if current_time > schedule_list[-1]:
        return np.inf
    for stt in schedule_list:
        if stt > current_time:
            break
    return stt - current_time

def bus_walk_separation(path):
    paths = []
    previous_type = 'walk'
    current_path = []
    for node in path:
        if isinstance(node, int):
            current_type = 'walk'
        else:
            current_type = 'bus'
        if current_type == previous_type:
            current_path.append(node)

        else:
            paths.append(copy.deepcopy(current_path))
            current_path = [node]
        previous_type = current_type

    paths.append(copy.deepcopy(current_path))
    return paths

# G_eva, G_routing, route_list, fleet_size, origins_node_list, destinations_node_list, origin_pos_dict, origin_ds_dict, full_dt_data, traffic_signals_list, od_in_TOP_OD, intersections_df, OD_cell_ind, primal_od_list, time_dict, length_dict, cover_list_dict, intersections_xy_dict, regular_user_number, signal_bundle_dict, intersection_time_loss, tolerance = 300, meters = 1320, no_repeat = True, complex_mode = False, cutoff = None, stop_time_loss = 30, departure_time = None, print_eva = False, return_eva = True

def evaluation_agent_based(taxi_fleet_size, full_dt_data, bus_lines, bus_fleet_size, G_eva, G_routing, traffic_signals_list, signal_bundle_dict, intersections_xy_dict, stop_time_loss, no_repeat, complex_mode, unit_bus_cost, unit_taxi_cost, intersection_time_loss, value_coefficients = [1., 1., 1., 1., 0.2, 0.2, 0.5, 0.5], global_eva = True, cutoff = None, start_hour = 7, end_hour = 22, OD_related_input_dict = None, print_ = True, log2file = False, return_dist_lists = False):
    ER_fossil = 256.274 # g CO2eq / km
    ER_electric = 76.67 # g CO2eq / km
    
    time_dict_routing, length_dict_routing = create_dict(G_routing)
    time_dict_eva, length_dict_eva = create_dict(G_eva)
    
    user_id = full_dt_data.rider_id.to_list()
    origins_node_list = full_dt_data.origin_nid.to_list()
    destinations_node_list = full_dt_data.destination_nid.to_list()
    drive_distance_list = full_dt_data.drive_distance.to_list()
    walk_time_list = full_dt_data.walk_time.to_list()
    trip_distance = full_dt_data.drive_distance.to_list()

    traveler_number_list = np.array(full_dt_data.passengers.to_list())
    date_list = full_dt_data.date.to_list()
    regular_user_index = np.array(full_dt_data.regular_user.to_list())
    regular_user_number = len(set(full_dt_data[full_dt_data['regular_user'] == True].rider_id.to_list()))
    total_regular_user_travel_number = np.sum(traveler_number_list[regular_user_index == True])
    day_count = 1
    start_time_sec_list = full_dt_data.start_time_sec.to_list()
    
    taxi_fleet = Taxi_Fleet(taxi_fleet_size, G_routing, intersection_time_loss, traffic_signals_list, signal_bundle_dict, time_dict_routing, length_dict_routing)
    taxi_cost = taxi_fleet_size * unit_taxi_cost
    
#     if OD_related_input_dict is not None:
    od_in_TOP_OD = OD_related_input_dict['od_in_TOP_OD']
    OD_cell_ind = OD_related_input_dict['OD_cell_ind']
    primal_od_list = OD_related_input_dict['primal_od_list']

    top_od_travel_by_bus = np.zeros(len(primal_od_list))
    top_od_volume_list = np.array([x[0] for x in primal_od_list]) # [volume, start_point, end_point]

        
    if bus_lines is not None:
        ## Opration Time Estimation
        approaching_time_from_departure, line_lengths, bus_intersection_time_loss_dict = opeartion_time_cal(G_eva, G_routing, bus_lines, bus_fleet_size, traffic_signals_list, time_dict_routing, length_dict_routing, intersections_xy_dict, signal_bundle_dict, intersection_time_loss, stop_time_loss = stop_time_loss, no_repeat = no_repeat, complex_mode = complex_mode, cutoff = cutoff, return_dict = True)
        try:
            approaching_time_from_departure = [np.round(TL) for TL in approaching_time_from_departure]
        except:
            print('bus_lines: ', bus_lines)
            print('approaching_time_from_departure: ', approaching_time_from_departure)
            print('bus_fleet_size: ', bus_fleet_size)
            ## Bus Schedual Creation
        schedule, head_way, daily_bus_milage = bus_schedule_assign(bus_lines, line_lengths, bus_fleet_size, approaching_time_from_departure, start_hour, end_hour, break_time = 15)
        
        ## Create Multi-layer Bus-Pedestrian Network
        G_eva_bus, wait_time, actual_tolerance, cir, stops_spacing, bus_operation_length, circuity = graph_creation(G_eva, G_routing, bus_lines, bus_fleet_size, traffic_signals_list, time_dict_routing, length_dict_routing, intersections_xy_dict, signal_bundle_dict, intersection_time_loss, stop_time_loss = stop_time_loss, no_repeat = no_repeat, complex_mode = complex_mode, cutoff = cutoff, zero_wait = True)
        
        time_dict, length_dict, bus_dict = create_dict(G_eva_bus, cal_bus = True)
        
        bus_cost = np.sum(bus_fleet_size) * unit_bus_cost

    else:
        bus_cost = 0

    total_cost = taxi_cost + bus_cost

    walker = 0
    bus_rider = 0
    taxi_rider = 0
    regular_user_served = 0
    bus_served_regular_user_id_list = []
    total_transit_traveler_trip_dist = 0
    total_bus_traveler_trip_dist = 0
#     total_taxi_traveler_trip_dist = 0
    bus_wait_time_list = []
    taxi_wait_time_list = []
    bus_time_saving_list = []
    taxi_time_saving_list = []
    taxi_travel_time_list = []
    transit_time_saving_ratio_list = []
    bus_time_saving_ratio_list = []

    bus_walk_to_pickup_distance_list = []
    bus_walk_to_des_distance_list = []
    bus_walk_transfer_distance_list = []
    bus_vehicle_distance_list = []
    bus_trip_distance_list = []

    taxi_trip_distance_list = []

    walk_trip_distance_list = []

    total_taxi_milage = 0
    previous_entry_date = date_list[0]
    for trip_ in range(len(origins_node_list)):
        source = origins_node_list[trip_]
        target = destinations_node_list[trip_]
        order_time = start_time_sec_list[trip_]
        date = date_list[trip_]
        if date != previous_entry_date:
            day_count += 1
            previous_entry_date = date
            total_taxi_milage += taxi_fleet.daily_total_dist()
            taxi_fleet = Taxi_Fleet(taxi_fleet_size, G_routing, intersection_time_loss, traffic_signals_list, signal_bundle_dict, time_dict_routing, length_dict_routing)
        ### Calculate walk time
        walk_time = walk_time_list[trip_]
#         walk_time, walk_dist, walk_paths = multi_source_dijkstra(G_eva, [source], targets = [target], time_dict = time_dict_eva, length_dict = length_dict_eva)
#         walk_time, walk_dist, walk_path = walk_time[target], walk_dist[target], walk_paths[target]
#         walk_time += intersection_time_loss * len(list_intersection(walk_path, traffic_signals_list, signal_bundle_dict))
  
        ### Calculate fixed-route bus transit time (if bus+walk time shorter than walking)
#         utils.multi_source_dijkstra(G_eva, [source], targets = [target], time_dict = time_dict_eva, length_dict = length_dict_eva)
        if bus_lines is not None:
            bus_time, bus_dist, bus_paths, bus_travel_modes, _ = multi_source_dijkstra(G_eva_bus, [source], targets = [target], time_dict = time_dict, length_dict = length_dict, bus_dict = bus_dict)
            bus_time, bus_dist, bus_path, bus_travel_mode = bus_time[target], bus_dist[target], bus_paths[target], bus_travel_modes[target]
    #         bus_time += intersection_time_loss * len(utils.list_intersection(bus_path, traffic_signals_list, signal_bundle_dict))
            current_time = order_time

            if bus_travel_mode == True:
                bus_time = 0
                bus_wait_time = 0
                bus_walk_to_pickup_distance = 0
                bus_walk_to_des_distance = 0
                bus_walk_transfer_distance = 0
                bus_vehicle_distance = 0
                path_chunks = bus_walk_separation(bus_path)

                chunks_num = len(path_chunks)
                for ch_i, chunk in enumerate(path_chunks):
                    for i in range(len(chunk) - 1):
                        bus_time += time_dict[chunk[i]][chunk[i + 1]]
                    if isinstance(chunk[0], int):
                        # walk intersection time loss
                        if ch_i == 0:
                            for i in range(len(chunk) - 1):
                                bus_walk_to_pickup_distance += length_dict[chunk[i]][chunk[i + 1]]
                        elif ch_i == chunks_num - 1:
                            for i in range(len(chunk) - 1):
                                bus_walk_to_des_distance += length_dict[chunk[i]][chunk[i + 1]]
                        else:
                            for i in range(len(chunk) - 1):
                                bus_walk_transfer_distance += length_dict[chunk[i]][chunk[i + 1]]

                        try:
                            bus_time += intersection_time_loss * len(list_intersection(chunk, traffic_signals_list, signal_bundle_dict))
                        except:
                            chunk = [node_ for node_ in chunk if isinstance(node_, int)]
                            bus_time += intersection_time_loss * len(list_intersection(chunk, traffic_signals_list, signal_bundle_dict))

                        current_time = order_time + bus_time
                    else:
                        for i in range(len(chunk) - 1):
                            bus_vehicle_distance += length_dict[chunk[i]][chunk[i + 1]]
                        bus_wait_time_current_line = time_gap_cal(schedule[chunk[0]], current_time)
                        bus_time += bus_wait_time_current_line
                        bus_wait_time += bus_wait_time_current_line
                        for i in range(len(chunk) - 1):
                            try:
                                bus_time += bus_intersection_time_loss_dict[chunk[i]][chunk[i + 1]] # Bus intersection time loss
                            except:
                                continue
                                # print('============== Error! =============')
                                # print('Chunk: ', chunk)
                                # print('i: {}, chunk[i]: {}, chunk[i+1]: {}'.format(i, chunk[i], chunk[i+1]))
                                # print('Path: ', bus_path)
                                # print('Chunks: ', path_chunks)
                                # print('Recreate chunks: ', bus_walk_separation(bus_path))
                                # print('bus_intersection_time_loss_dict: ')
                                # print(bus_intersection_time_loss_dict)
                                # print('-----------------------------------')
                                # bus_travel_mode = False
                        current_time = order_time + bus_time

            if bus_time > walk_time:
                bus_travel_mode = False
        else:
            bus_travel_mode = False
            bus_time = np.inf

        ### Calculate ride-hailing travel time
        taxi_time, taxi_wait_time, pu_heading_time, taxi_travel_time, taxi_trip_dist, taxi_id = taxi_fleet.wait_travel_time_cal(source, target, order_time)
#         print(taxi_time)

        if bus_travel_mode == True:
            if bus_time <= taxi_time:
                bus_rider += traveler_number_list[trip_]
                total_transit_traveler_trip_dist += traveler_number_list[trip_] * drive_distance_list[trip_]
                total_bus_traveler_trip_dist += traveler_number_list[trip_] * drive_distance_list[trip_]
                if regular_user_index[trip_]:
                    regular_user_served += traveler_number_list[trip_]
                    bus_served_regular_user_id_list.append(user_id[trip_])
                if od_in_TOP_OD[trip_] != 9999:
    #                 print(od_in_TOP_OD[i])
                    top_od_travel_by_bus[od_in_TOP_OD[trip_]] += 1
        
                for traveler_ in range(traveler_number_list[trip_]):
                    bus_wait_time_list.append(bus_wait_time)
                    ts = walk_time - bus_time
                    tsr = ts / walk_time
                    bus_time_saving_list.append(ts)
                    bus_time_saving_ratio_list.append(tsr)
                    transit_time_saving_ratio_list.append(tsr)

                    bus_walk_to_pickup_distance_list.append(bus_walk_to_pickup_distance)
                    bus_walk_to_des_distance_list.append(bus_walk_to_des_distance)
                    bus_walk_transfer_distance_list.append(bus_walk_transfer_distance)
                    bus_vehicle_distance_list.append(bus_vehicle_distance)
                    bus_trip_distance_list.append(trip_distance[trip_])

            else:
                total_transit_traveler_trip_dist += traveler_number_list[trip_] * drive_distance_list[trip_]
                if regular_user_index[trip_]:
                    regular_user_served += traveler_number_list[trip_]
                taxi_rider += traveler_number_list[trip_]
                taxi_fleet.assign_passenger_car(source, target, order_time, taxi_time, pu_heading_time, taxi_travel_time, taxi_trip_dist, taxi_id)
                for traveler_ in range(traveler_number_list[trip_]):
                    taxi_wait_time_list.append(taxi_wait_time)
                    ts = walk_time - taxi_time
                    taxi_time_saving_list.append(ts)
                    transit_time_saving_ratio_list.append(ts / walk_time)
                    taxi_travel_time_list.append(taxi_travel_time)
                    taxi_trip_distance_list.append(trip_distance[trip_])
#                 print((walk_time, bus_time, taxi_time))
        else:
            if walk_time < taxi_time:
                walker += traveler_number_list[trip_]
                walk_trip_distance_list += [trip_distance[trip_]] * traveler_number_list[trip_]
            else:
                total_transit_traveler_trip_dist += traveler_number_list[trip_] * drive_distance_list[trip_]
                if regular_user_index[trip_]:
                    regular_user_served += traveler_number_list[trip_]
                taxi_rider += traveler_number_list[trip_]
                taxi_fleet.assign_passenger_car(source, target, order_time, taxi_time, pu_heading_time, taxi_travel_time, taxi_trip_dist, taxi_id)
                for traveler_ in range(traveler_number_list[trip_]):
                    taxi_wait_time_list.append(taxi_wait_time)
                    taxi_time_saving_list.append(walk_time - taxi_time)
                    transit_time_saving_ratio_list.append((walk_time - taxi_time) / walk_time)
                    taxi_travel_time_list.append(taxi_travel_time)
                    taxi_trip_distance_list.append(trip_distance[trip_])
                    
    if bus_lines is not None:
        total_bus_milage = daily_bus_milage * day_count / 1000
    else:
        total_bus_milage = 0
    total_taxi_milage = total_taxi_milage / 1000
    total_transit_traveler_trip_dist = total_transit_traveler_trip_dist / 1000
    total_bus_traveler_trip_dist = total_bus_traveler_trip_dist / 1000
    
    total_bus_GHG_emission = total_bus_milage * ER_electric
    total_taxi_GHG_emission = total_taxi_milage * ER_electric
    
    transit_GHG_emission = total_bus_GHG_emission + total_taxi_GHG_emission
    private_car_GHG_emission_bus_only = total_bus_traveler_trip_dist * ER_fossil
    private_car_GHG_emission = total_transit_traveler_trip_dist * ER_fossil
    
    GHG_saving_ratio = (private_car_GHG_emission - transit_GHG_emission) / private_car_GHG_emission
    if bus_lines is not None:
        bus_GHG_saving_ratio = (private_car_GHG_emission_bus_only - total_bus_GHG_emission) / private_car_GHG_emission_bus_only
    else:
        bus_GHG_saving_ratio = 0
    
    if bus_lines is not None:
        total_bus_wait_time = np.sum(bus_wait_time_list)
        average_bus_wait_time = np.mean(bus_wait_time_list)
    else:
        total_bus_wait_time = 0
        average_bus_wait_time = 0

    total_taxi_wait_time = np.sum(taxi_wait_time_list)
    average_taxi_wait_time = np.mean(taxi_wait_time_list)


    total_transit_wait_time = np.sum(bus_wait_time_list + taxi_wait_time_list)
    average_transit_wait_time = np.mean(bus_wait_time_list + taxi_wait_time_list)
    
    ## Benefit-cost ratio
    if bus_lines is not None:
        total_bus_saved_time = np.sum(bus_time_saving_list)
        bus_benefit = total_bus_saved_time * 0.004961
        bus_benefit_cost_ratio = bus_benefit / bus_cost
    else:
        total_bus_saved_time = 0
        bus_benefit = 0
        bus_benefit_cost_ratio = 0
    
    taxi_benefit = np.sum(taxi_time_saving_list) * 0.004961
    taxi_benefit_cost_ratio = taxi_benefit / taxi_cost
    if print_:
        print()
    
    total_transit_saved_time = np.sum(bus_time_saving_list + taxi_time_saving_list)
    transit_benefit = total_transit_saved_time * 0.004961
    transit_benefit_cost_ratio = transit_benefit / total_cost
    
    ave_TSR = np.mean(transit_time_saving_ratio_list)
    if bus_lines is not None:
        bus_ave_TSR = np.mean(bus_time_saving_ratio_list)
    else:
        bus_ave_TSR = 0

    if bus_lines is not None:
        bus_regular_user_serving_ratio = len(set(bus_served_regular_user_id_list)) / regular_user_number
        top_od_travel_by_bus_ratio = top_od_travel_by_bus / top_od_volume_list
        bus_top_od_cover_ratio = len(top_od_travel_by_bus_ratio[top_od_travel_by_bus_ratio != 0]) / len(top_od_travel_by_bus_ratio)
    else:
        bus_regular_user_serving_ratio = 0
        top_od_travel_by_bus_ratio = 0
        bus_top_od_cover_ratio = 0
    
    if print_:
        print('Average transit time saving ratio: {}'.format(ave_TSR))
        
        print('Transit system GHG emission: {}'.format(transit_GHG_emission))
        print('Private car GHG emission: {}'.format(private_car_GHG_emission))
        print('Transit GHG emission saving ratio: {}'.format(GHG_saving_ratio))
        
        print('Average taxi_travel_time: {}'.format(np.mean(taxi_travel_time_list)))
        print()
    
        print('Waiting Time')
        print('total_bus_wait_time: {}'.format(total_bus_wait_time))
        print('average_bus_wait_time: {}'.format(average_bus_wait_time))
        print('total_taxi_wait_time: {}'.format(total_taxi_wait_time))
        print('average_taxi_wait_time: {}'.format(average_taxi_wait_time))
        print('total_transit_wait_time: {}'.format(total_transit_wait_time))
        print('average_transit_wait_time: {}'.format(average_transit_wait_time))
        print()
    
        print('Benefit-cost Ratio')
        print('bus_benefit: {}'.format(bus_benefit))
        print('bus_benefit_cost_ratio: {}'.format(bus_benefit_cost_ratio))
        print('taxi_benefit: {}'.format(taxi_benefit))
        print('taxi_benefit_cost_ratio: {}'.format(taxi_benefit_cost_ratio))
        print('transit_benefit: {}'.format(taxi_benefit + bus_benefit))
        print('transit_benefit_cost_ratio: {}'.format(transit_benefit_cost_ratio))
    
        if bus_rider != 0:
            print('Benefit per bus rider: {}'.format(bus_benefit / bus_rider))    
        if taxi_rider != 0:
            print('Benefit per taxi rider: {}'.format(taxi_benefit / taxi_rider))
    benefit_per_transit_rider = transit_benefit / (bus_rider + taxi_rider)
    if print_:
        print('Benefit per transit rider: {}'.format(benefit_per_transit_rider))

        print()
        if bus_rider != 0:
            print('Bus cost per ridership: {}'.format((bus_cost) / (bus_rider)))
        if taxi_rider != 0:
            print('Taxi cost per ridership: {}'.format((taxi_cost) / (taxi_rider)))
    transit_cost_per_rider = (bus_cost + taxi_cost) / (bus_rider + taxi_rider)
    if print_:
        print('Transit per ridership cost: {}'.format(transit_cost_per_rider))
        print('Transit per rider cost - benefit: {}'.format(transit_cost_per_rider - benefit_per_transit_rider))

        print()
    total_travelers = walker + bus_rider + taxi_rider
    transit_service_ratio = (bus_rider + taxi_rider) / total_travelers
    bus_service_ratio = bus_rider / total_travelers
    if print_:
        print('travel_by_walk_ratio: {}'.format(walker / total_travelers))
        print('travel_by_bus_ratio: {}'.format(bus_rider / total_travelers))
        print('travel_by_taxi_ratio: {}'.format(taxi_rider / total_travelers))
        print('travel_by_transit_ratio: {}'.format(transit_service_ratio))

        print()
        print('Regular user travel served ratio: {}'.format(regular_user_served / total_regular_user_travel_number))

    if bus_lines is not None:
        spacing_diviation_ratio = np.mean(np.abs(np.array(stops_spacing) - 400) / 400)
        circuity = np.array([c for c in circuity if c is not None])
    else:
        spacing_diviation_ratio = 0
        circuity = []
    
    if len(circuity) != 0:
        inv_circuity_mean = np.nanmean(1 / np.array(circuity))
    else:
        inv_circuity_mean = 0
    
    if global_eva:
        individual_scores = np.array([transit_benefit_cost_ratio, transit_service_ratio, ave_TSR, GHG_saving_ratio, bus_top_od_cover_ratio, bus_regular_user_serving_ratio, inv_circuity_mean, - spacing_diviation_ratio])
    else:
        individual_scores = np.array([bus_benefit_cost_ratio, bus_service_ratio, bus_ave_TSR, bus_GHG_saving_ratio, bus_top_od_cover_ratio, bus_regular_user_serving_ratio, inv_circuity_mean, - spacing_diviation_ratio])
    
    value_coefficients = np.array(value_coefficients)
    scores = individual_scores * value_coefficients
    score = np.sum(scores)
    if print_:
        print('============')
        print('Scores:')
        eva_terms = ['benefit_cost_ratio', 'service_ratio', 'average_time_saving_ratio', 'GHG_saving_ratio', 'bus_top_od_coverage', 'bus_regular_user_serving_ratio', 'inv_circuity_mean', 'spacing_diviation_ratio']
        for i, term in enumerate(eva_terms):
            print('{}: {}, score: {}'.format(term, individual_scores[i], scores[i]))
        print()
        print('*******')
        print('Overall score: ', score)

    if log2file:
        logger.info('Transit Configuration: ')
        logger.info('Bus Routes: {}, Bus Fleet Size: {}'.format(bus_lines, bus_fleet_size))
        logger.info('Taxi Fleet Size: {}'.format(taxi_fleet_size))
        logger.info('Average transit time saving ratio: {}'.format(ave_TSR)) 
        logger.info('Transit system GHG emission: {}'.format(transit_GHG_emission))
        logger.info('Private car GHG emission: {}'.format(private_car_GHG_emission))
        logger.info('Transit GHG emission saving ratio: {}'.format(GHG_saving_ratio))
        logger.info('Average taxi_travel_time: {}'.format(np.mean(taxi_travel_time_list)))
        logger.info('\n')
        logger.info('Waiting Time')
        logger.info('total_bus_wait_time: {}'.format(total_bus_wait_time))
        logger.info('average_bus_wait_time: {}'.format(average_bus_wait_time))
        logger.info('total_taxi_wait_time: {}'.format(total_taxi_wait_time))
        logger.info('average_taxi_wait_time: {}'.format(average_taxi_wait_time))
        logger.info('total_transit_wait_time: {}'.format(total_transit_wait_time))
        logger.info('average_transit_wait_time: {}'.format(average_transit_wait_time))
        logger.info('\n')
        logger.info('Benefit-cost Ratio')
        logger.info('bus_benefit: {}'.format(bus_benefit))
        logger.info('bus_benefit_cost_ratio: {}'.format(bus_benefit_cost_ratio))
        logger.info('taxi_benefit: {}'.format(taxi_benefit))
        logger.info('taxi_benefit_cost_ratio: {}'.format(taxi_benefit_cost_ratio))
        logger.info('transit_benefit: {}'.format(taxi_benefit + bus_benefit))
        logger.info('transit_benefit_cost_ratio: {}'.format(transit_benefit_cost_ratio))
        if bus_rider != 0:
            logger.info('Benefit per bus rider: {}'.format(bus_benefit / bus_rider))    
        if taxi_rider != 0:
            logger.info('Benefit per taxi rider: {}'.format(taxi_benefit / taxi_rider))
        logger.info('Benefit per transit rider: {}'.format(benefit_per_transit_rider))
        logger.info('\n')
        if bus_rider != 0:
            logger.info('Bus cost per ridership: {}'.format((bus_cost) / (bus_rider)))
        if taxi_rider != 0:
            logger.info('Taxi cost per ridership: {}'.format((taxi_cost) / (taxi_rider)))
        logger.info('Transit per ridership cost: {}'.format(transit_cost_per_rider))
        logger.info('Transit per rider cost - benefit: {}'.format(transit_cost_per_rider - benefit_per_transit_rider))
        logger.info('\n')
        logger.info('travel_by_walk_ratio: {}'.format(walker / total_travelers))
        logger.info('travel_by_bus_ratio: {}'.format(bus_rider / total_travelers))
        logger.info('travel_by_taxi_ratio: {}'.format(taxi_rider / total_travelers))
        logger.info('travel_by_transit_ratio: {}'.format(transit_service_ratio))
        logger.info('Regular user travel served ratio: {}'.format(regular_user_served / total_regular_user_travel_number))
        logger.info('======Scores======')
        eva_terms = ['benefit_cost_ratio', 'service_ratio', 'average_time_saving_ratio', 'GHG_saving_ratio', 'bus_top_od_coverage', 'bus_regular_user_serving_ratio', 'inv_circuity_mean', 'spacing_diviation_ratio']
        for i, term in enumerate(eva_terms):
            logger.info('{}: {}, score: {}'.format(term, individual_scores[i], scores[i]))
        logger.info('Overall score: {}'.format(score))
        logger.info('\n')
        
    if not return_dist_lists:
        return score, (transit_cost_per_rider, transit_benefit_cost_ratio, transit_benefit, transit_service_ratio, average_transit_wait_time, ave_TSR, transit_GHG_emission, GHG_saving_ratio)
    else:
        return score, (transit_cost_per_rider, transit_benefit_cost_ratio, transit_benefit, transit_service_ratio, average_transit_wait_time, ave_TSR, transit_GHG_emission, GHG_saving_ratio), (bus_walk_to_pickup_distance_list, bus_walk_to_des_distance_list, bus_walk_transfer_distance_list, bus_vehicle_distance_list, bus_trip_distance_list, taxi_trip_distance_list, walk_trip_distance_list)

    # bus_walk_to_pickup_distance_list = []
    # bus_walk_to_des_distance_list = []
    # bus_walk_transfer_distance_list = []
    # bus_vehicle_distance_list = []
    # bus_trip_distance_list = []

    # taxi_trip_distance_list = []

    # walk_trip_distance_list = []

def demand_allocation(taxi_fleet_size, full_dt_data, bus_lines, bus_fleet_size, schedule, daily_bus_milage, bus_intersection_time_loss_dict, G_eva, G_routing, traffic_signals_list, signal_bundle_dict, intersections_xy_dict, stop_time_loss, no_repeat, complex_mode, unit_bus_cost, unit_taxi_cost, intersection_time_loss, cutoff = None, print_ = True, log2file = False):
    ER_fossil = 256.274 # g CO2eq / km
    ER_electric = 76.67 # g CO2eq / km
    
    time_dict_eva, length_dict_eva = create_dict(G_eva)
    origins_node_list = full_dt_data.origin_nid.to_list()
    destinations_node_list = full_dt_data.destination_nid.to_list()
    drive_distance_list = full_dt_data.drive_distance.to_list()
    traveler_number_list = np.array(full_dt_data.passengers.to_list())
    date_list = full_dt_data.date.to_list()
    regular_user_index = np.array(full_dt_data.regular_user.to_list())
    total_regular_user_travel_number = np.sum(traveler_number_list[regular_user_index == True])
    day_count = 1
    start_time_sec_list = full_dt_data.start_time_sec.to_list()
    taxi_fleet = Taxi_Fleet(taxi_fleet_size, G_routing, intersection_time_loss, traffic_signals_list, signal_bundle_dict)
    ## Multi-layer bus transit network generation
    time_dict_routing, length_dict_routing = create_dict(G_routing, cal_bus = False)
    if bus_lines is not None:
        G_eva_bus, _, _, _, _, _, _ = graph_creation(G_eva, G_routing, bus_lines, bus_fleet_size, traffic_signals_list, time_dict_routing, length_dict_routing, intersections_xy_dict, signal_bundle_dict, intersection_time_loss, stop_time_loss = stop_time_loss, no_repeat = no_repeat, complex_mode = complex_mode, cutoff = cutoff, zero_wait = True)
        time_dict, length_dict, bus_dict = create_dict(G_eva_bus, cal_bus = True)
        bus_cost = np.sum(bus_fleet_size) * unit_bus_cost
    else:
        bus_cost = 0
    taxi_cost = taxi_fleet_size * unit_taxi_cost
    total_cost = taxi_cost + bus_cost
    
    walker = 0
    bus_rider = 0
    taxi_rider = 0
    regular_user_served = 0
    total_transit_traveler_trip_dist = 0
#     total_taxi_traveler_trip_dist = 0
    bus_wait_time_list = []
    taxi_wait_time_list = []
    bus_time_saving_list = []
    taxi_time_saving_list = []
    taxi_travel_time_list = []
    transit_time_saving_ratio_list = []
    
    total_taxi_milage = 0
    previous_entry_date = date_list[0]
    for trip_ in range(len(origins_node_list)):
        source = origins_node_list[trip_]
        target = destinations_node_list[trip_]
        order_time = start_time_sec_list[trip_]
        date = date_list[trip_]
        if date != previous_entry_date:
#             print(date)
#             daily_ft, mean_daily_ft = taxi_fleet.daily_free_time()
#             print(daily_ft)
#             print(mean_daily_ft)
#             print('---')
            day_count += 1
            previous_entry_date = date
            total_taxi_milage += taxi_fleet.daily_total_dist()
            taxi_fleet = Taxi_Fleet(taxi_fleet_size, G_routing, intersection_time_loss, traffic_signals_list, signal_bundle_dict)
        ### Calculate walk time
        walk_time, walk_dist, walk_paths = multi_source_dijkstra(G_eva, [source], targets = [target], time_dict = time_dict_eva, length_dict = length_dict_eva)
        walk_time, walk_dist, walk_path = walk_time[target], walk_dist[target], walk_paths[target]
        walk_time += intersection_time_loss * len(list_intersection(walk_path, traffic_signals_list, signal_bundle_dict))
        
        ### Calculate fixed-route bus transit time (if bus+walk time shorter than walking)
#         multi_source_dijkstra(G_eva, [source], targets = [target], time_dict = time_dict_eva, length_dict = length_dict_eva)
        if bus_lines is not None:
            bus_time, bus_dist, bus_paths, bus_travel_modes, _ = multi_source_dijkstra(G_eva_bus, [source], targets = [target], time_dict = time_dict, length_dict = length_dict, bus_dict = bus_dict)
            bus_time, bus_dist, bus_path, bus_travel_mode = bus_time[target], bus_dist[target], bus_paths[target], bus_travel_modes[target]
    #         bus_time += intersection_time_loss * len(list_intersection(bus_path, traffic_signals_list, signal_bundle_dict))
            current_time = order_time

            if bus_travel_mode == True:
                bus_time = 0
                path_chunks = bus_walk_separation(bus_path)
                for chunk in path_chunks:
                    for i in range(len(chunk) - 1):
                        bus_time += time_dict[chunk[i]][chunk[i + 1]]
                    if isinstance(chunk[0], int):
                        # walk intersection time loss
                        bus_time += intersection_time_loss * len(list_intersection(chunk, traffic_signals_list, signal_bundle_dict))
                        current_time = order_time + bus_time
                    else:
                        bus_wait_time = time_gap_cal(schedule[chunk[0]], current_time)
                        bus_time += bus_wait_time
                        for i in range(len(chunk) - 1):
                            bus_time += bus_intersection_time_loss_dict[chunk[i]][chunk[i + 1]] # Bus intersection time loss
                        current_time = order_time + bus_time

            if bus_time > walk_time:
                bus_travel_mode = False
        else:
            bus_travel_mode = False
            bus_time = np.inf
#         if (bus_travel_mode == True) and (np.isinf(bus_time) or np.isinf(bus_wait_time)):
#             print('GIAO!!!!')
#             print(walk_time)
#             print(bus_time)
#             print(bus_wait_time)
#             print()
        ### Calculate ride-hailing travel time
        taxi_time, taxi_wait_time, pu_heading_time, taxi_travel_time, taxi_trip_dist, taxi_id = taxi_fleet.wait_travel_time_cal(source, target, order_time)
#         print(taxi_time)

        if bus_travel_mode == True:
            if bus_time <= taxi_time:
                bus_rider += traveler_number_list[trip_]
                total_transit_traveler_trip_dist += traveler_number_list[trip_] * drive_distance_list[trip_]
                if regular_user_index[trip_]:
                    regular_user_served += traveler_number_list[trip_]
                for traveler_ in range(traveler_number_list[trip_]):
                    bus_wait_time_list.append(bus_wait_time)
                    bus_time_saving_list.append(walk_time - bus_time)
                    transit_time_saving_ratio_list.append((walk_time - bus_time) / walk_time)
            else:
                total_transit_traveler_trip_dist += traveler_number_list[trip_] * drive_distance_list[trip_]
                if regular_user_index[trip_]:
                    regular_user_served += traveler_number_list[trip_]
                taxi_rider += traveler_number_list[trip_]
                taxi_fleet.assign_passenger_car(source, target, order_time, taxi_time, pu_heading_time, taxi_travel_time, taxi_trip_dist, taxi_id)
                for traveler_ in range(traveler_number_list[trip_]):
                    taxi_wait_time_list.append(taxi_wait_time)
                    taxi_time_saving_list.append(walk_time - taxi_time)
                    transit_time_saving_ratio_list.append((walk_time - taxi_time) / walk_time)
                    taxi_travel_time_list.append(taxi_travel_time)
#                 print((walk_time, bus_time, taxi_time))
        else:
            if walk_time < taxi_time:
                walker += traveler_number_list[trip_]
            else:
                total_transit_traveler_trip_dist += traveler_number_list[trip_] * drive_distance_list[trip_]
                if regular_user_index[trip_]:
                    regular_user_served += traveler_number_list[trip_]
                taxi_rider += traveler_number_list[trip_]
                taxi_fleet.assign_passenger_car(source, target, order_time, taxi_time, pu_heading_time, taxi_travel_time, taxi_trip_dist, taxi_id)
                for traveler_ in range(traveler_number_list[trip_]):
                    taxi_wait_time_list.append(taxi_wait_time)
                    taxi_time_saving_list.append(walk_time - taxi_time)
                    transit_time_saving_ratio_list.append((walk_time - taxi_time) / walk_time)
                    taxi_travel_time_list.append(taxi_travel_time)
                    
#         if not np.isinf(taxi_time):
#             taxi_rider += traveler_number_list[trip_]
#             taxi_fleet.assign_passenger_car(source, target, order_time, taxi_time, taxi_id)
#             for traveler_ in range(traveler_number_list[trip_]):
#                 taxi_wait_time_list.append(taxi_wait_time)
#                 taxi_time_saving_list.append(walk_time - taxi_time)
#                 taxi_travel_time_list.append(taxi_travel_time)

#                 print((walk_time, bus_time, taxi_time))
    
    ## Wait time
#     if bus_lines is not None:
    if bus_lines is not None:
        total_bus_milage = daily_bus_milage * day_count / 1000
    else:
        total_bus_milage = 0
    total_taxi_milage = total_taxi_milage / 1000
    total_transit_traveler_trip_dist = total_transit_traveler_trip_dist / 1000
    
    total_bus_GHG_emission = total_bus_milage * ER_electric
    total_taxi_GHG_emission = total_taxi_milage * ER_electric
    
    transit_GHG_emission = total_bus_GHG_emission + total_taxi_GHG_emission
    private_car_GHG_emission = total_transit_traveler_trip_dist * ER_fossil
    
    GHG_saving_ratio = (private_car_GHG_emission - transit_GHG_emission) / private_car_GHG_emission
    
    total_bus_wait_time = np.sum(bus_wait_time_list)
    average_bus_wait_time = np.mean(bus_wait_time_list)
    
    total_taxi_wait_time = np.sum(taxi_wait_time_list)
    average_taxi_wait_time = np.mean(taxi_wait_time_list)
    
#     if bus_lines is not None:
    total_transit_wait_time = np.sum(bus_wait_time_list + taxi_wait_time_list)
    average_transit_wait_time = np.mean(bus_wait_time_list + taxi_wait_time_list)
    
    ## Benefit-cost ratio
    bus_benefit = np.sum(bus_time_saving_list) * 0.004961
    bus_benefit_cost_ratio = bus_benefit / bus_cost
    
    taxi_benefit = np.sum(taxi_time_saving_list) * 0.004961
    taxi_benefit_cost_ratio = taxi_benefit / taxi_cost
    if print_:
        print()
    
    transit_benefit = np.sum(bus_time_saving_list + taxi_time_saving_list) * 0.004961
    transit_benefit_cost_ratio = transit_benefit / total_cost
    
    ave_TSR = np.mean(transit_time_saving_ratio_list)
    
    if print_:
        print('Average transit time saving ratio: {}'.format(ave_TSR))
        
        print('Transit system GHG emission: {}'.format(transit_GHG_emission))
        print('Private car GHG emission: {}'.format(private_car_GHG_emission))
        print('Transit GHG emission saving ratio: {}'.format(GHG_saving_ratio))
        
        print('Average taxi_travel_time: {}'.format(np.mean(taxi_travel_time_list)))
        print()
    
        print('Waiting Time')
        print('total_bus_wait_time: {}'.format(total_bus_wait_time))
        print('average_bus_wait_time: {}'.format(average_bus_wait_time))
        print('total_taxi_wait_time: {}'.format(total_taxi_wait_time))
        print('average_taxi_wait_time: {}'.format(average_taxi_wait_time))
        print('total_transit_wait_time: {}'.format(total_transit_wait_time))
        print('average_transit_wait_time: {}'.format(average_transit_wait_time))
        print()
    
        print('Benefit-cost Ratio')
        print('bus_benefit: {}'.format(bus_benefit))
        print('bus_benefit_cost_ratio: {}'.format(bus_benefit_cost_ratio))
        print('taxi_benefit: {}'.format(taxi_benefit))
        print('taxi_benefit_cost_ratio: {}'.format(taxi_benefit_cost_ratio))
        print('transit_benefit: {}'.format(taxi_benefit + bus_benefit))
        print('transit_benefit_cost_ratio: {}'.format(transit_benefit_cost_ratio))
    
        if bus_rider != 0:
            print('Benefit per bus rider: {}'.format(bus_benefit / bus_rider))    
        if taxi_rider != 0:
            print('Benefit per taxi rider: {}'.format(taxi_benefit / taxi_rider))
    benefit_per_transit_rider = transit_benefit / (bus_rider + taxi_rider)
    if print_:
        print('Benefit per transit rider: {}'.format(benefit_per_transit_rider))

        print()
        if bus_rider != 0:
            print('Bus cost per ridership: {}'.format((bus_cost) / (bus_rider)))
        if taxi_rider != 0:
            print('Taxi cost per ridership: {}'.format((taxi_cost) / (taxi_rider)))
    transit_cost_per_rider = (bus_cost + taxi_cost) / (bus_rider + taxi_rider)
    if print_:
        print('Transit per ridership cost: {}'.format(transit_cost_per_rider))
        print('Transit per rider cost - benefit: {}'.format(transit_cost_per_rider - benefit_per_transit_rider))

        print()
    total_travelers = walker + bus_rider + taxi_rider
    transit_service_ratio = (bus_rider + taxi_rider) / total_travelers
    if print_:
        print('travel_by_walk_ratio: {}'.format(walker / total_travelers))
        print('travel_by_bus_ratio: {}'.format(bus_rider / total_travelers))
        print('travel_by_taxi_ratio: {}'.format(taxi_rider / total_travelers))
        print('travel_by_transit_ratio: {}'.format(transit_service_ratio))

        print()
        print('Regular user travel served ratio: {}'.format(regular_user_served / total_regular_user_travel_number))

    if log2file:
        logger.info('Transit Configuration: ')
        logger.info('Bus Routes: {}, Bus Fleet Size: {}'.format(bus_lines, bus_fleet_size))
        logger.info('Taxi Fleet Size: {}'.format(taxi_fleet_size))
        logger.info('Average transit time saving ratio: {}'.format(ave_TSR))
        logger.info('Transit system GHG emission: {}'.format(transit_GHG_emission))
        logger.info('Private car GHG emission: {}'.format(private_car_GHG_emission))
        logger.info('Transit GHG emission saving ratio: {}'.format(GHG_saving_ratio))
        logger.info('Average taxi_travel_time: {}'.format(np.mean(taxi_travel_time_list)))
        logger.info('Waiting Time')
        logger.info('total_bus_wait_time: {}'.format(total_bus_wait_time))
        logger.info('average_bus_wait_time: {}'.format(average_bus_wait_time))
        logger.info('total_taxi_wait_time: {}'.format(total_taxi_wait_time))
        logger.info('average_taxi_wait_time: {}'.format(average_taxi_wait_time))
        logger.info('total_transit_wait_time: {}'.format(total_transit_wait_time))
        logger.info('average_transit_wait_time: {}'.format(average_transit_wait_time))
        logger.info('Benefit-cost Ratio')
        logger.info('bus_benefit: {}'.format(bus_benefit))
        logger.info('bus_benefit_cost_ratio: {}'.format(bus_benefit_cost_ratio))
        logger.info('taxi_benefit: {}'.format(taxi_benefit))
        logger.info('taxi_benefit_cost_ratio: {}'.format(taxi_benefit_cost_ratio))
        logger.info('transit_benefit: {}'.format(taxi_benefit + bus_benefit))
        logger.info('transit_benefit_cost_ratio: {}'.format(transit_benefit_cost_ratio))
        if bus_rider != 0:
            logger.info('Benefit per bus rider: {}'.format(bus_benefit / bus_rider))    
        if taxi_rider != 0:
            logger.info('Benefit per taxi rider: {}'.format(taxi_benefit / taxi_rider))

        logger.info('Benefit per transit rider: {}'.format(benefit_per_transit_rider))
        if bus_rider != 0:
            logger.info('Bus cost per ridership: {}'.format((bus_cost) / (bus_rider)))
        if taxi_rider != 0:
            logger.info('Taxi cost per ridership: {}'.format((taxi_cost) / (taxi_rider)))
        logger.info('Transit per ridership cost: {}'.format(transit_cost_per_rider))
        logger.info('Transit per rider cost - benefit: {}'.format(transit_cost_per_rider - benefit_per_transit_rider))
        logger.info('travel_by_walk_ratio: {}'.format(walker / total_travelers))
        logger.info('travel_by_bus_ratio: {}'.format(bus_rider / total_travelers))
        logger.info('travel_by_taxi_ratio: {}'.format(taxi_rider / total_travelers))
        logger.info('travel_by_transit_ratio: {}'.format(transit_service_ratio))
        logger.info('Regular user travel served ratio: {}'.format(regular_user_served / total_regular_user_travel_number))
        logger.info('\n')

    return transit_cost_per_rider, transit_benefit_cost_ratio, transit_benefit, transit_service_ratio, average_transit_wait_time, ave_TSR, transit_GHG_emission, GHG_saving_ratio


def create_dict(G_routing, cal_bus = False):
    time_dict = {}
    length_dict = {}
    bus_dict = {}
    for nid, _ in G_routing.nodes(data = True):
        time_dict[nid] = {}
        length_dict[nid] = {}
        bus_dict[nid] = {}
    if cal_bus == False:
        for u, v, key, data in G_routing.out_edges(data=True, keys=True):
            time_dict[u][v] = data['travel_time']
            length_dict[u][v] = data['length']
        return time_dict, length_dict
    else:
        for u, v, key, data in G_routing.out_edges(data=True, keys=True):
            time_dict[u][v] = data['travel_time']
            length_dict[u][v] = data['length']
            bus_dict[u][v] = data['bus']
        return time_dict, length_dict, bus_dict

def set_append(s, v, inplace = False):
    if inplace:
        return s.add(v)
    else:
        s_extent = copy.deepcopy(s)
        s_extent.add(v)
        return s_extent

def multi_source_dijkstra(G, sources, targets=None, cutoff=None, time_dict = None, length_dict = None, bus_dict = None, actual_tolerance = None, wait_time = None):    
    if not sources:
        raise ValueError("sources must not be empty")
    paths = {source: [source] for source in sources}  # dictionary of paths
    if bus_dict is not None:
        dist, length, travel_mode, trip_wait_time = _dijkstra_multisource(
            G, sources, time_dict, length_dict, bus_dict=bus_dict, paths=paths, cutoff=cutoff, targets=targets, actual_tolerance=actual_tolerance, wait_time = wait_time
        )
    else:
        dist, length = _dijkstra_multisource(
            G, sources, time_dict, length_dict, bus_dict=bus_dict, paths=paths, cutoff=cutoff, targets=targets, actual_tolerance=actual_tolerance, wait_time = wait_time
        )
        
    try:
        if bus_dict is not None:
            return (dist, length, paths, travel_mode, trip_wait_time)
        else:
            return (dist, length, paths)
    except KeyError as e:
        raise nx.NetworkXNoPath(f"No path to {targets}.") from e

def _dijkstra_multisource(
    G, sources, time_dict, length_dict, bus_dict=None, pred=None, paths=None, cutoff=None, targets=None, actual_tolerance = None, wait_time = None
):
    G_succ = G._succ if G.is_directed() else G._adj
    targets_init = copy.deepcopy(targets)
    push = heappush
    pop = heappop
    dist = {}  # dictionary of final distances
    seen = {}
    travel_mode = {}
    length = {}
    if bus_dict is not None:
        bus_line_pass = {} # set([0])
    # fringe is heapq with 3-tuples (distance,c,node)
    # use the count c to avoid comparing nodes (may not be able to)
    c = count()
    fringe = []
#     bus_examin = _weight_function(G, 'bus')
#     path_length = _weight_function(G, 'length')
    for source in sources:
        if source not in G:
            raise nx.NodeNotFound(f"Source {source} not in G")
        seen[source] = 0
        travel_mode[source] = False
        if bus_dict is not None:
            bus_line_pass[source] = set([0])
#         print(bus_line_pass)
        push(fringe, (0, next(c), source))
        length[source] = 0
    while fringe:
        (d, _, v) = pop(fringe)
        if v in dist:
            continue  # already searched this node.
        dist[v] = d
        
        if v in targets:
            targets.remove(v)
            if len(targets) == 0:
                break
        for u, e in G_succ[v].items():
            cost = time_dict[v][u] # weight(v, u, e)
            if bus_dict is not None:
                bus_ind = bus_dict[v][u] #bus_examin(v, u, e)
#             if bus_ind > 0:
                bus_or_not = True if bus_ind > 0 else False
#             bus_line_pass.add(bus_ind)
            if cost is None:
                continue
            vu_dist = dist[v] + cost
            if cutoff is not None:
                if vu_dist > cutoff:
                    continue
            if u in dist:
                u_dist = dist[u]
                if vu_dist < u_dist:
                    raise ValueError("Contradictory paths found:", "negative weights?")
                elif pred is not None and vu_dist == u_dist:
                    pred[u].append(v)
            elif u not in seen or vu_dist < seen[u]:
                path_l = length_dict[v][u] # path_length(v, u, e)
                vu_length = length[v] + path_l
                length[u] = vu_length
                seen[u] = vu_dist
                push(fringe, (vu_dist, next(c), u))
                if bus_dict is not None:
                    travel_mode[u] = travel_mode[v] or bus_or_not
#                 print(bus_line_pass)
#                 print(v)
                    bus_line_pass[u] = set_append(bus_line_pass[v], bus_ind) # bus_line_pass[v].add(bus_ind)
#                 vu_bus_length = path_l if bus_or_not else 0
#                 travel_length_by_bus[u] = travel_length_by_bus[v] + vu_bus_length
                if paths is not None:
                    paths[u] = paths[v] + [u]
                if pred is not None:
                    pred[u] = [v]
            elif vu_dist == seen[u]:
                if pred is not None:
                    pred[u].append(v)
    trip_wait_time = {}
    if actual_tolerance is not None:
        trip_wait_time = {}
        for target in targets_init:
            try:
                trip_wait_time[target] = 0
                for ele in bus_line_pass[target]:
                    if ele != 0:
                        line = ele - 1
                        dist[target] += actual_tolerance[line]
                        trip_wait_time[target] += (wait_time[line] + actual_tolerance[line])
            except:
                redundant = 0
                
    # The optional predecessor and path dictionaries can be accessed
    # by the caller via the pred and paths objects passed as arguments.
    if bus_dict is not None:
        return dist, length, travel_mode, trip_wait_time
    else:
        return dist, length
        
def trim_duplicate(route_list, score_included = False):
    route_l = []
    if score_included:
        rl = copy.deepcopy(route_list[1])
    else:
        rl = copy.deepcopy(route_list)
    for r in range(len(rl)):
        stop_list = []
        pre = np.inf
        for s in range(len(rl[r])):
            if rl[r][s] != pre:
                stop_list.append(rl[r][s])
                pre = rl[r][s]
        route_l.append(stop_list)
    if score_included:
        route_list_condense = (route_list[0], route_l)
    else:
        route_list_condense = route_l
    return route_list_condense

def route_validity_check(line, G_routing, time_dict, length_dict, no_repeat = True, complex_mode = False, cutoff = None):
    route_eva = line.copy()
    # [(t, l, [inter_ids1, ...]), (), ()]
    # none
    routes_d1 = line_route_finder(G_routing, route_eva, time_dict, length_dict, no_repeat = no_repeat, complex_mode = complex_mode, cutoff = cutoff)
    routes_d2 = line_route_finder(G_routing, route_eva[::-1], time_dict, length_dict, no_repeat = no_repeat, complex_mode = complex_mode, cutoff = cutoff)
    
    if (routes_d1 is not None) and (routes_d2 is not None):
        return True
    elif ((routes_d1 is not None) or (routes_d2 is not None)) and (line[0] == line[-1]):
        return True
    else:
        return False

def bus_validity_check(route_list, G_routing, no_repeat = True, complex_mode = False, cutoff = None):
    time_dict, length_dict = create_dict(G_routing)
    for i in range(len(route_list)):
        if route_list[i][0] != route_list[i][-1]:  # Non-circular bus line, two-way operation
            routes_d1 = line_route_finder(G_routing, route_list[i], time_dict, length_dict, no_repeat = no_repeat, complex_mode = complex_mode, cutoff = cutoff)
            if routes_d1 is not None:
                routes_d2 = line_route_finder(G_routing, route_list[i][::-1], time_dict, length_dict, no_repeat = no_repeat, complex_mode = complex_mode, cutoff = cutoff)
                if routes_d2 is None:
                    return False
            else:
                return False
        else: # Circular bus line, one-way operation
            routes_d1 = line_route_finder(G_routing, route_list[i], time_dict, length_dict, no_repeat = no_repeat, complex_mode = complex_mode, cutoff = cutoff)
            if routes_d1 is None:
                routes_d2 = line_route_finder(G_routing, route_list[i][::-1], time_dict, length_dict, no_repeat = no_repeat, complex_mode = complex_mode, cutoff = cutoff)
                if routes_d2 is None:
                    return False
                else:
                    routes_ = routes_d2
            else:
                routes_ = routes_d1
    return True
    
def plot_routes(route_list, G_routing, time_dict, length_dict, route_linewidth = 10, orig_dest_size=100, route_alpha=0.5, i = None, no_repeat = True, complex_mode = False, cutoff = None, save = False, filepath = 'bus_route.pdf', figsize =(8, 8)):
    color_dict = ['r', 'b', 'y', 'c', 'm', 'g']
    plot_list = []
    
    best_routes = copy.deepcopy(route_list)
    color_list = []
    if i is None:
        for i in range(len(best_routes)):
            routes_d1 = line_route_finder(G_routing, best_routes[i], time_dict, length_dict, no_repeat = no_repeat, complex_mode = complex_mode, cutoff = cutoff)
            routes_d2 = line_route_finder(G_routing, best_routes[i][::-1], time_dict, length_dict, no_repeat = no_repeat, complex_mode = complex_mode, cutoff = cutoff)
            if routes_d1 is not None:
                routes = copy.deepcopy(routes_d1)
            else:
                routes = copy.deepcopy(routes_d2)

            for j in range(len(routes)):
                path = routes[j][2]
                plot_list.append(path)
                color_list.append(color_dict[i])
    else:
        routes_d1 = line_route_finder(G_routing, best_routes[i], time_dict, length_dict, no_repeat = True, complex_mode = False, cutoff = 10)
        routes_d2 = line_route_finder(G_routing, best_routes[i][::-1], time_dict, length_dict, no_repeat = True, complex_mode = False, cutoff = 10)
        if routes_d1 is not None:
            routes = copy.deepcopy(routes_d1)
        else:
            routes = copy.deepcopy(routes_d2)

        for j in range(len(routes)):
            path = routes[j][2]
            plot_list.append(path)
            color_list.append(color_dict[i])
            
    if len(plot_list) == 1:
        fig, ax = ox.plot_graph_route(G_routing, plot_list[0], route_color = color_list[0], bgcolor = 'white', edge_color = 'black', route_linewidth = route_linewidth, orig_dest_size=orig_dest_size, route_alpha=route_alpha, node_color = 'black', figsize = figsize, save = save, filepath = filepath)#, show = False)
    else:
        fig, ax = ox.plot_graph_routes(G_routing, plot_list, route_colors = color_list, bgcolor = 'white', edge_color = 'black', route_linewidth = route_linewidth, orig_dest_size=orig_dest_size, route_alpha=route_alpha, node_color = 'black', figsize = figsize, save = save, filepath = filepath)#, show = False)
#     ax.annotate('2---------3.5%', (1, 1))
#     fig.show()
        

#     if len(plot_list) == 1:
#         fig, ax = ox.plot_graph_route(G_routing, plot_list[0], route_color = color_list[0], bgcolor = 'white', edge_color = 'black', node_color = 'black', show = False, close=False)
#     else:
#         fig, ax = ox.plot_graph_routes(G_routing, plot_list, route_colors = color_list, bgcolor = 'white', edge_color = 'black', node_color = 'black', show = False, close=False)
    # plt.savefig('bus_route.pdf')

# plot_routes([demo_line], i = None, no_repeat = True, complex_mode = False, cutoff = 10)


def bus_assign(line_number, max_fleet, fix = False):
    fleet_size_list = []
    if fix:
        mf = max_fleet
        for i in range(20):
            ind_list = list(np.random.choice(range(1, mf, 1), line_number - 1, replace=False))
            ind_list.append(mf)
            ind_list = sorted(ind_list)
            fleet_size = [ind_list[i + 1] - ind_list[i] for i in range(len(ind_list) - 1)]
            fleet_size.insert(0, ind_list[0])
            if fleet_size not in fleet_size_list:
                fleet_size_list.append(fleet_size)
    else:        
        for mf in range(line_number, max_fleet + 1):
            for i in range(20):
                ind_list = list(np.random.choice(range(1, mf, 1), line_number - 1, replace=False))
                ind_list.append(mf)
                ind_list = sorted(ind_list)
                fleet_size = [ind_list[i + 1] - ind_list[i] for i in range(len(ind_list) - 1)]
                fleet_size.insert(0, ind_list[0])
                if fleet_size not in fleet_size_list:
                    fleet_size_list.append(fleet_size)
    return fleet_size_list

# (taxi_fleet_size, full_dt_data, bus_lines, bus_fleet_size, G_eva, G_routing, traffic_signals_list, signal_bundle_dict, intersections_xy_dict, stop_time_loss, no_repeat, complex_mode, unit_bus_cost, unit_taxi_cost, intersection_time_loss, value_coefficients = [1., 1., 1., 1., 0.2, 0.2, 0.5, 0.5], global_eva = True, cutoff = None, start_hour = 7, end_hour = 22, OD_related_input_dict = None, print_ = True)

def evaluate_bus_fleet_assign_agent_based(taxi_fleet_size, full_dt_data, route_list, max_bus_fleet_size, G_ped, G_routing, traffic_signals_list, signal_bundle_dict, intersections_xy_dict, stop_time_loss, no_repeat, complex_mode, unit_bus_cost, unit_taxi_cost, intersection_time_loss, value_coefficients = [1., 1., 1., 1., 0.2, 0.2, 0.5, 0.5], global_eva = True, cutoff = None, start_hour = 7, end_hour = 22, OD_related_input_dict = None, fix = True):
    line_number = len(route_list)
    bus_fleet_size_list = bus_assign(line_number, max_bus_fleet_size, fix = fix)
    bus_fleet_size_score_tuple_list = []
    for bus_fleet_size in bus_fleet_size_list:
#         print('Bus lines:')
#         print(route_list)
#         print('bus_fleet_size:')
#         print(bus_fleet_size)
#         print()
        score, _ = evaluation_agent_based(taxi_fleet_size, full_dt_data, route_list, bus_fleet_size, G_ped, G_routing, traffic_signals_list, signal_bundle_dict, intersections_xy_dict, stop_time_loss, no_repeat, complex_mode, unit_bus_cost, unit_taxi_cost, intersection_time_loss, OD_related_input_dict = OD_related_input_dict, global_eva = global_eva, start_hour = 7, end_hour = 22, print_ = False)
        bus_fleet_size_score_tuple_list.append((score, bus_fleet_size))
    bus_fleet_size_score_tuple_list = sorted(bus_fleet_size_score_tuple_list, key=lambda x: x[0], reverse = True)
    return bus_fleet_size_score_tuple_list[0][0], bus_fleet_size_score_tuple_list[0][1]
