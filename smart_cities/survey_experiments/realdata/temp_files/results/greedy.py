import networkx as nx
import numpy as np
import sys
filepath = '/home/hushiji/Research/smart_cities/smart_cities/survey_experiments/realdata'
sys.path.insert(0, '/home/hushiji/Research/smart_cities/smart_cities/survey_experiments/realdata/')
import create_data
import matplotlib.pyplot as plt
import os.path
import make_plots
import write_data

soc_lower_bound = 0.8
budget = create_data.budget

def greedy_algorithm(graph, seed):
    myroutes = make_plots.get_base_routes(seed)
    install_set = set()
    current_budget = budget
    for pos in myroutes:
        current_budget = update_install(graph, myroutes[pos], install_set, current_budget)
    return install_set
    
    
def update_install(graph, path, install_set, current_budget):
    current_range_soc = 1
    for roadSeg in path:
        if roadSeg in install_set:
            INSTALL = True
        else:
            INSTALL = False
        velocity = graph.node[roadSeg]['speed_urban']
        distance = graph.node[roadSeg]['length']*0.00621371 #real soc no decrease or increase speed or distance
        next_range_soc = write_data.nextSOC_real_data(current_range_soc, distance, 1, velocity, INSTALL)
        cost = graph.node[roadSeg]['length']*0.236
        if next_range_soc < soc_lower_bound and current_budget - cost > 0:
            INSTALL = True
            next_range_soc = write_data.nextSOC_real_data(current_range_soc, distance, 1, velocity, INSTALL)
            install_set.add(roadSeg)
            current_budget -= cost
        current_range_soc = next_range_soc
    return current_budget
 
 
 
if __name__ == '__main__':
    graph = create_data.roadSegGraph
    seed = 96332
    install_set = greedy_algorithm(graph, seed)
    print install_set

