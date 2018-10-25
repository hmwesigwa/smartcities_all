"""
Given an initial set of install locations, this script
determines an initial feasible solution to the opt model,
and passes it to cplex to improve on.

Input: routesFile, roadcatFile, initial_socFile,
    set of initial solution
Output: improved solution


NOTE: routes in routeFiles have index shifted by 2, final
road seg is always 1
      Roadcategory[node] takes the real indexing, starting from 1.
i.e node 10 in route is node 8 in road cat

NOTE: roadSeg == 2 does not exist, start index is 3

"""

from pyomo.environ import *
# import the module that contains your model
import create_model
import create_data
import sys
import networkx as nx
import random
import re
import copy


def initialize_edge(node,
                    instance,
                    route_no,
                    install_dict,
                    nLayers,
                    Roadcategory,
                    Routes, 
                    initial_soc,
                    install_used):
    """ initilize edge
    This function finds the next edge and initilizes it

    instance.x[route_no, node, next_node] = 1
    Return next_node
    """
    roadSeg = create_data.node_to_roadSeg(node, nLayers)
    #install_used[roadSeg] = True
    #install_used[str(int(roadSeg) -2)]= True
    #assert roadSeg == Routes[route_no][0]
    soc = create_data.node_to_soc(node, nLayers)
    if soc < 1.0e-10 and roadSeg != Routes[route_no][0]:
        instance.x[route_no, node, 'destination'] = 1
        next_node = 'destination'
        return next_node, instance, install_used
    if roadSeg == Routes[route_no][-1]:
        instance.x[route_no, node, 'destination'] = 1
        next_node = 'destination'
        return next_node, instance, install_used
    elif roadSeg == Routes[route_no][0]:
        install_used[str(int(roadSeg) -2)]= True
        next_roadSeg = Routes[route_no][1]
        next_start_node = create_data.roadSeg_to_node(next_roadSeg, nLayers)
        init_soc = initial_soc[route_no]
        current_posn = create_data.position_in_layer(init_soc, nLayers)
        top_node = create_data.roadSeg_to_node(roadSeg, nLayers)
        if node == top_node + current_posn:
            road_cat = Roadcategory[str(roadSeg -2)]
            next_install_posn = max(current_posn - road_cat, 0)
            next_no_install_posn = min(current_posn + road_cat, nLayers-1)
            if road_cat != 0:
                if install_dict[str(roadSeg -2)]:
                    next_node = next_start_node + next_install_posn
                    instance.x[route_no, node, next_node] = 1
                    #out.append(next_start_node + next_install_posn)
                    return next_node, instance, install_used
                else:
                    next_node = next_start_node + next_no_install_posn
                    instance.x[route_no, node, next_node] = 1
                    #out.append(next_start_node + next_no_install_posn )
                    return next_node, instance, install_used
            else:
                next_node = next_start_node + next_no_install_posn + road_cat
                instance.x[route_no, node, next_node] = 1
                #out.append(next_start_node + next_no_install_posn + road_cat)
                return next_node, instance, install_used
        else:
            raise ValueError('node != top_node + current_posn')
    else:
        install_used[str(int(roadSeg) -2)]= True
        roadSeg_indx = Routes[route_no].index(roadSeg)     
        next_roadSeg = Routes[route_no][roadSeg_indx + 1]
        next_start_node = create_data.roadSeg_to_node(next_roadSeg, nLayers)
        current_soc = create_data.node_to_soc(node, nLayers)
        #road_cat = create_data.road_category[str(roadSeg -2)]
        road_cat = Roadcategory[str(roadSeg -2)]
        if current_soc > 1.0e-10 and road_cat != 0:
            #INSTALL = True
            #INSTALL = True
            if install_dict[str(roadSeg -2)]:
                INSTALL = True
                next_soc = create_data.nextSOC(current_soc, INSTALL,
                                               nLayers, road_cat)
                next_node_posn = create_data.position_in_layer(next_soc,
                                                               nLayers)
                next_node = next_start_node + next_node_posn
                instance.x[route_no, node, next_node] = 1
                #out.append(next_node)
                return next_node, instance, install_used
            else:
                # INSTALL = False
                INSTALL = False
                next_soc = create_data.nextSOC(current_soc,
                                               INSTALL, nLayers, road_cat)
                next_node_posn = create_data.position_in_layer(next_soc,
                                                               nLayers)
                next_node = next_start_node + next_node_posn

                # I believe next if was to make sure no repeated edges
                #if next_node != out[-1]:
                #   out.append(next_node)
                instance.x[route_no, node, next_node] = 1
                return next_node, instance, install_used
        elif current_soc > 1.0e-10 and road_cat == 0:
            #INSTALL = False
            INSTALL = False
            next_soc = create_data.nextSOC(current_soc,
                                           INSTALL, nLayers, road_cat)
            next_node_posn = create_data.position_in_layer(next_soc,
                                                           nLayers)
            next_node = next_start_node + next_node_posn
            #out.append(next_node)
            instance.x[route_no, node, next_node] = 1
            return next_node, instance, install_used
    

def initial_variable_based_on_route(instance,
                                    route_no,
                                    install_dict,
                                    nLayers,
                                    Roadcategory,
                                    Routes, 
                                    initial_soc,
                                    install_used  # road may not have enough soc
                                    ):
    """
    For each u in route, set roadInstall[u] = 1, and
    x[route_no, node1, node2] = 0 or 1 accordingly.

    NOTE: before running this function, all variables should be
    set to 0, otherwise some variable will be an initialized
    """

    # initialize (route_no, source, node)
        # assume full soc for now, shoud be changed for init_soc
    next_node = create_data.roadSeg_to_node(Routes[route_no][0], nLayers)
    instance.x[route_no, 'source', next_node] = 1
    #print next_node
    roadSeg = Routes[route_no][0]
    install_used[str(int(roadSeg) -2)]= True

    # initialize all edges for given route_no
    while next_node != 'destination':
        next_node, instance, install_used = initialize_edge(next_node,
                                                            instance,
                                                            route_no,
                                                            install_dict,
                                                            nLayers,
                                                            Roadcategory,
                                                            Routes, 
                                                            initial_soc,
                                                            install_used)
    return instance, install_used


def preprocess():
    """
    Read sys.argv[-1] and return
    Roadcategory, Routes, initial_soc 
    """
    mystr = sys.argv[-1]
    routefile = mystr.replace('.dat', '_routes.txt')
    roadcatfile = mystr.replace('.dat', '_road_cat.txt')
    Roadcategory = {}
    Routes = {}
    initial_soc = {}

    # get road category from file
    myFile = open(roadcatfile, 'r')
    for line in myFile:
      line = line.split()
      node, cat = line
      Roadcategory[str(int(node))] = int(cat)
    myFile.close()
    myFile = open(routefile, 'r')

    # get routes from file
    initial_soc_file = open("/home/hushiji/Research/smart_cities/\
smart_cities/general_soc/analysis/iterative_initial_soc.txt", "r")
    count = 1
    for line in myFile:
      lines_split = line.split()
      route = [int(r) for r in lines_split]
      Routes[count] = route
      count += 1
    myFile.close()

    # get initial soc from file
    count = 1
    for line in initial_soc_file:
      initial_soc[count] = float(line)
      count += 1
    initial_soc_file.close()
    return Roadcategory, Routes, initial_soc


def initialize2zero(instance):
    """ initilize all variables to 0."""
    for varname in instance.component_objects(Var):  # Var is pyomo imported
        varobject = getattr(instance, str(varname))  # instance.str(v)
        for index in varobject:
            varobject[index] = 0  # instance.v[index] = 0
    return instance

def sorted_central_nodes(centrality):
    cen_node = [(centrality[node], node) for node in centrality]
    cen_node = sorted(cen_node, reverse=True)
    return [node for _, node in cen_node]


def freq_counter_centrality(Routes):
    """ return counter of number of times node is used
    if all used once, give preference to nodes closer to origin

    NOTE: indexing reduced by 2 to match original
    """
    centrality = {}
    for route_no in Routes:
        path = Routes[route_no][:-1]
        dist_from_origin = 1
        for node in path:
            road = str(int(node) - 2)
            if road not in centrality:
                centrality[road] = 1 + 10.0**(-dist_from_origin)
            else:
                centrality[road] += 1
                centrality[road] += 10.0**(-dist_from_origin)
            dist_from_origin += 1
    return centrality



def get_centrality_solution(Routes, budget_percentage):
    """ Get init soln using centrality """

    comp_graph = create_data.get_manhattan_neigh_graph()
    route_graph = nx.Graph()

    for route_no in Routes:
        path = Routes[route_no]
        for i in range(len(path[:-2])): # dont include final roadseg 1
            node1 = str(int(path[i]) - 2)
            node2  = str(int(path[i + 1]) -2)
            route_graph.add_edge(node1, node2)
            route_graph.node[node1]['length'] = comp_graph.node[node1]['length']
            route_graph.node[node1]['length'] = comp_graph.node[node1]['length']
    total_length_graph = 0  # length of road in comp_graph
    total_length_routes = 0  # length of road in route_graph

    for node in route_graph.nodes():
        total_length_routes += comp_graph.node[node]['length']
    for node in comp_graph.nodes():
        total_length_graph += comp_graph.node[node]['length']
    
    max_length =  budget_percentage*total_length_routes
    print 'total_length_routes:', total_length_routes*0.236
    print 'budget_percentage:', budget_percentage*0.236
    print 'max_length:', max_length*0.236
    #centrality = nx.betweenness_centrality(route_graph)
    centrality = freq_counter_centrality(Routes)
    sorted_nodes = sorted_central_nodes(centrality)

    install_dict = dict.fromkeys(comp_graph.nodes(), False)

    distance_used = 0
    out = []
    for node in sorted_nodes:
        d = float(comp_graph.node[node]['length'])
        if distance_used + d < max_length:
            install_dict[node] = True
            distance_used += d
            out.append(int(node) + 2)
    return install_dict


def get_install_dict():
    """Example install dictionary for debug purposes.
    Has budget of 0.1 of total routes used
    """
    init = [26, 44, 62, 83, 101, 114, 142, 157, 169, 193,\
    236, 284, 293, 309, 311, 341, 345, 346, 370, 378, 404,\
    405, 408, 411, 428, 430, 435, 452, 461, 473, 474, 494,\
    503, 504, 519, 553, 578, 591, 615, 617, 618, 643, 668,\
    676, 681, 685, 705, 711, 716, 717, 720, 755, 763, 778,\
    797, 798, 819, 846, 907]

    init = [5,7,17,18,37,42,44,47,56,59,61,62,74,79,97,114,\
    142,150,173,193,196,208,221,236,255,256,266,272,274,280,\
    284,308,309,315,316,343,344,346,347,357,359,361,369,370,\
    378,380,391,393,399,404,405,411,412,428,430,435,439,444,\
    462,488,492,494,503,504,516,527,532,540,550,552,553,572,\
    578,588,602,615,616,618,630,633,643,644,657,680,685,703,\
    710,720,735,736,748,755,759,760,764,776,779,798,803,809,\
    810,826,829,853,857,871,874,883,911,916]


    init = [5,7,17,18]#37,42,44,47,56,59,61] #,62,74,79,97,114, 221, 696, 841, 822, 359, 535, 18, 79, 196,\
    #413, 97, 412, 400, 249, 390, 549, 597, 429, 362, 906, 569, 648, 676, 389]

    install_dict = {}
    #  if a route doesn't have enough soc to reach a roadseg, then it will
    #  not use that roadseg even if a unit is installed
    #  thus units not used can be considered as no install roadsegs in order
    #  to get an initial feasible solution
    install_used = {}
    for i in range(1,915):
        install_dict[str(i)] = False
        install_used[str(i)] = False
    for i in init:
        install_dict[str(i-2)] = True
    return install_dict, install_used

def get_install_length(install_dict_used):
    """given install_dict_used, compute length of roadSegs """
    graph = create_data.get_manhattan_neigh_graph()
    length = 0
    for road in install_dict_used:
        if install_dict_used[road]:
            #node = str(int(road) - 2)
            length += float(graph.node[road]['length'])
    return length

def initialize_install_dict(instance, install_dict):
    """ initialize roadInstall variable """
    for node in install_dict:
        if install_dict[node]:
            road = int(node) + 2  # model indexing
            instance.roadInstall[road] = 1
    return instance


def debug(instance):
    """ set initial solution as upper and lower bounds"""
    instance.debug = ConstraintList()
    for varname in instance.component_objects(Var):  # Var is pyomo imported
        varobject = getattr(instance, str(varname))  # instance.str(v)
        for index in varobject:
            #varobject[index] = 0  # instance.v[index] = 0
            instance.debug.add( varobject[index] >= varobject[index].value )
            instance.debug.add( varobject[index] <= varobject[index].value )
    return instance

def get_budget_from_dat(mydatafile):
    myfile = open(mydatafile)
    for line in myfile:
        if line.startswith('param budget := '):
            mybudget =  re.findall(r'\d+', line)
            budget = float(mybudget[0])
            assert len(mybudget) == 1
            myfile.close()
            return budget


def binary_search_instance(seq, t):
    min = 0
    max = len(seq) - 1
    while True:
        if max < min:
            return -1
        m = (min + max) // 2
        if seq[m] < t:
            min = m + 1
        elif seq[m] > t:
            max = m - 1
        else:
            return m


def get_best_instance(budget_sequence,
                      budget_dat,
                      instance,
                      Roadcategory,
                      Routes,
                      initial_soc,
                      nLayers):
    """ use binary search to get best instance 
    best instance is instance i such that i is a initial feasible solution
    while i + 1 is not

    NOTE: This is really not the best way to do it

    We first implement seqeuntial not binary search
    """
    print '\t deepcopy...'
    empty_instance = copy.deepcopy(instance)
    current_instance = copy.deepcopy(instance)
    print '\t deepcopy...DONE'
    cond = True
    i = 0
    print '\t while loop ...'
    while cond:
        budget_percentage = budget_sequence[i]
        prev_instance = copy.deepcopy(current_instance)
        current_instance, install_dict_used, cond = initialize_with_centrality(
                                                  empty_instance,
                                                  Roadcategory,
                                                  Routes,
                                                  initial_soc,
                                                  budget_percentage,
                                                  nLayers,
                                                  budget_dat)
        print budget_percentage
        i += 1
    print 'best budget_percentage', budget_percentage
    return prev_instance

def initialize_with_centrality(instance,
                               Roadcategory,
                               Routes,
                               initial_soc,
                               budget_percentage,
                               nLayers,
                               budget_dat):
    """get instance with given budget for centrality """
    install_dict = get_centrality_solution(Routes, budget_percentage)
    install_used = dict.fromkeys(install_dict.keys(), False)

    # intialize to zero
    instance = initialize2zero(instance)
    
    for route_no in Routes:
        instance, install_used = initial_variable_based_on_route(instance,
                                                                 route_no,
                                                                 install_dict,
                                                                 nLayers,
                                                                 Roadcategory,
                                                                 Routes, 
                                                                 initial_soc,
                                                                 install_used)

    
    # initialize install_dic 
    install_dict_used = dict.fromkeys(install_dict.keys(), False)
    for node in install_dict:
        if install_dict[node] and install_used[node]:
            install_dict_used[node] = True

    instance = initialize_install_dict(instance, install_dict_used)

    #instance = debug(instance)
    # compute initialize budget
    length = get_install_length(install_dict_used)
    print "warmstart length:", length*0.236, budget_dat
    return instance, install_dict_used, length*0.236 <= budget_dat






def single_experiment():
    """ compares initial solution with betweenness Vs CPLEX for single .dat"""

    # load the data
    #instance = create_model.model.create_instance('mydata_velocity0_1.dat')
    #mydata = 'mydata_btn_cen0.dat'
    mydatafile = sys.argv[-1]
    print mydatafile
    print 'creating model...'
    instance = create_model.model.create_instance(mydatafile)
    
    print 'creating model...DONE'
    timelimit = 100
    print 'cold start....'
    # get cold start initial solution
    solver = SolverFactory("cplex")
    #solver.options['timelimit'] = timelimit*3
    solver.options['parallel'] = -1
    solver.options['mip limits solutions'] = 1
    results_cplex = solver.solve(instance, warmstart=False, tee=True)
    init_cold = instance.soc() + 0.00001
    print 'init_cold:', init_cold

    # get cold start solution after timelimit
    solver = SolverFactory("cplex")
    solver.options['timelimit'] = timelimit
    solver.options['parallel'] = -1
    solver.options['conflict display'] = 2
    results_cplex = solver.solve(instance, warmstart=True, tee=True)
    final_cold = instance.soc() + 0.00001
    print 'final_cold', final_cold
    del instance
    print 'del instance...DONE'
    print 'creating model2...'
    instance = create_model.model.create_instance(mydatafile)
    print 'creating model2...DONE'
    #print 'deepcopy'
    #empty_instance = copy.deepcopy(instance)
    #print 'deepcopy..DONE'
    nLayers = 11

    # give initial solution
    #instance = initialize2zero(instance)

    # initialize edges in model
    print 'preprocessing...'
    Roadcategory, Routes, initial_soc = preprocess()
    print 'preprocessing...DONE'
    # get install dictionary
    #install_dict, install_used = get_install_dict()
    budget_dat = get_budget_from_dat(mydatafile)
    budget_sequence = [i/100.0 for i in range(10, 21)]
    print 'getting best instance ...'

    # for small number of routes
    '''
    instance = get_best_instance(budget_sequence,
                                  budget_dat,
                                  instance,
                                  Roadcategory,
                                  Routes,
                                  initial_soc,
                                  nLayers)
    '''
    
    instance, install_dict_used, cond = initialize_with_centrality(
                                        instance,
                                        Roadcategory,
                                        Routes,
                                        initial_soc,
                                        budget_sequence[0],
                                        nLayers,
                                        budget_dat)
    print 'getting best instance ...DONE'
    # create a solver and pass options\

    solver = SolverFactory("cplex")
    solver.options['mip limits solutions'] = 1 # stop after first feasible
    solver.options['mipgap'] = 0.01
    solver.options['parallel'] = -1
    solver.options['conflict display'] = 2
    #solver.options['randomseed'] = random.randint(1,1000)

    # solve the first time (tee=True prints the cplex output)
    #results = solver.solve(instance, tee=True)
    #assert str(results.solver.termination_condition) == 'optimal'

    # get warmstart solution
    results = solver.solve(instance, warmstart=True, tee=True)
    init_warm = instance.soc()
    print 'warmstart:', init_warm

    # get warmstart solution after timelimit
    solver = SolverFactory("cplex")
    solver.options['timelimit'] = timelimit
    solver.options['parallel'] = -1
    solver.options['conflict display'] = 2
    results = solver.solve(instance, warmstart=True, tee=True)
    final_warm = instance.soc()
    print 'final warmstart:', final_warm

    '''
    # get cold start initial solution
    instance = initialize2zero(instance)  # debug: clear previous solution
    solver = SolverFactory("cplex")
    solver.options['timelimit'] = timelimit*3
    solver.options['parallel'] = -1
    solver.options['mip limits solutions'] = 1
    results_cplex = solver.solve(instance, warmstart=False, tee=True)
    init_cold = instance.soc() + 0.00001
    print 'init_cold:', init_cold
   
    # get cold start solution after timelimit
    solver = SolverFactory("cplex")
    solver.options['timelimit'] = timelimit
    solver.options['parallel'] = -1
    solver.options['conflict display'] = 2
    results_cplex = solver.solve(instance, warmstart=True, tee=False)
    final_cold = instance.soc() + 0.00001
    print 'final_cold', final_cold
    '''
    print 'init improvement:', init_warm/init_cold
    print 'final improvement:', final_warm/final_cold
    
    improvementfile = mydatafile.replace('.dat', '_improvmnt.txt')

    myFile = open(improvementfile, 'w')
    out = '% init_improve \t final_improve \n'
    out += str(init_warm/init_cold) + '\t' + str(final_warm/final_cold)
    myFile.write(out)
    myFile.close()
    #instance.display()
    # display variabes with results

    '''
    graph = create_data.get_manhattan_neigh_graph()
    length = 0
    varobject = getattr(instance, 'roadInstall')

    for index in sorted(varobject):
        if varobject[index].value > 0:
           length += float(graph.node[str(index - 2)]['length'])*0.236
    print 'cplexlenght:', length
    '''
    
def multiple_experiment():
    """ for mulitple .dat files run experiment comparing
    betweenness and CPLEX initial solution"""
    pass


if __name__ == '__main__':
    single_experiment()
 
    
  
    
