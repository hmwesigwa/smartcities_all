from pyomo.environ import *
#import create_model
import create_data
import sys
import networkx as nx
import random


def build_model_and_solve():
    import create_model
    reload(create_model)
    # load the data
    #instance = create_model.model.create_instance('mydata_velocity0_1.dat')
    #mydata = 'mydata_btn_cen0.dat'
    mydatafile = 'temp_files/mydata.dat'
    print 'creating model...'
    instance = create_model.model.create_instance(mydatafile)
    print 'creating model...DONE'
    timelimit = 10000
    print 'cold start....'
    # get cold start initial solution
    solver = SolverFactory("cplex")
    #solver.options['timelimit'] = timelimit*3
    solver.options['parallel'] = -1
    #solver.options['mip limits solutions'] = 1
    results_cplex = solver.solve(instance, warmstart=False, tee=True)
    init_cold = instance.soc() + 0.00001
    print 'init_cold:', init_cold
    
    varobject = getattr(instance, 'roadInstall')
    install_nodes_set = set()
    for index in sorted(varobject):
        x_val = varobject[index].value 
        if x_val > 0.001:
            install_nodes_set.add(str(index - 2))
    return install_nodes_set


def get_base_routes():
    myfile = open('temp_files/base_routes.txt', 'r')
    Routes = {}
    for line in myfile:
        path = line.split()
        start = path[0]
        end = path[-1]
        Routes[(start, end)] = path
    return Routes


def base_routes_solution():
    # data to file
    print 'heresd'
    create_data.main_shortest_paths()
    # model
    
    install_nodes_set = build_model_and_solve()
    return install_nodes_set


def extra_routes_solution():
    # data to file
    create_data.extra_paths()
    # model
    #reload(create_data)
    install_nodes_set = build_model_and_solve()
    return install_nodes_set


def comp_solutions1(install_base, install_extra):
    myroutes = get_base_routes()
    graph = create_data.roadSegGraph
    for i in myroutes:
        for node in myroutes[i]:
            if node not in install_base:
                print 'error', node
    base_avg_soc = create_data.average_final_soc(graph, myroutes, install_base)
    extra_avg_soc = create_data.average_final_soc(graph, myroutes, install_extra)
    no_install_soc = create_data.no_install_avg_final_soc(graph, myroutes)
    print install_base
    print install_extra
    print base_avg_soc, extra_avg_soc, no_install_soc

def experiment():
    install_base = base_routes_solution()
    install_extra = extra_routes_solution()
    comp_solutions1(install_base, install_extra)
    
if __name__ == '__main__':
    print 'start'
    random.seed(0)
    experiment()




