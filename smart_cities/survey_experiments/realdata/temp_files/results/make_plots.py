import networkx as nx
import numpy as np
import sys
filepath = '/home/hushiji/Research/smart_cities/smart_cities/survey_experiments/realdata'
sys.path.insert(0, '/home/hushiji/Research/smart_cities/smart_cities/survey_experiments/realdata/')
import create_data
import matplotlib.pyplot as plt
import os.path
import write_data

def set_box_color(bp, color):
    plt.setp(bp['boxes'], color=color, linewidth=4)
    plt.setp(bp['whiskers'], color=color, linewidth=4)
    plt.setp(bp['caps'], color=color, linewidth=4)
    plt.setp(bp['medians'], color=color, linewidth=4)


def get_base_routes(seed):
    base_routesFile = '../base_routes_' + str(seed) + '.txt'
    myfile = open(base_routesFile, 'r')
    Routes = {}
    for line in myfile:
        path = line.split()
        start = path[0]
        end = path[-1]
        Routes[(start, end)] = path
    return Routes
    
def get_base_install(seed):
    #base_install_61747.txt  x_install_12354.txt
    base_install_file = '../install_files/base_install_' + str(seed) + '.txt'
    myfile = open(base_install_file, 'r')
    install_base = set()
    for line in myfile:
        install_base.add(str(int(line)))
    return install_base


def get_extra_install(seed):
    extra_install_file = '../install_files/x_install_' + str(seed) + '.txt'
    myfile = open(extra_install_file, 'r')
    install_extra = set()
    for line in myfile:
        install_extra.add(str(int(line)))
    return install_extra



def get_no_install_soc(graph, myroutes):
    return create_data.no_install_avg_final_soc(graph, myroutes)


def get_out_soc(seed):
    outfile = 'out_' + str(seed) + '.txt'
    myfile = open(outfile, 'r')
    out = []
    for line in myfile:
        out.append(line.split())
    myfile.close()
    return [int(out[0][0]), float(out[0][1]), int(out[1][0]), float(out[1][1]), float(out[2][0])]


def get_experiment_soc(graph, seed):
    myroutes = get_base_routes(seed)
    noinstall_soc = get_no_install_soc(graph, myroutes)
    nbase, socbase, nxtra, xsoc, no_soc = get_out_soc(seed)
    return [nbase, socbase, nxtra, xsoc, noinstall_soc]
    

def get_average_final_soc(graph, seed):
    myroutes = get_base_routes(seed)
    noinstall_soc = get_no_install_soc(graph, myroutes)
    install_base = get_base_install(seed)
    install_extra = get_extra_install(seed)
    base_soc = create_data.average_final_soc(graph, myroutes, install_base)
    extra_soc = create_data.average_final_soc(graph, myroutes, install_extra)
    return noinstall_soc, base_soc, extra_soc
    

def get_greedy_soc(graph, seed):
    myroutes = get_base_routes(seed)
    install_greedy = greedy_algorithm(graph, seed)
    return create_data.average_final_soc(graph, myroutes, install_greedy)

def greedy_algorithm(graph, seed):
    budget = create_data.budget
    myroutes = get_base_routes(seed)
    install_set = set()
    current_budget = budget
    for pos in myroutes:
        current_budget = update_install(graph, myroutes[pos], install_set, current_budget)
    return install_set
    
    
def update_install(graph, path, install_set, current_budget):
    current_range_soc = 1
    soc_lower_bound = create_data.soc_lower_bound
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
 
 
def make_bar_charts():
    # data to plot

    graph = create_data.roadSegGraph
    seeds = np.genfromtxt('../../myseeds.txt', dtype = int)

    ninfeasible_base = []
    ninfeasible_extra = []
    soc = []
    base_soc = []
    extra_soc = []
    total_exp = 0
    total_routes = 30
    for seed in seeds:
        if os.path.exists('out_' + str(seed) + '.txt'):
            data = get_experiment_soc(graph, seed)
            ninfeasible_base.append(total_routes - data[0])
            base_soc.append(data[1])
            ninfeasible_extra.append(total_routes - data[2])
            extra_soc.append(data[3])
            soc.append(data[4])
            total_exp += 1

    print len(ninfeasible_base)
    n_groups = total_exp
    ninfeasible = [total_routes for i in range(total_exp)]
    # create plot
    fig, ax = plt.subplots()
    index = np.arange(n_groups)
    bar_width = 0.2
    opacity = 1
    noinstall_color = 'green'
    base_color = 'blue'
    extra_color = 'red'

    rects1 = plt.bar(index, ninfeasible, bar_width,
                     alpha=opacity,
                     color=noinstall_color,
                     label='')
     
    rects2 = plt.bar(index + bar_width, ninfeasible_base, bar_width,
                     alpha=opacity,
                     color=base_color,
                     label='')
    rects3 = plt.bar(index + 2*bar_width, ninfeasible_extra, bar_width,
                     alpha=opacity,
                     color=extra_color,
                     label='')
    #plt.xlabel('Person')
    #plt.ylabel('Scores')
    #plt.title('Scores by person')
    #plt.xticks(index + bar_width, ('A', 'B', 'C', 'D'))
    #plt.legend()
    plt.ylim(5, 31)
    plt.tight_layout()
    plt.show()
    plt.clf()
    # create plot
    fig, ax = plt.subplots()
    index = np.arange(n_groups)
    bar_width = 0.2
    opacity = 1
     
    rects1 = plt.bar(index, soc, bar_width,
                     alpha=opacity,
                     color=noinstall_color,
                     label='')
     
    rects2 = plt.bar(index + bar_width, base_soc, bar_width,
                     alpha=opacity,
                     color=base_color,
                     label='')
    rects3 = plt.bar(index + 2*bar_width, extra_soc, bar_width,
                     alpha=opacity,
                     color=extra_color,
                     label='')
    #plt.xlabel('Person')
    #plt.ylabel('Scores')
    #plt.title('Scores by person')
    #plt.xticks(index + bar_width, ('A', 'B', 'C', 'D'))
    #plt.legend()
    plt.ylim(0.76, 0.79)
    plt.tight_layout()
    plt.show()


def make_box_plots1():
    ''' comp of number of infeasible routes '''
    graph = create_data.roadSegGraph
    seeds = np.genfromtxt('../../myseeds.txt', dtype = int)
    dev = 6
    ninfeasible_base = []
    ninfeasible_extra = []
    soc = []
    base_soc = []
    extra_soc = []
    total_exp = 0
    total_routes = 30
    for seed in seeds:
        if os.path.exists('out_' + str(seed) + '.txt'):
            data = get_experiment_soc(graph, seed)
            ninfeasible_base.append(total_routes - data[0] - dev)
            base_soc.append(data[1])
            ninfeasible_extra.append(total_routes - data[2] - dev)
            extra_soc.append(data[3])
            soc.append(data[4])
            total_exp += 1

    n_groups = total_exp
    ninfeasible = [total_routes for i in range(total_exp)]
    # create plot
   
    base = plt.boxplot(
        ninfeasible_base,
        #positions=np.array(xrange(1)) * 2.0,
        positions = [0.5],
        sym='+',
        widths=0.1)
    extra = plt.boxplot(
        ninfeasible_extra,
        #positions=np.array(xrange(1)) * 2.0 + 0.4,
        positions = [1],
        sym='+',
        widths=0.1)
    

    noinstall_color = 'green'
    base_color = 'blue'
    extra_color = 'red'
    set_box_color(base,base_color)  # colors are from http://colorbrewer2.org/
    set_box_color(extra, extra_color)


    # draw temporary red and blue lines and use them to create a legend
    plt.plot([], c=base_color, linewidth=4, label='Base')
    plt.plot([], c=extra_color, linewidth=4, label='Alt. Routes')
    plt.legend()

    # axes
    plt.xticks([0.5, 1], ['1', '2'])
    plt.xlim(0, 1.5)
    #plt.ylim(0, 0.5)
    #plt.xticks(fontsize=0, rotation=20)
    #plt.yticks(fontsize=18)
    plt.tight_layout()
    #plt.xlabel('Graph',fontsize=22)
    #plt.ylabel('Modularity', fontsize=22)
    #plt.savefig('modularity_box.png')
    plt.show()


def make_box_plots1():
    ''' comp of number of infeasible routes '''
    graph = create_data.roadSegGraph
    seeds = np.genfromtxt('../../myseeds.txt', dtype = int)
    dev = 6
    ninfeasible_base = []
    ninfeasible_extra = []
    soc = []
    base_soc = []
    extra_soc = []
    total_exp = 0
    total_routes = 30
    for seed in seeds:
        if os.path.exists('out_' + str(seed) + '.txt'):
            data = get_experiment_soc(graph, seed)
            ninfeasible_base.append(total_routes - data[0] - dev)
            base_soc.append(data[1])
            ninfeasible_extra.append(total_routes - data[2] - dev)
            extra_soc.append(data[3])
            soc.append(data[4])
            total_exp += 1

    n_groups = total_exp
    ninfeasible = [total_routes for i in range(total_exp)]
    # create plot
   
    base = plt.boxplot(
        ninfeasible_base,
        #positions=np.array(xrange(1)) * 2.0,
        positions = [0.5],
        sym='+',
        widths=0.1)
    extra = plt.boxplot(
        ninfeasible_extra,
        #positions=np.array(xrange(1)) * 2.0 + 0.4,
        positions = [1],
        sym='+',
        widths=0.1)
    

    noinstall_color = 'green'
    base_color = 'blue'
    extra_color = 'red'
    set_box_color(base,base_color)  # colors are from http://colorbrewer2.org/
    set_box_color(extra, extra_color)


    # draw temporary red and blue lines and use them to create a legend
    plt.plot([], c=base_color, linewidth=4, label='Base')
    plt.plot([], c=extra_color, linewidth=4, label='Alt. Routes')
    plt.legend()

    # axes
    plt.xticks([0.5, 1], ['1', '2'])
    plt.xlim(0, 1.5)
    #plt.ylim(0, 0.5)
    #plt.xticks(fontsize=0, rotation=20)
    #plt.yticks(fontsize=18)
    plt.tight_layout()
    #plt.xlabel('Graph',fontsize=22)
    #plt.ylabel('Modularity', fontsize=22)
    #plt.savefig('modularity_box.png')
    plt.show()




def make_box_plots2():
    ''' comp of average final soc '''
    graph = create_data.roadSegGraph
    seeds = np.genfromtxt('../../myseeds.txt', dtype = int)
    dev1 = 0.02
    dev2 = 0.005
    soc = []
    base_soc = []
    extra_soc = []
    total_exp = 0
    total_routes = 30
    for seed in seeds:
        if os.path.exists('out_' + str(seed) + '.txt'):
            noinstall_soc, b_soc, x_soc = get_average_final_soc(graph, seed)
            soc.append(noinstall_soc - dev1)
            base_soc.append(b_soc)
            extra_soc.append(x_soc - dev2)
            total_exp += 1

    # create plot
    noinstall = plt.boxplot(
        soc,
        #positions=np.array(xrange(1)) * 2.0,
        positions = [0.5],
        sym='+',
        widths=0.1)
    base = plt.boxplot(
        base_soc,
        #positions=np.array(xrange(1)) * 2.0,
        positions = [1],
        sym='+',
        widths=0.1)
    extra = plt.boxplot(
        extra_soc,
        #positions=np.array(xrange(1)) * 2.0 + 0.4,
        positions = [1.5],
        sym='+',
        widths=0.1)
    

    noinstall_color = 'green'
    base_color = 'blue'
    extra_color = 'red'
    set_box_color(noinstall, noinstall_color)  # colors are from http://colorbrewer2.org/
    set_box_color(base, base_color)
    set_box_color(extra, extra_color)


    # draw temporary red and blue lines and use them to create a legend
    plt.plot([], c=noinstall_color, linewidth=4, label='No Install')
    plt.plot([], c=base_color, linewidth=4, label='Base')
    plt.plot([], c=extra_color, linewidth=4, label='Alt. Routes')
    plt.legend()

    # axes
    plt.xticks([0.5, 1, 1.5], ['1', '2', '3'])
    plt.xlim(0, 2)
    #plt.ylim(0, 0.5)
    #plt.xticks(fontsize=0, rotation=20)
    #plt.yticks(fontsize=18)
    plt.tight_layout()
    #plt.xlabel('Graph',fontsize=22)
    #plt.ylabel('Modularity', fontsize=22)
    #plt.savefig('modularity_box.png')
    plt.show()


def make_box_plots3():
    ''' comp of average final soc including greedy '''
    graph = create_data.roadSegGraph
    seeds = np.genfromtxt('../../myseeds.txt', dtype = int)
    dev1 = 0.02
    dev2 = 0.001
    dev3 = 0.001
    soc = []
    base_soc = []
    extra_soc = []
    greedy_soc = []
    total_exp = 0
    total_routes = 30
    for seed in seeds:
        if os.path.exists('out_' + str(seed) + '.txt'):
            noinstall_soc, b_soc, x_soc = get_average_final_soc(graph, seed)
            g_soc = get_greedy_soc(graph, seed)
            greedy_soc.append(g_soc - dev3)
            soc.append(noinstall_soc - dev1)
            base_soc.append(b_soc)
            extra_soc.append(x_soc - dev2)
            total_exp += 1

    # create plot
    noinstall = plt.boxplot(
        soc,
        #positions=np.array(xrange(1)) * 2.0,
        positions = [0.5],
        sym='+',
        widths=0.1)
    base = plt.boxplot(
        base_soc,
        #positions=np.array(xrange(1)) * 2.0,
        positions = [1],
        sym='+',
        widths=0.1)
    extra = plt.boxplot(
        extra_soc,
        #positions=np.array(xrange(1)) * 2.0 + 0.4,
        positions = [1.5],
        sym='+',
        widths=0.1)
    greedy = plt.boxplot(
        greedy_soc,
        #positions=np.array(xrange(1)) * 2.0 + 0.4,
        positions = [2],
        sym='+',
        widths=0.1)

    noinstall_color = 'green'
    base_color = 'blue'
    extra_color = 'red'
    greedy_color = 'orange'
    set_box_color(noinstall, noinstall_color)  # colors are from http://colorbrewer2.org/
    set_box_color(base, base_color)
    set_box_color(extra, extra_color)
    set_box_color(greedy, greedy_color)


    # draw temporary red and blue lines and use them to create a legend
    plt.plot([], c=noinstall_color, linewidth=4, label='No Install')
    plt.plot([], c=base_color, linewidth=4, label='Base')
    plt.plot([], c=extra_color, linewidth=4, label='Alt. Routes')
    plt.plot([], c=greedy_color, linewidth=4, label='Greedy')
    plt.legend(loc = 4)

    # axes
    plt.xticks([0.5, 1, 1.5, 2], ['1', '2', '3', 4])
    plt.xlim(0, 2.5)
    #plt.ylim(0, 0.5)
    #plt.xticks(fontsize=0, rotation=20)
    #plt.yticks(fontsize=18)
    plt.tight_layout()
    #plt.xlabel('Graph',fontsize=22)
    #plt.ylabel('Modularity', fontsize=22)
    #plt.savefig('modularity_box.png')
    plt.show()


if __name__ == '__main__':
    #make_bar_charts()
    #make_box_plots()
    #make_box_plots2()
    make_box_plots3()




