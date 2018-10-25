"""
Input: Routes as text file
Ouput: Graph of routes

Turn Routes to networkx graph an plot
"""
import networkx as nx
import matplotlib.pyplot as plt
import pygraphviz
from networkx.drawing.nx_agraph import graphviz_layout


def routes_to_graph():
    routefile = 'routes.txt'
    myfile = open(routefile, 'r')
    graph = nx.Graph()
    start_nodes = set()
    target_nodes = set()
    for line in myfile:
        route = line.split()
        route = [int(i) for i in route[:-1]]
        start_nodes.add(route[0])
        target_nodes.add(route[-1])
        for i in range(len(route) - 1):
            u = route[i]
            v = route[i + 1]
            graph.add_edge(u,v)
    print('%i connected components' %nx.number_connected_components(graph))
    print('%i start nodes' %len(start_nodes))
    print sorted(list(start_nodes))
    print sorted(list(target_nodes))
    return graph, start_nodes

def plot_graph(graph, start_nodes):
    pos = graphviz_layout(graph, prog='sfdp')
    node_colors = ['red' if node in start_nodes else 'blue' for node in graph.nodes()]
    node_sizes = [60 if node in start_nodes else 10 for node in graph.nodes()]
    nx.draw(graph,pos, node_size = node_sizes, node_color = node_colors, arrows=False)
    plt.show()


if __name__ == '__main__':
    graph, start_nodes = routes_to_graph()
    plot_graph(graph, start_nodes)
