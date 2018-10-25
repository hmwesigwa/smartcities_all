import networkx as nx

def total_distance(graph):
    dist = 0
    for node in graph.nodes():
        dist += graph.node[node]['length']
    return dist



if __name__ == '__main__':
    graph = nx.read_graphml("manhattan_neigborhood.graphml")
    dist = total_distance(graph)
    max_budget = dist * 2.35
    onepercent = 0.01 * max_budget
    print('total distance', dist)
    print('One percent budget', onepercent)
