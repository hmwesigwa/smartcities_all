#!/usr/bin/python
from __future__ import division
import sys, os
import argparse
import math
import networkx as nx
import random
import numpy as np
import matplotlib.pyplot as plt
from scipy import sparse
import scipy
import collections
import random
    

  
#update data and data_var for a given route
def path_to_layeredGraph(path, pathNumber, data, data_var, nLayers):
  
  #soure arcs
  next_node = roadSeg_to_node(path[0], nLayers)
  #Assume we start when fully charged
  #if not fully charged, we need to change next_node i.e (next_node + i)
  #data['arcs'] = "".join([data['arcs'], " (", str(pathNumber), ",source,", str(next_node), ")\n" ]) 
  #data['weight'] = "".join([data['weight'], " ", str(pathNumber), " source ", str(next_node), " 0\n" ]) 
  for i in range(nLayers):
    data_var['nodes'].add(next_node + i)
    data['arcs'] = "".join([data['arcs'], " (", str(pathNumber), ",source,", str(next_node + i), ")\n" ]) 
    data['weight'] = "".join([data['weight'], " ", str(pathNumber), " source ", str(next_node + i), " 0\n" ]) 
  
  #destination arcs
  node = roadSeg_to_node(path[-1], nLayers)
  data_var['roadSegs'].add(path[-1])
  for i in range(nLayers):
    data_var['nodes'].add(node + i)
    data['arcs'] = "".join([data['arcs'], " (", str(pathNumber), ",", str(node + i), ", destination) \n" ]) 
    data['weight'] = "".join([data['weight'], " ", str(pathNumber), " ", str(node + i), " destination 0 \n" ]) 
    
 
  for i, roadSeg in enumerate(path[:-1]):
    data_var['roadSegs'].add(roadSeg)
    next_roadSeg = path[i+1]
    start_node = roadSeg_to_node(roadSeg, nLayers)
    next_start_node = roadSeg_to_node(next_roadSeg, nLayers)
    for j in range(nLayers):
      current_node = start_node + j
      data_var['nodes'].add(current_node)
      current_soc = node_to_soc(current_node, nLayers)
      if current_soc<1.0e-10:
        break
      #INSTALL = True
      INSTALL = True
      next_soc = nextSOC(current_soc, INSTALL, nLayers)
      next_node_posn = position_in_layer(next_soc, nLayers)
      next_node = next_start_node + next_node_posn
      data['arcs'] = "".join([data['arcs'], " (",str(pathNumber), ",", str(current_node), ",", str(next_node), ") \n" ])
      data['weight'] = "".join([data['weight'], " ",str(pathNumber), " ", str(current_node), " ", str(next_node), " 1\n" ])

      #INSTALL = False
      INSTALL = False
      next_soc = nextSOC(current_soc, INSTALL, nLayers)
      next_node_posn = position_in_layer(next_soc, nLayers)
      next_node = next_start_node + next_node_posn
      data['arcs'] = "".join([data['arcs'], " (",str(pathNumber), ",", str(current_node), ",", str(next_node), ")\n" ])
      data['weight'] = "".join([data['weight'], " ",str(pathNumber), " ", str(current_node), " ", str(next_node), " 0\n" ])
      if roadSeg == path[0]:
        break

  return data, data_var
    
#determine soc for a given node in layered graph
def node_to_soc(node, nLayers):
  posn = (node-1)%nLayers 
  soc =  1-  posn/(nLayers-1)
  
  return soc
  
  
#determine next soc if WCU is installed  
def nextSOC(currentSOC, INSTALL, nLayers):
  if currentSOC == 0:
    return currentSOC

  if INSTALL:
    return min(1, currentSOC + 1.0/(nLayers-1))
  else:
    return max(0, currentSOC - 1.0/(nLayers-1))


def prevSOC(currentSOC, nLayers):
  
  prev_soc = set()
  for i in range(1,nLayers):
    p_soc = i/(nLayers-1)
    soc = nextSOC(p_soc, True, nLayers)

    if soc <= currentSOC + 1.0e-10 and soc >= currentSOC - 1.0e-10:
      prev_soc.add(p_soc)
    
    soc = nextSOC(p_soc, False, nLayers)
    if soc <= currentSOC + 1.0e-10 and soc >= currentSOC - 1.0e-10:
      prev_soc.add(p_soc)
  
  return prev_soc
#determine ith position from the top of layered graph in a given s
#zero based index
def position_in_layer(soc, nLayers):
  posn = (nLayers-1)*(1 - soc) 
  posn = round(posn)
  return int(posn)
  
#determine top most node of layered graph for a given roadSeg
def roadSeg_to_node(roadSeg, nLayers):
  
  node = (roadSeg-1)*nLayers + 1
  
  return node

#determine corresponding roadSeg for a given node
def node_to_roadSeg(node, nLayers):

  if node == 'source' or node == 'destination':
    return node
  else:
    roadSeg = math.ceil(node/nLayers)
  return int(roadSeg)
 
 
 
#initialize data and data_var
def init_dataFile(nLayers, budget, nRoutes, largeNumber):
  
  data = {}
  data_var = {}
  data['nodes'] =       'set nodes := source destination'
  data['arcs'] =        'set arcs :=\n'
  data['roadSegs'] =    'set roadSegs :='
  data['route_no'] =      'set route_no :='
  data['nRoutes'] =     'param nRoutes :='
  data['nLayers'] =     'param nLayers :='
  data['largeNumber'] = 'param largeNumber :=' 
  data['nRoadSegs'] =   'param nRoadSegs :='
  data['budget'] =      'param budget :='
  data['costInstall'] = 'param costInstall :=\n'
  data['weight'] =      'param weight :=\n'
  data_var['nodes'] = set()
  data_var['roadSegs'] = set()
  data_var['nRoutes'] = nRoutes
  data_var['nLayers'] =  nLayers
  data_var['largeNumber'] = largeNumber
  data_var['budget'] =   budget
  
  return data, data_var
 
def write_data_to_file(filename, data, data_var):

  myFile = open(filename, 'w')
  #nodes
  for i in data_var['nodes']:
    data['nodes'] += " "
    data['nodes'] += str(i)
  data['nodes'] += ';\n'
  
  #arcs
  if data['arcs'][-1] == '\n':
    data['arcs'] = data['arcs'][:-1] +  ';\n'
  else:
    data['arcs'] += ';\n'
    
  #roadSegs
  for roadSeg in data_var['roadSegs']:
    data['roadSegs'] += " "
    data['roadSegs'] += str(roadSeg)
  data['roadSegs'] += ';\n'
  
  #routes
  for i in range(1, data_var['nRoutes'] +1):
    data['route_no'] += " "
    data['route_no'] += str(i)
  data['route_no'] += ';\n'
  
  #nRoutes
  data['nRoutes'] += str(data_var['nRoutes'])
  data['nRoutes'] += ';\n'
  
  #nLayers
  data['nLayers'] += ' '
  data['nLayers'] += str(data_var['nLayers'])
  data['nLayers'] += ';\n'
  
  #largeNumber
  data['largeNumber'] += ' '
  data['largeNumber'] += str(data_var['largeNumber'])
  data['largeNumber'] += ';\n'
  
  #nRoadSegs
  data['nRoadSegs'] += ' '
  data['nRoadSegs'] += str(len(data_var['roadSegs']))
  data['nRoadSegs'] += ';\n'
  
  #budget
  data['budget'] += ' '
  data['budget'] +=  str(data_var['budget'])
  data['budget'] += ';\n'
  
  #costInstall
  for roadSeg in data_var['roadSegs']:
    data['costInstall'] += str(roadSeg) + " 1\n"
  if data['costInstall'][-1] == '\n':
    data['costInstall'] = data['costInstall'][:-1] +  ';\n'
  else:
    data['costInstall'] += ';\n'
  
  #weight
  if data['weight'][-1] == '\n':
    data['weight'] = data['weight'][:-1] +  ';\n'
  else:
    data['weight'] += ';\n'
  

  for val in data:
    myFile.write(data[val])

  '''
  myFile.write(data['nodes'])
  myFile.write(data['arcs'])
  myFile.write(data['roadSegs'])
  myFile.write(data['route_no'])
  myFile.write(data['nRoutes']) 
  myFile.write(data['nLayers'])
  myFile.write(data['largeNumber']) 
  myFile.write(data['nRoadSegs'])
  myFile.write(data['budget'])
  myFile.write(data['costInstall'])
  myFile.write(data['weight'])
  '''
  myFile.close()
  
def write_route_to_file(path,write):
  myFile = open('routes.txt', write)
  
  out = [str(i) for i in path]
  out = " ".join(out)
  out += "\n"
  myFile.write(out)
  myFile.close()
 
def row(num, n):
  return int(math.ceil(num/n))

def col(num, n):
  return num - (row(num,n) -1)*n
  
def random_routes(graph):
  nodes = graph.nodes()
  nnodes = nx.number_of_nodes(graph)
  all_route_no = range(1,nnodes*nnodes + 1) 
  random.shuffle(all_route_no)
  out = []
  
  for num in all_route_no:
    r = row(num, nnodes)
    c = col(num, nnodes)
    
    out.append((nodes[r-1],nodes[c-1]))
  
  #out = out[:num_routes]
  return out
  
  
def remove_subroute(path, non_subroute, nnodes, nodes_list, node_indx):
  #node_indx['1'] = 0 node 1 is at inx 0
  
  if len(path) ==2:
    start = path[0]
    end = path[-1]
    c = node_indx[start] + 1
    r = node_indx[end] + 1
    route_no = (r-1)*nnodes + c 
    non_subroute[route_no] = False

  else:
    for i, start in enumerate(path[:-1]):
      for end in path[i :]:
        if (start, end) != (path[0], path[-1]):
          c = node_indx[start] + 1
          r = node_indx[end] + 1
          route_no = (r-1)*nnodes + c 
          non_subroute[route_no] = False
          #print 'subpath',(start, end), route_no
        
  return non_subroute
#@profile
def nextSOC_real_data(initial_soc, distance, install_factor, velocity, INSTALL):
  #d=distance #length of the road segment, mile
  #v=velocity #speed of the road segment, mph

  #k=round(np.mean([27*3, 28*3, 29*4, 30*7, 32*10, 33*4, 34*5,35*3, 36*5, 37, 38*6, 40*3, 47]))/100 #mileage of EV, KWh/mile
  k = 1.41
  #SOC_i=initial_soc #Initial SOC
  #residue=-0.0 #Rounding error of SOC
  #range_val=round(np.mean([81,81,81,238,82,82,114,83,83,87,84,84,62,107,84,84,59,68,68,68,68,93\
#,93,93,76,76,218,259,294,240,240,270,270,270,210,249,315,208,270,238,200,253,253,253,257,250\
#,265,265,235,265,265,87,87,87,187])) #range of EV in miles
  #range_val = 164.34545454545454 #from above
  #E_abs=round(k*range_val) #Total Energy of Battery Pack, KWh
  E_abs = 232
  #WCU_flag=INSTALL #WCU Flag, 0 for no WCU, 1 for WCU
  
  #P_WCU_abs = np.mean([20,60,3,4.2,7.7,2,3,6,17,62,100,3.3]) #KW rating of WCU
  P_WCU_abs = 24.016666666666666
  
  #n=np.mean([0.9,0.85,0.92,0.93,0.91,0.8,0.72,0.71,0.74,0.75,0.9]) #Efficiency factor due to misalignment of coils
  n=0.83
  #d_factor=install_factor #Factor of the road length which contains WCU

  #E_i= E_abs*(initial_soc+residue) #Initial Energy of Battery Pack at time t, KWh
  E_i= E_abs*initial_soc #Initial Energy of Battery Pack at time t, KWh

  time =distance/velocity #time taken to pass the road segment, h
  P_cons=k*velocity #Power consumed in the process, KW
  E_cons=P_cons*time #Energy consumed in the process, KWh
  E_rem=E_i-E_cons #Energy remaining after passing the road, KWh

  if not INSTALL:
      #SOC_f=max(min(round(1000*E_rem/E_abs)/1000,1),0) #Final SOC after passing the road without WCU
      SOC_f= max(0, E_rem/E_abs)
      #residue=(E_rem/E_abs)-(round(100*E_rem/E_abs)/100) #Final residue after passing the road without WCU
  else:
      '''
      d_WCU=d*d_factor #Length of WCU, mile
      t_WCU=d_WCU/v #Time taken to pass over the WCU, h
      P_WCU_act=P_WCU_abs*n #Actual Power due to efficiency factor, KW
      E_WCU=P_WCU_act*t_WCU #Energy provided for EV, KWh
      E_rem2=E_rem+E_WCU #Energy of EV after passing the road with WCU, KWh
      SOC_f= min(1,E_rem2/E_abs)
      '''

      P_WCU_act = P_WCU_abs*n #Actual Power due to efficiency factor, KW
      E_WCU = P_WCU_act*time #Energy provided for EV, KWh
      E_rem2 = E_rem+E_WCU #Energy of EV after passing the road with WCU, KWh
      SOC_f = min(1,E_rem2/E_abs)
      
      #SOC_f=max(min(round(1000*E_rem2/E_abs)/1000,1),0) #Final SOC after passing the road with WCU
      #residue=(E_rem2/E_abs)-(round(100*E_rem2/E_abs)/100) #Final residue after passing the road with WCU    
  
  #return round(SOC_f,3)
  return SOC_f
  
      
if __name__ == "__main__":
  parser = argparse.ArgumentParser(description="write data file for optimization model")
  parser.add_argument("--graph", metavar="myFile", default=" ", type=str, help="graphml file")
  parser.add_argument("--budget", type=int, default=0, help='budget')
  parser.add_argument("--nsamples", type=int, default=0,help='number of random samples')
  args = parser.parse_args()
  
  budget = getattr(args, 'budget')
  graphFile = getattr(args, 'myFile')
  nsamples = getattr(args, 'nsamples')
  roadSegGraph = nx.read_graphml(graphFile)
  #roadSegGraph = nx.read_graphml("roadSegGraph_968.graphml")
  #roadSegGraph = nx.read_graphml("roadSegGraph_110.graphml")
  #roadSegGraph = nx.read_graphml("roadSegGraph.graphml")
  nLayers = 4

  largeNumber = 10**5
  filename = 'example_all_routes.dat'

  #number of routes
  num_routes = nsamples
  pathNumber = 1
  data, data_var = init_dataFile(nLayers, budget, num_routes, largeNumber)
  
  rand_routes = random_routes(roadSegGraph)
  
  non_subroute = {}
  nnodes = nx.number_of_nodes(roadSegGraph)
  node_list = roadSegGraph.nodes()
  node_indx = {}
  count = 0
  for i in node_list:
    node_indx[i] = count
    count += 1

  for u in range(1,nnodes*nnodes + 1):
    non_subroute[u] = True

  for i, j in rand_routes:
    if i !=j :
      shortest_path = nx.shortest_path(roadSegGraph, source=i, target=j, weight=None)
      #non_subroute = remove_subroute(shortest_path, non_subroute, nnodes, node_list, node_indx)
      if 0 < -3:
      #if len(shortest_path) < -3:
        path  = [int(v) + 2 for v in shortest_path] #add 2 to make node index start at 2
        path.append(1) #1 represents the destination layered roadseg
        if pathNumber > 1:
          write = 'a'
        else:
          write = 'w'
        write_route_to_file(path, write)
        data, data_var = path_to_layeredGraph(path, pathNumber, data, data_var, nLayers)

        pathNumber +=1
        if pathNumber > num_routes:
          break
  #print pathNumber -1, "routes"
  if pathNumber < num_routes:
    print("ERROR: less routes than desired")
    os.system("rm example_all_routes.dat")
  else:
    write_data_to_file(filename, data, data_var) 
      
  #print 'number of roadSegs:', len(data_var['roadSegs']) -1
  count = 0
  for i in non_subroute:
    if non_subroute[i]:
      count += 1
      start = node_list[col(int(i), nnodes)-1]
      stop = node_list[row(int(i), nnodes)-1]
      shortest_path = nx.shortest_path(roadSegGraph, source=start, target=stop, weight=None)
      #print i, len(shortest_path)
  #print count
 
