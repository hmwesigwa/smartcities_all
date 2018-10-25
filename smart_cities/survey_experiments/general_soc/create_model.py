from __future__ import division
from pyomo.environ import *
import math
import create_data
import sys
"""Model for Wireless Charging Lane Placement:
Input: initial_soc.txt  # file of initial soc
       mydata_routes.txt  # file for routes

global variable create_data.road_category[roadSeg]

NOTE: mydata.dat and mydata_routes.txt must be in the format
      xyz.dat and xyz_routes.txt

NOTE: routes in routeFiles have index shifted by 2, final
road seg is always 1
      Roadcategory[node] takes the real indexing, starting from 1
"""
model = AbstractModel()
model.nodes =    Set()
model.route_no = Set()
model.nRoutes = Param(within=NonNegativeIntegers)
model.nLayers = Param(within=NonNegativeIntegers)
model.largeNumber = Param(within=NonNegativeIntegers)
model.budget = Param(within=NonNegativeIntegers)
model.nRoadSegs = Param(within=NonNegativeIntegers)
model.arcs =     Set(within=model.route_no*model.nodes*model.nodes)
model.weight    =     Param(model.arcs, within=NonNegativeReals)
model.roadSegs = Set()
model.roadInstall = Var(model.roadSegs, within=Boolean)
#model.roadInstall = Var(model.roadSegs, within=NonNegativeReals)
model.x    =     Var(model.arcs, within=Boolean)
#model.x    =     Var(model.arcs, within=NonNegativeReals)
model.costInstall = Param(model.roadSegs)
model.boundary_nodes = Set(within=model.route_no*model.nodes)
model.boundary_node_weights = Param(model.boundary_nodes)



model.Routes = {}
#read routes from file
mystr = sys.argv[-1]
routefile = mystr.replace('.dat', '_routes.txt')
roadcatfile = mystr.replace('.dat', '_road_cat.txt')
Roadcategory = {}
myFile = open(roadcatfile, 'r')
for line in myFile:
  line = line.split()
  node, cat = line
  Roadcategory[str(int(node))] = int(cat)
myFile.close()
#myFile = open("routes.txt", "r")
myFile = open(routefile, 'r')
initial_soc_file = open("/home/hushiji/Research/smart_cities/smart_cities/general_soc/analysis/iterative_initial_soc.txt", "r")
#initial_soc_file = open(socfile, 'r')
count = 1
for line in myFile:
  lines_split = line.split()
  route = [int(r) for r in lines_split]
  model.Routes[count] = route
  count += 1
myFile.close()
count = 1
model.initial_soc = {}
for line in initial_soc_file:
  model.initial_soc[count] = float(line)
  count += 1
initial_soc_file.close()


def roadSeg_to_node(model, roadSeg):
  node = (roadSeg-1)*model.nLayers.value + 1
  return node


def node_to_roadSeg(model, node):
  if node == 'source' or node == 'destination':
    return node
  else:
    roadSeg = math.ceil(node/model.nLayers.value)
    #roadSeg = (node - node%(model.nLayers.value))/model.nLayers.value + 1
  return int(roadSeg)


def boundary_nodes_init(model, route_no):
  out = []
  for roadSeg in model.Routes[route_no][1:-1]:
    start_node = create_data.roadSeg_to_node(roadSeg, model.nLayers.value)
    out.append(start_node + model.nLayers.value -1)
  roadSeg = model.Routes[route_no][-1]
  start_node = start_node = create_data.roadSeg_to_node(roadSeg, model.nLayers.value)
  for i in range(model.nLayers.value):
    out.append(start_node +i)
  return out
    
#model.boundary_nodes = Set(model.route_no, initialize=boundary_nodes_init) 


def NodesOut_init(model, node, route_no):
  out = []
  if node == 'source':
    start_node = create_data.roadSeg_to_node(model.Routes[route_no][0], model.nLayers.value)
    for i in range( model.nLayers.value ):
      out.append(start_node + i)
  elif node == 'destination':
    return out
  else:
    roadSeg = create_data.node_to_roadSeg(node, model.nLayers.value)
    if roadSeg not in model.Routes[route_no]:
      return out
    soc = create_data.node_to_soc(node, model.nLayers.value)
    if soc < 1.0e-10 and roadSeg != model.Routes[route_no][0]:
      out.append('destination')
      return out
    if roadSeg == model.Routes[route_no][-1]:
      out.append('destination')
    elif roadSeg == model.Routes[route_no][0]:
      next_roadSeg = model.Routes[route_no][1]
      next_start_node = create_data.roadSeg_to_node(next_roadSeg, model.nLayers.value)
      init_soc = model.initial_soc[route_no]
      current_posn = create_data.position_in_layer(init_soc, model.nLayers.value)
      top_node = create_data.roadSeg_to_node(roadSeg, model.nLayers.value)
      if node == top_node + current_posn:
        #road_cat = create_data.road_category[str(roadSeg -2)]
        road_cat = Roadcategory[str(roadSeg -2)]
        next_install_posn = max(current_posn - road_cat, 0)
        next_no_install_posn = min(current_posn + road_cat, model.nLayers.value-1)
        if road_cat != 0:
          out.append(next_start_node + next_install_posn)
          out.append(next_start_node + next_no_install_posn )
          #if route_no == 128 and node == 23:
          #  print next_start_node, top_node, current_posn, next_start_node + next_install_posn, next_start_node + next_no_install_posn 
        else:
          #out.append(next_start_node + next_no_install_posn + road_cat)
          out.append(next_start_node + next_no_install_posn + road_cat)
    else:    
      roadSeg_indx = model.Routes[route_no].index(roadSeg)     
      next_roadSeg = model.Routes[route_no][roadSeg_indx + 1]
      next_start_node = create_data.roadSeg_to_node(next_roadSeg, model.nLayers.value)
      current_soc = create_data.node_to_soc(node, model.nLayers.value)
      #road_cat = create_data.road_category[str(roadSeg -2)]
      road_cat = Roadcategory[str(roadSeg -2)]
      if current_soc > 1.0e-10 and road_cat != 0:
        #INSTALL = True
        INSTALL = True
        next_soc = create_data.nextSOC(current_soc, INSTALL, model.nLayers.value, road_cat)
        next_node_posn = create_data.position_in_layer(next_soc, model.nLayers.value)
        next_node = next_start_node + next_node_posn
        out.append(next_node)
        #INSTALL = False
        INSTALL = False
        next_soc = create_data.nextSOC(current_soc, INSTALL, model.nLayers.value, road_cat)
        next_node_posn = create_data.position_in_layer(next_soc, model.nLayers.value)
        next_node = next_start_node + next_node_posn
        if next_node != out[-1]:
          out.append(next_node)
      elif current_soc > 1.0e-10 and road_cat == 0:
        #INSTALL = False
        INSTALL = False
        next_soc = create_data.nextSOC(current_soc, INSTALL, model.nLayers.value, road_cat)
        next_node_posn = create_data.position_in_layer(next_soc, model.nLayers.value)
        next_node = next_start_node + next_node_posn
        ########
        #if node == 16402:
        #print node, route_no,road_cat, out
        #if next_node != out[-1]:
        out.append(next_node)
  return out
  
model.NodesOut = Set(model.nodes,model.route_no, initialize=NodesOut_init)


def NodesIn_init(model, node, route_no):
  out = []

  
  
  if node == 'source':
    return out
  
  elif node == 'destination':
    start_node = create_data.roadSeg_to_node(model.Routes[route_no][-1], model.nLayers.value)
    for i in range( model.nLayers.value ):
      out.append(start_node + i)
    for roadSeg in model.Routes[route_no][1:-1]:
      start_node = create_data.roadSeg_to_node(roadSeg, model.nLayers.value)
      out.append(start_node + model.nLayers.value -1)
  
  else:
    roadSeg = create_data.node_to_roadSeg(node, model.nLayers.value)
    if roadSeg not in model.Routes[route_no]:
      return out
    if roadSeg == model.Routes[route_no][0]:
      out.append('source')
    else:
      roadSeg_indx = model.Routes[route_no].index(roadSeg)     
      prev_roadSeg = model.Routes[route_no][roadSeg_indx - 1]
      prev_start_node = create_data.roadSeg_to_node(prev_roadSeg, model.nLayers.value)
      current_soc = create_data.node_to_soc(node, model.nLayers.value)
      #prev_road_cat = create_data.road_category[str(prev_roadSeg -2)]
      prev_road_cat = Roadcategory[str(prev_roadSeg -2)]
      prev_soc = create_data.prevSOC(current_soc, model.nLayers.value, prev_road_cat)
      posn = [create_data.position_in_layer(soc, model.nLayers.value) for soc in prev_soc]
      
      
      if roadSeg == model.Routes[route_no][1]:
        init_soc = model.initial_soc[route_no]
        init_posn = create_data.position_in_layer(init_soc, model.nLayers.value)
        if init_posn in posn:
          out.append(prev_start_node + init_posn)
      else:
        out = [prev_start_node + i for i in posn]
        
      
  return out
  
model.NodesIn = Set(model.nodes,model.route_no, initialize=NodesIn_init)



def flowRuleEfficient(model, node,route):
  #define paths
  #constraint_equation = True
  #for route in range(1, model.nRoutes.value + 1): 
  #source node
  #print node, route
  if node == 'source':
    next_roadSeg = model.Routes[route][0]
    next_roadSegNode = roadSeg_to_node(model, next_roadSeg)
    flow_out = sum( \
      model.x[route, node, r] \
      for r in model.NodesOut[node,route] \
    )  
    constraint_equation = (flow_out == 1)
  
  #destination node
  elif node == 'destination':

    prev_roadSeg = model.Routes[route][-1]
    prev_roadSegNode = roadSeg_to_node(model, prev_roadSeg)
    flow_in = sum( \
      model.x[route, r, node] \
      for r in model.NodesIn[node,route] \
    )

    
    constraint_equation = (flow_in == 1)

  #internal node
  else:
    roadSeg = node_to_roadSeg(model, node)
    #print node, route, roadSeg
    if roadSeg not in model.Routes[route]:
      constraint_equation = Constraint.Feasible
      return constraint_equation
    #source roadSeg
    if roadSeg == model.Routes[route][0]:
      next_roadSeg = model.Routes[route][1]
      next_roadSegNode = roadSeg_to_node(model, next_roadSeg)
      prev_roadSegNode = 'source'
      amount_out = sum(\
        model.x[route, node, r] \
        for r in model.NodesOut[node,route] \
      )

      amount_in = model.x[route, prev_roadSegNode, node]
      constraint_equation = (amount_in - amount_out == 0)
    #destination roadSeg
    elif roadSeg == model.Routes[route][-1]:
      next_roadSegNode = 'destination'
      prev_roadSeg = model.Routes[route][-2]
      prev_roadSegNode = roadSeg_to_node(model, prev_roadSeg)
      amount_in = sum( \
        model.x[route, r, node] \
        for r in model.NodesIn[node,route] \
      )
      amount_out = model.x[route, node,  next_roadSegNode]

      constraint_equation = (amount_in - amount_out == 0)
      
    #internal roadSeg
    else:
      i = model.Routes[route].index(roadSeg)
      next_roadSeg = model.Routes[route][i + 1]
      prev_roadSeg = model.Routes[route][i - 1]
      next_roadSegNode = roadSeg_to_node(model, next_roadSeg)
      prev_roadSegNode = roadSeg_to_node(model, prev_roadSeg)


      amount_out = sum( \
        model.x[route, node, r] \
        for r in model.NodesOut[node,route] \
      ) 

      amount_in = sum( \
        model.x[route, r, node] \
        for r in model.NodesIn[node,route] \
      )

      constraint_equation = (amount_in - amount_out == 0)
      if type(amount_in) == int and type(amount_out) == int:
        constraint_equation = Constraint.Feasible
  return constraint_equation
        

model.flow = Constraint(model.nodes, model.route_no, rule=flowRuleEfficient) 


def number_required_to_install(model, roadSeg):
  number_required = 0
  for route in range(1, model.nRoutes.value + 1):
    for node in range(model.nLayers.value*(roadSeg - 1) +1, roadSeg*model.nLayers.value + 1):
      for u in model.NodesOut[node, route]:
        if u != 'destination':
          number_required += model.weight[route,node, u]*model.x[route, node,u]
  return number_required


def install_upper(model, roadSeg):
  constraint_equation = (model.roadInstall[roadSeg] <= number_required_to_install(model, roadSeg))
  return constraint_equation
model.installU = Constraint(model.roadSegs, rule=install_upper)


def install_lower(model, roadSeg):
  #constraint_equation = (model.largeNumber.value*model.roadInstall[roadSeg] >= number_required_to_install(model, roadSeg) - 0.5)
  # - 0.5 added because of R_k very small real
  constraint_equation = (model.largeNumber.value*model.roadInstall[roadSeg] >= number_required_to_install(model, roadSeg))
  return constraint_equation
model.installL = Constraint(model.roadSegs, rule=install_lower)


# additional constraint, if roadInstall == 1, then all must benefit (not just some)
def number_benefit(model, roadSeg):
  # number of routes to potentially benefit from install
  number_benefit = 0
  for route in range(1, model.nRoutes.value + 1):
    for node in range(model.nLayers.value*(roadSeg - 1) +1, roadSeg*model.nLayers.value + 1):
      for u in model.NodesOut[node, route]:
        if u != 'destination':
          number_benefit += model.x[route, node,u]
  return number_benefit


def number_benefit_upper(model, roadSeg):
  constraint_equation = (number_required_to_install(model, roadSeg) - number_benefit(model, roadSeg) <= model.largeNumber.value*(1 - model.roadInstall[roadSeg]))
  return constraint_equation
model.benefitU = Constraint(model.roadSegs, rule=number_benefit_upper)


def number_benefit_lower(model, roadSeg):
  constraint_equation = (model.largeNumber.value*(model.roadInstall[roadSeg] -1) <= number_required_to_install(model, roadSeg) - number_benefit(model, roadSeg))
  return constraint_equation
model.benefitL = Constraint(model.roadSegs, rule=number_benefit_lower)

#budget
def budget(model):
      
    out = 0
    for r in model.roadSegs:
      out += model.costInstall[r]*model.roadInstall[r]
      
    constraint_equation = (out <= model.budget.value)
    
    return constraint_equation
    
model.budgetLimit = Constraint(rule=budget)
    
    
def totalRule(model):
  output = 0
  for (r,i,j) in model.arcs:
    output += model.weight[r,i,j]*model.x[r,i,j]
  return output

#model.maxFlow = Objective(rule=totalRule, sense=maximize)    

def minBudget(model):
  output = 0
  for roadSeg in model.roadSegs:
    output += model.costInstall[roadSeg]*model.roadInstall[roadSeg]
  return output


#model.minBudget = Objective(rule=minBudget, sense=minimize) 
def maxSOC(model):
  output = 0
  
  for route_no,node in model.boundary_nodes:
    output += model.boundary_node_weights[route_no, node]*model.x[route_no, node, 'destination']
  return output
model.soc = Objective(rule=maxSOC, sense=maximize)     
