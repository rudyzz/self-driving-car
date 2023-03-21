import math
import numpy
from matplotlib import pyplot as plt
import cv2
import utils
import time
import random
import heapq
import copy

class HaltonPlanner(object):
  
  # planningEnv: Should be a HaltonEnvironment
  def __init__(self, planningEnv):
    self.planningEnv = planningEnv

  # Generate a plan
  # Assumes that the source and target were inserted just prior to calling this
  # Returns the generated plan
  def plan(self):
    self.sid = self.planningEnv.graph.number_of_nodes() - 2 # Get source id
    self.tid = self.planningEnv.graph.number_of_nodes() - 1 # Get target id

    self.closed = {} # The closed list
    self.parent = {self.sid:None} # A dictionary mapping children to their parents
    # f(n) = g(n) + h(n)
    # h(n) : self.planningEnv.get_heuristic(self.currid, self.tid)
    # self.open: dictionary id map f(n)
    self.open = {self.sid: 0 + self.planningEnv.get_heuristic(self.sid, self.tid)} # The open list
    # g(n)
    self.gValues = {self.sid:0} # A mapping from node to shortest found path length to that node 
    self.planIndices = []
    self.cost = 0
    # ------------------------------------------------------------
    # YOUR CODE HERE
    # 
    # Implement A*
    # Functions that you will probably use
    # - self.get_solution() -> backtrack parents
    # - self.planningEnv.get_successors()
    # - self.planningEnv.get_distance()
    # - self.planningEnv.get_heuristic()
    # Note that each node in the graph has both an associated id and configuration
    # You should be searching over ids, not configurations. get_successors() will return
    #   the ids of nodes that can be reached. Once you have a path plan
    #   of node ids, get_solution() will compute the actual path in SE(2) based off of
    #   the node ids that you have found.
    #-------------------------------------------------------------


    parents = {self.sid: {self.sid:None}} #a dict of dict contains path history to the every frontier node
    
    while self.open:
      openq = []
      openlist = [(item[1],item[0]) for item in self.open.items()] #heapq sort based on the 1st val of each tuple element
      for h in openlist:
        heapq.heappush(openq, h)
      sel = heapq.heappop(openq) #sel[0]=g+h, sel[1]=id
      if sel[1] == self.tid:
        self.parent = parents[self.tid]
        break
      a = self.open.pop(sel[1])
      self.closed[sel[1]] = sel[0]
      adj = self.planningEnv.get_successors(sel[1]) #adjacent nodes
      for node in adj:
        if node in self.closed:
          continue
        #edge collision check
        parentConfig = self.planningEnv.get_config(sel[1])
        adjConfig = self.planningEnv.get_config(node) #current adjacent node
        if not self.planningEnv.manager.get_edge_validity(parentConfig, adjConfig):
          continue
        g = self.gValues[sel[1]] + self.planningEnv.get_distance(sel[1], node)
        h = self.planningEnv.get_heuristic(node, self.tid)
        f = g + h
        if node in self.gValues:
          if g > self.gValues[node]:
            continue
        self.open[node] = f
        self.gValues[node] = g
        pre_path = copy.deepcopy(parents[sel[1]])
        pre_path[node] = sel[1]
        parents[node] = pre_path

    return self.get_solution(self.tid) #here we return get_solution(selected_indices)




    # while bool (self.open):
    #   minHeap = []
    #   for item in self.open.items():
    #     heapq.heappush(minHeap, (item[1], item[0]))
    #   curr_node = heapq.heappop(minHeap) #curr_node[0]: f(n), curr_node[1]: id

    #   self.open.pop(curr_node[1]) # remove the currentNode from the openList
    #   self.closed[curr_node[1]] = curr_node[0] # add the currentNode to the closedList

    #   # if currentNode is the goal-> backtrack to get path
    #   if (curr_node[1] == self.tid):
    #     path_plan = self.get_solution(curr_node[1])
    #     return path_plan

    #   aja_nodes = self.planningEnv.get_successors(curr_node[1]) # list of ids that can be reached
    #   for child in aja_nodes:
    #     if child in self.closed.keys(): 
    #       continue
    #     currConfig = self.planningEnv.get_config(curr_node[1])
    #     childConfig = self.planningEnv.get_config(child) #current adjacent node
    #     if not self.planningEnv.manager.get_edge_validity(currConfig, childConfig):
    #       continue
    #     child_g = self.gValues[curr_node[1]] + self.planningEnv.get_distance(curr_node[1], child)
    #     child_h = self.planningEnv.get_heuristic(child, self.tid)
    #     child_f = child_g + child_h 
    #     if child in self.open.keys():
    #       if child_g > self.gValues[curr_node[1]]:
    #         continue
    #     self.open[child] = child_f
    #     self.gValues[child] = child_g
    #     self.parent[child] = curr_node[1]

    # return []




    # while len(self.open) != 0:

    #   # Pop the top key
    #   top = min(self.open, key = self.open.get)
    #   self.closed[top] = self.open[top]
    #   del self.open[top]

    #   if top == self.tid:
    #     plan = self.get_solution(self.tid)
    #     return plan

    #   for vid in self.planningEnv.get_successors(top):
    #     if vid in self.closed:
    #       continue

    #     tempG = self.gValues[top] + self.planningEnv.get_distance(top, vid)
    #     if vid in self.open:
    #       if tempG > self.gValues[vid]:
    #         continue

    #     self.open[vid] = tempG + self.planningEnv.get_heuristic(vid, self.tid)
    #     self.gValues[vid] = tempG
    #     self.parent[vid] = top


  # Try to improve the current plan by repeatedly checking if there is a shorter path between random pairs of points in the path
  def post_process(self, plan, timeout):

    t1 = time.time()
    elapsed = 0
    while elapsed < timeout: # Keep going until out of time
      # ---------------------------------------------------------
      # YOUR CODE HERE
      
      # Pseudocode
      
      # Pick random id i
      # Pick random id j
      # Redraw if i == j
      # Switch i and j if i > j
      i, j = random.randint(0, len(plan)-1), random.randint(0, len(plan)-1) # randint(a,b) -> random pick a int from range [a,b]
      while i == j: 
        i = random.randint(0, len(plan)-1)
        j = random.randint(0, len(plan)-1)
      if i > j:
        temp = i
        i = j
        j = temp

      
      # if we can find path between i and j (Hint: look inside ObstacleManager.py for a suitable function)
        # Get the path
        # Reformat the plan such that the new path is inserted and the old section of the path is removed between i and j
        # Be sure to CAREFULLY inspect the data formats of both the original plan and the plan returned
        # to ensure that you edit the path correctly
      # config: The configuration to check (in meters and radians)
      config_i = [plan[i][0], plan[i][1]]
      config_j = [plan[j][0], plan[j][1]]
      if self.planningEnv.manager.get_edge_validity(config_i, config_j):
        new_plan = []
        # add [0, i) plan
        for index in range(0, i):  
          new_plan.append(plan[index])
        # add [i, j) newplan
        new_plan = list(new_plan)
        px, py, clen = self.planningEnv.manager.discretize_edge(config_i, config_j)
        path_replacement = [list(a) for a in zip(px, py)]
        for p in path_replacement:
          new_plan.append(p)
        # add [j, end) plan  
        for index in range(j, len(plan)): 
          new_plan.append(plan[index])
        plan = numpy.array(new_plan)

      elapsed = time.time() - t1
    return plan

  # Backtrack across parents in order to recover path
  # vid: The id of the last node in the graph
  def get_solution(self, vid):

    # Get all the node ids
    planID = []
    while vid is not None:
      planID.append(vid)
      vid = self.parent[vid]

    plan = []
    planID.reverse()
    for i in range(len(planID) - 1):
      startConfig = self.planningEnv.get_config(planID[i])
      goalConfig = self.planningEnv.get_config(planID[i + 1])

      px, py, clen = self.planningEnv.manager.discretize_edge(startConfig, goalConfig)
      plan.append([list(a) for a in zip(px, py)])
      self.planIndices.append(len(plan))
      self.cost += clen

    flatPlan = [item for sublist in plan for item in sublist]
    return flatPlan

  # Visualize the plan
  def simulate(self, plan):
    # Get the map
    envMap = 255*(self.planningEnv.manager.mapImageBW+1) # Hacky way to get correct coloring
    envMap = cv2.cvtColor(envMap, cv2.COLOR_GRAY2RGB)
    
    for i in range(numpy.shape(plan)[0]-1): # Draw lines between each configuration in the plan
      startPixel = Utils.world_to_map(plan[i], self.planningEnv.manager.map_info)
      goalPixel = Utils.world_to_map(plan[i+1], self.planningEnv.manager.map_info)
      cv2.line(envMap,(startPixel[0],startPixel[1]),(goalPixel[0],goalPixel[1]),(255,0,0),5)

    # Generate window
    cv2.namedWindow('Simulation', cv2.WINDOW_NORMAL)
    cv2.imshow('Simulation', envMap)

    # Terminate and exit elegantly
    cv2.waitKey(20000)
    cv2.destroyAllWindows()
