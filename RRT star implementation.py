import numpy as np 
import pandas as pd
import random 
import matplotlib.pyplot as plt

class Node():
  
  def __init__(self, location, parent, cost):
    self.location = np.array(location)
    self.parent = None
    self.cost = cost


# def distance(x, y):
#   dist = np.sqrt((x.location[0] - y.location[0])**2 + (x.location[1] - y.location[1])**2 + (x.location[2] - y.location[2])**2)
#   return dist 

# distance def for test
def distance(x, y):
  dist = np.sqrt((x.location[0] - y.location[0])**2 + (x.location[1]-y.location[1])**2)
  return dist


# steer function to find the joining location
def steer(a, b, delta):
    dis = distance(a, b)
    slope = delta/dis
    
    if dis > delta : 
      x1 = a.location[0] + slope*(b.location[0] - a.location[0])
      y1 = a.location[1] + slope*(b.location[1] - a.location[1])
      b.location = np.array([x1, y1])
      b.cost = a.cost + distance(a, b)
      return b
    else:
      return b

# function to find the nearest node to b from node_list     
def find_nearest_node(node_list, b):    # node_list is the list of all present nodes and b is the newly randomly plotted node
    min_dist = distance(node_list[0], b)
    check = node_list[0]
    for node in node_list:
      d = distance(node, b)
      if d < min_dist : 
        min_dist = d
        check = node
    return check, min_dist, node_list.index(check)


# function to check if there is any obstacle on the line joining 2 nodes 
def obstacle_detect(new_node, old_node, obstacle_grid):
  # check if there is any obstacle on the line joining the new node and the old node
  # return True if there isn't any obstacle and return False if there is 
  flagOb = 0
  don = distance(new_node, old_node)
  
  defx = new_node.location[0] - old_node.location[0]
  defy = new_node.location[1] - old_node.location[1]
  for ii in range(len(obstacle_grid)): 
    # distance of line joining nodes from obstacle check
    ua = abs((obstacle_grid[ii,1]-new_node.location[1])*defx + (obstacle_grid[ii,0]-new_node.location[0])*defy)
    dist_ob = ua/don

    # side of obstacle check 
    l_new = (new_node.location[1] - obstacle_grid[ii,1])*defy + (new_node.location[0] - obstacle_grid[ii,0])*defx
    l_old = (old_node.location[1] - obstacle_grid[ii,1])*defy + (old_node.location[0] - obstacle_grid[ii,0])*defx
    side = np.sign(l_new*l_old)
    dist_old = np.sqrt((old_node.location[0] - obstacle_grid[ii, 0])**2 + (old_node.location[1] - obstacle_grid[ii, 1])**2)
    dist_new = np.sqrt((new_node.location[0] - obstacle_grid[ii, 0])**2 + (new_node.location[1] - obstacle_grid[ii, 1])**2)

    # final check with OR of the two checks 
    check_1 = ((dist_old > obstacle_grid[ii,2]) and (dist_new > obstacle_grid[ii,2])) and (side == 1)
    check_2 = dist_ob > obstacle_grid[ii,2]
    final_check = check_1 or check_2

    flagOb = flagOb + (1 - int(final_check)) 
    #flag_array.append(flagOb)
  return (flagOb == 0)

def circular_boundary_based_obstacle_detection(new_node, obstacle_matrix, vehicle_radius):
  # only for actual car implementation, not for testing
  # similar to the obstacle_detect() function above 
  pass



################ Main Function ################# 

# for testing
dis_list = []
don_list = []
flag_array = []
ar1 = np.zeros((100, 100))                                         # occupancy grid of just zeros
obstacle_grid = np.array([[50,40,10],[80,80,20], [20, 70, 8], [30, 30, 25]])  # each row is x, y, radius
for i in range(len(obstacle_grid)):                              # assigning 1 to centres of obstacles
  ar1[obstacle_grid[i,0], obstacle_grid[i,1]] = 1

occupancy_grid = ar1   # import an occupancy grid, format not clear yet hopefully x, y, flag form
total_nodes = len(occupancy_grid[:,0])*len(occupancy_grid[0,:])      # total possible nodes = 100*1000
#x_max = np.max(occupancy_grid[:,0])
#x_min = 0
#y_max = np.max(occupancy_grid[:,1])
#y_min = np.min(occupancy_grid[:,1])
node_list = []
start_pos = np.array([80,10])        # [x,y] got from transformation from global to vehicle co-ordinates
end_pos = np.array([10,70])       # [x,y] got from transformation from global to vehicle co-ordinates
start_node = Node(start_pos, None, 0)
end_node = Node(end_pos, None, 0)
delta = 5               # for steering 
check_radius = 10       # for finding the best parent 
goal_check_radius = 30  # for checking if we are near the goal
node_list.append(start_node) # add the start node to the node list 

while len(node_list) < 4000:
     
  # get a random point in the non-obsracle area on the map 
  # Note - random points are points from the point cloud data itself
  xrand_test = random.randint(0,100)
  yrand_test = random.randint(0,100)
  #rand_ind = random.randint(0, len(occupancy_grid[:,0])-1)
  #x_rand_new = occupancy_grid[rand_ind, 0]
  #y_rand_new = occupancy_grid[rand_ind, 1] 
  #pos_rand_new = np.array([x_rand_new, y_rand_new]) 
  pos_rand_new = np.array([xrand_test, yrand_test])


  # check if any of the existing nodes is the goal position itself 
  # if it is, then break the loop and give out the path
  flag = 0 
  for nn in node_list : 
    if (nn.location).all == (end_node.location).all :
      flag = flag + 1
  if flag > 0 : 
    break 

  # create a dummy node for this new location and find the nearest node
  new_rand_node = Node(pos_rand_new, None, 0)
  nearest_node, nearest_dist, nearest_node_pos = find_nearest_node(node_list, new_rand_node)
  
  # check if there is any obstacle between the new node and the nearest node
  # if there is no obstacle between new node and nearest node, there won't be any obstacle between steered node and nearest node either 
  if obstacle_detect(new_rand_node, nearest_node, obstacle_grid) == True :

    # assign parents and cost to this node only if there is no obstacle     
    new_rand_node.parent = nearest_node
    new_rand_node.cost = nearest_node.cost + distance(new_rand_node, nearest_node) 
    # steer the new node to modified sample node
    modified_sample_node = steer(nearest_node, new_rand_node, delta)

    # modified_sample_node is the new node whos parent is the nearest node and cost has been updated according to the steer function 
    # we now need to assign it to the best parent
    # get the list of nodes within 'check_radius' distance from this new node
    # consider only those nodes which have no obstacle between them and this new node
    if obstacle_detect(modified_sample_node, nearest_node, obstacle_grid) == True :
      
      nodes_in_vicinity = []
      for n in node_list:
        d = distance(n, modified_sample_node)
        if (d < check_radius) and (obstacle_detect(modified_sample_node, n, obstacle_grid) == True): 
          nodes_in_vicinity.append(n)
         
      if len(nodes_in_vicinity) > 0 :
        # get the node from this list which has the minimum cost and doesn't collide with any obstacle  
        c_min = distance(nodes_in_vicinity[0], modified_sample_node) + nodes_in_vicinity[0].cost
        least_index = 0
        for i, m in enumerate(nodes_in_vicinity) :
          dd = distance(modified_sample_node, m)
          new_cost = dd + m.cost
          if (new_cost < modified_sample_node.cost) and (obstacle_detect(modified_sample_node, m, obstacle_grid) == True):
            modified_sample_node.cost = new_cost 
            least_index = i
       
        # Assign the node with the least cost as the parent node
        # this node is assigned from node_list rather than the nodes_in_vicinity list
        new_parent = nodes_in_vicinity[least_index] 
        for checker in node_list : 
          if (new_parent.location).all == (checker.location).all :
            modified_sample_node.parent = checker
            modified_sample_node.cost = checker.cost + distance(modified_sample_node, checker)
            
      # Append the new node to the list of all present nodes
      node_list.append(modified_sample_node)
  else:
    continue 

# Once the while loop ends, we check for the node nearest to the end node
goal_dist = []
for iter in range(len(node_list)) :
  goal_distance = distance(node_list[iter], end_node)
  if obstacle_detect(node_list[iter], end_node, obstacle_grid) == True:
    goal_dist.append(goal_distance)
  else:
    goal_dist.append(np.inf)

# assign the nearest node as the parent of the end node the corresponding cost
find_min = min(goal_dist)
nearest_to_goal = node_list[goal_dist.index(find_min)]    # nearest node to the goal
end_node.parent = nearest_to_goal 
end_node.cost = nearest_to_goal.cost + find_min

# getting the sequenced list of nodes from the start position to the goal position 
current_node = end_node
way = [end_node]
while current_node.parent != None : 
  current_node = current_node.parent
  way.append(current_node)

# return the reverse of the list, since we back-trace the path through parents
way = way[::-1]               
 
  
# improve the obtained way by straightening it 



# printing and visualizing results
x_posn = np.zeros((len(node_list),1))
y_posn = np.zeros((len(node_list),1))
for it, indt in enumerate(node_list):
  x_posn[it] = indt.location[0]
  y_posn[it] = indt.location[1]
x_pos = np.zeros((len(way),1))
y_pos = np.zeros((len(way),1))
for i, ind in enumerate(way):
  x_pos[i] = ind.location[0]
  y_pos[i] = ind.location[1]
plt.plot(x_posn, y_posn, 'oy', markersize = '2')
plt.plot(x_pos, y_pos, 'o:r')
plt.plot(obstacle_grid[:,0], obstacle_grid[:,1], 'ob')
plt.show()

# print the way points 
print(len(node_list))
print(end_node.cost)
print(len(way))
print(way[2].location[1])
for i in way:
  print(i.location)