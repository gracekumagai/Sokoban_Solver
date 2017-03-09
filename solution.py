#Look for #IMPLEMENT tags in this file. These tags indicate what has
#to be implemented to complete the Sokoban warehouse domain.

#   You may add only standard python imports---i.e., ones that are automatically
#   available on TEACH.CS
#   You may not remove any imports.
#   You may not import or otherwise source any of your own files

# import os for time functions
import os
from search import * #for search engines
from sokoban import SokobanState, Direction, PROBLEMS, sokoban_goal_state #for Sokoban specific classes and problems
from test_problems import PROBLEMS

INF = 2**31

#SOKOBAN HEURISTICS
def heur_displaced(state):
  '''trivial admissible sokoban heuristic'''
  '''INPUT: a sokoban state'''
  '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''       
  count = 0
  for box in state.boxes:
    if box not in state.storage:
      count += 1
  return count

def heur_manhattan_distance(state):
#IMPLEMENT
    '''admissible sokoban heuristic: manhattan distance'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''      
    #We want an admissible heuristic, which is an optimistic heuristic. 
    #It must always underestimate the cost to get from the current state to the goal.
    #The sum Manhattan distance of the boxes to their closest storage spaces is such a heuristic.  
    #When calculating distances, assume there are no obstacles on the grid and that several boxes can fit in one storage bin.
    #You should implement this heuristic function exactly, even if it is tempting to improve it.
    #Your function should return a numeric value; this is the estimate of the distance to the goal.
    
    #Assume that a solution exists - ie that there exist storage locations and the restriction list does not include all storage options
    sum_distance = 0
    for box, i in state.boxes.items():
      closest = INF
      for goal in state.storage:
          if (state.restrictions == None) or (goal in state.restrictions[i]):
            distance = abs(box[0] - goal[0]) + abs(box[1] - goal[1])
            if distance < closest:
              closest = distance        
      sum_distance += closest
        
    return sum_distance


def heur_alternate(state):
#IMPLEMENT
    '''a better sokoban heuristic'''
    '''INPUT: a sokoban state'''
    '''OUTPUT: a numeric value that serves as an estimate of the distance of the state to the goal.'''        
    #heur_manhattan_distance has flaws.   
    #Write a heuristic function that improves upon heur_manhattan_distance to estimate distance between the current state and the goal.
    #Your function should return a numeric value for the estimate of the distance to the goal.
    global prev_state
    global prev_heur

    #Return the last heuristic if the boxes remain in the same location A.K.A only the robot moved
    #If at START
    if state.parent == None:
      prev_state = state.boxes
    else:
      if prev_state == state.boxes:
        return prev_heur
    
    #Initialize the current state as the previous state for the next iteration
    prev_state = state.boxes

    #Use for upgraded Manhattan Distance Calculation
    sum_distance = 0
    assigned = [0 for x in range(len(state.storage))]

    index = 0

    #Only look at boxes that have not been stored
    for box, i in state.boxes.items():
      closest = INF

      if not isAtStorage(state, box, i):
        #If the box is cornered by the border or an obstacle
        if isBlocked(state, box): 
          prev_heur = INF
          return INF #Unsolvable

        #If the box can be stored in ANY storage
        elif state.restrictions == None:

          for goal in state.storage:

            #Check if the box is stuck on the edge
            if isUnsolvableEdge(state, box, goal):
              prev_heur = INF
              return INF #Unsolvable

            #Assign specific storages to boxes (not done in prev Manhattan)
            elif goal not in assigned:
              distance = abs(box[0] - goal[0]) + abs(box[1] - goal[1])
              if distance < closest:
                closest = distance
                assigned[index] = goal
          index += 1

        #For boxes with specific storage bins
        else:
          for goal in state.restrictions[i]:

            #Check if box is stuck on edge
            if isUnsolvableEdge(state, box, goal):
              prev_heur = INF
              return INF #Unsolvable

            else:
              distance = abs(box[0] - goal[0]) + abs(box[1] - goal[1])
              if distance < closest:
                closest = distance
          sum_distance += closest
    
        #Increase cost by number of obstacles within the box's radius
        sum_distance += personalSpace(state, box)

    prev_heur = sum_distance
    return prev_heur

def personalSpace(state, box):
  '''Check if any of the 8 neighboring tiles to the box have an obstacle, if they do increase the cost by number of obstacles in this space
  Inspired by looking at the difficult problems 30-39'''
  personal_space = ((box[0], box[1]+1), (box[0]+1, box[1]), (box[0]-1, box[1]), (box[0], box[1]-1),(box[0]-1, box[1]-1),(box[0]+1, box[1]+1),(box[0]+1, box[1]-1), (box[0]-1, box[1]+1))
  return len(set(personal_space)&set(state.obstacles)) 

#def personalSpaceGoal(state, goal):
#  personal_space = ((goal[0], goal[1]+1), (goal[0]+1, goal[1]), (goal[0]+1, goal[1]+1), (goal[0]-1, goal[1]-1), (goal[0]-1, goal[1]), (goal[0], goal[1]-1))
#  return len(set(personal_space)&set(state.obstacles))

def isAtStorage(state, box, i):
  '''Check if box is at an allowed storage location'''
  if box in state.storage and (state.restrictions == None or box in state.restrictions[i]):
    return True
  return False

def isSolvable(restrictions, i):
  '''Check if the problem is solvable by looking at the boxes restritions'''
  if restrictions != None and restrictions[i] == None:
    return False
  return True

def isCornered(state, box):
  if (box == (0,0)) or (box == (0, state.height - 1)) or (box == (state.width - 1, 0)) or (box == (state.width - 1, state.height -1)):
    return True
  return False

def isBlocked(state, box):
  '''Assuming bottom right corner is (0,0) and top right is (state.width-1, state.height-1) for descriptions'''
  #Obstacle to the right      Obstacle to the left
  if any(i in state.obstacles for i in ((box[0]+1, box[1]), (box[0]-1, box[1]))) or box[0] == 0 or box[1] == state.width-1:
    if any(i in state.obstacles for i in ((box[0], box[1]+1), (box[0], box[1]-1))) or box[1] == 0 or box[1] == state.height-1:
      return True
  return False

def isUnsolvableEdge(state, box, goal):
  '''Check if the box and goal are located on the same edge - if so the problem is Solvable and return True.
  Otherwise, box is on the edge and cannot be moved to the goal - it is unsolvable'''
  #Horizontal Edge
  if (box[0] != goal[0]) and ((box[0] == state.width-1) or (box[0] == 0)):
    return True
  #Vertical Edge
  elif (box[1] != goal[1]) and ((box[1] == state.height-1) or (box[1] == 0)):
    return True
  return False

def fval_function(sN, weight):
#IMPLEMENT
    """
    Provide a custom formula for f-value computation for Anytime Weighted A star.
    Returns the fval of the state contained in the sNode.

    @param sNode sN: A search node (containing a SokobanState)
    @param float weight: Weight given by Anytime Weighted A star
    @rtype: float
    """
    #Many searches will explore nodes (or states) that are ordered by their f-value.
    #For UCS, the fvalue is the same as the gval of the state. For best-first search, the fvalue is the hval of the state.
    #You can use this function to create an alternate f-value for states; this must be a function of the state and the weight.
    #The function must return a numeric f-value.
    #The value will determine your state's position on the Frontier list during a 'custom' search.
    #You must initialize your search engine object as a 'custom' search engine if you supply a custom fval function.

    #For Weighted A* f(node) = g(node) + w*h(node)

    return (sN.gval + weight*sN.hval)

def anytime_gbfs(initial_state, heur_fn, timebound = 10):
#IMPLEMENT
    '''Provides an implementation of anytime greedy best-first search, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False''' 

    #Init Time
    current_t = os.times()[0]
    final_t = current_t + timebound
 
    #Initialize Search
    se = SearchEngine('best_first', 'default')
    se.init_search(initial_state, goal_fn=sokoban_goal_state, heur_fn = heur_fn)

    #Init Cost
    cost = [float("inf"), float("inf"), float("inf")]

    #Perform initial search
    solution = se.search(timebound)
    best_solution = solution

    #Keep searching until time runs out
    while current_t < final_t:
      current_t = os.times()[0]
      remaining_t = final_t - current_t      

      if solution != False:
        if solution.gval <= cost[0]:
          cost = [solution.gval, solution.gval, solution.gval]
          best_solution = solution #Update best solution if the cost is less than previous
          solution = se.search(remaining_t, cost) #Use prev cost as a bound on next search
        else:
          break
      else:
        break

    return best_solution
                  

def anytime_weighted_astar(initial_state, heur_fn, weight=1., timebound = 10):
#IMPLEMENT
    '''Provides an implementation of anytime weighted a-star, as described in the HW1 handout'''
    '''INPUT: a sokoban state that represents the start state and a timebound (number of seconds)'''
    '''OUTPUT: A goal state (if a goal is found), else False''' 

    #Init Time
    current_t = os.times()[0]
    final_t = current_t + timebound
 
    #Init Search
    wrapped_fval_function = (lambda sN: fval_function(sN, weight))
    se = SearchEngine(strategy='custom', cc_level= 'default')
    se.init_search(initial_state, goal_fn=sokoban_goal_state, heur_fn = heur_fn, fval_function = wrapped_fval_function)

    #Init Cost
    cost = [float("inf"), float("inf"), float("inf")]

    #Perform initial search
    solution = se.search(timebound)
    best_solution = solution

    #Keep searching until time runs out
    while current_t < final_t:
      current_t = os.times()[0]
      remaining_t = final_t - current_t      

      if solution != False:
        if solution.gval <= cost[0]:
          cost = [solution.gval, solution.gval, solution.gval]
          best_solution = solution #Update best solution if the cost is less than previous
          solution = se.search(remaining_t, cost) #Use prev cost as a bound on next search
        else:
          break
      else:
        break

    return best_solution


    return False

if __name__ == "__main__":
  #TEST CODE
  solved = 0; unsolved = []; counter = 0; percent = 0; timebound = 2; #2 second time limit for each problem
  print("*************************************")  
  print("Running A-star")     

  for i in range(0, 10): #note that there are 40 problems in the set that has been provided.  We just run through 10 here for illustration.

    print("*************************************")  
    print("PROBLEM {}".format(i))
    
    s0 = PROBLEMS[i] #Problems will get harder as i gets bigger

    se = SearchEngine('astar', 'full')
    se.init_search(s0, goal_fn=sokoban_goal_state, heur_fn=heur_displaced)
    final = se.search(timebound)

    if final:
      final.print_path()
      solved += 1
    else:
      unsolved.append(i)    
    counter += 1

  if counter > 0:  
    percent = (solved/counter)*100

  print("*************************************")  
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))  
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))      
  print("*************************************") 

  solved = 0; unsolved = []; counter = 0; percent = 0; timebound = 8; #8 second time limit 
  print("Running Anytime Weighted A-star")   

  for i in range(0, 10):
    print("*************************************")  
    print("PROBLEM {}".format(i))

    s0 = PROBLEMS[i] #Problems get harder as i gets bigger
    weight = 10
    final = anytime_weighted_astar(s0, heur_fn=heur_displaced, weight=weight, timebound=timebound)

    if final:
      final.print_path()   
      solved += 1 
    else:
      unsolved.append(i)
    counter += 1      

  if counter > 0:  
    percent = (solved/counter)*100   
      
  print("*************************************")  
  print("{} of {} problems ({} %) solved in less than {} seconds.".format(solved, counter, percent, timebound))  
  print("Problems that remain unsolved in the set are Problems: {}".format(unsolved))      
  print("*************************************") 