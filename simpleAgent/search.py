# search.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


"""
In search.py, you will implement generic search algorithms which are called by
Pacman agents (in searchAgents.py).
"""

import util

class SearchProblem:
    """
    This class outlines the structure of a search problem, but doesn't implement
    any of the methods (in object-oriented terminology: an abstract class).

    You do not need to change anything in this class, ever.
    """

    def getStartState(self):
        """
        Returns the start state for the search problem.
        """
        util.raiseNotDefined()

    def isGoalState(self, state):
        """
          state: Search state

        Returns True if and only if the state is a valid goal state.
        """
        util.raiseNotDefined()

    def getSuccessors(self, state):
        """
          state: Search state

        For a given state, this should return a list of triples, (successor,
        action, stepCost), where 'successor' is a successor to the current
        state, 'action' is the action required to get there, and 'stepCost' is
        the incremental cost of expanding to that successor.
        """
        util.raiseNotDefined()

    def getCostOfActions(self, actions):
        """
         actions: A list of actions to take

        This method returns the total cost of a particular sequence of actions.
        The sequence must be composed of legal moves.
        """
        util.raiseNotDefined()


def tinyMazeSearch(problem):
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    from game import Directions
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem):
    """Search the deepest nodes in the search tree first."""
    """Search the deepest nodes in the search tree first."""

    #states to be explored (LIFO). holds nodes in form (state, action)
    frontier = util.Stack()
    #previously explored states (for path checking), holds states
    exploredNodes = []
    #define start node
    startState = problem.getStartState()
    startNode = (startState, [])
    
    frontier.push(startNode)
    
    while not frontier.isEmpty():
        #begin exploring last (most-recently-pushed) node on frontier
        currentState, actions = frontier.pop()
        
        if currentState not in exploredNodes:
            #mark current node as explored
            exploredNodes.append(currentState)

            if problem.isGoalState(currentState):
                return actions
            else:
                #get list of possible successor nodes in 
                #form (successor, action, stepCost)
                successors = problem.getSuccessors(currentState)
                
                #push each successor to frontier
                for succState, succAction, succCost in successors:
                    newAction = actions + [succAction]
                    newNode = (succState, newAction)
                    frontier.push(newNode)

    return actions  
    

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    # define a queue for open list
    open_queue = util.Queue()
    # initialize open stack with start position
    open_queue.push((problem.getStartState(), []))
    visited_list = []

    while not open_queue.isEmpty():
        X, actions = open_queue.pop()
        if X not in visited_list:
            visited_list.append(X)
            if problem.isGoalState(X):
                return actions
            else:
                # generate successors of X
                    children_of_X = problem.getSuccessors(X)

                    for each_child in children_of_X:
                            open_queue.push((each_child[0], actions + [each_child[1]]))
    return []

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    # define a queue for open list
    open_queue = util.PriorityQueue()
    # initialize open stack with start position
    open_queue.push((problem.getStartState(), [], 0), 0)
    visited_set = set()
    closed_list = []
    while not open_queue.isEmpty():
        X, actions, cost = open_queue.pop()
        visited_set.add(X)
        if problem.isGoalState(X):
            return actions
        else:
            # generate successors of X
            if (X not in closed_list):
                children_of_X = problem.getSuccessors(X)
                closed_list.append(X)

                for each_child in children_of_X:
                    if (each_child[0] in visited_set):
                        pass
                    else:
                        open_queue.update((each_child[0], actions + [each_child[1]], each_child[2] + cost), each_child[2] + cost)
    return []

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
     # define a stack for open list
    open_queue = util.PriorityQueue()
    # initialize open stack with start position
    open_queue.push((problem.getStartState(), [], 0), 0)
    visited_set = []
    closed_list = []
    while not open_queue.isEmpty():
        X, actions, cost = open_queue.pop()
        visited_set.append(X)
        if problem.isGoalState(X):
            return actions
        else:
            # generate successors of X
            if (X not in closed_list):
                children_of_X = problem.getSuccessors(X)
                closed_list.append(X)

                for each_child in children_of_X:
                    if (each_child[0] in visited_set):
                        pass
                    else:
                        heuristic_value = heuristic(each_child[0],problem)
                        open_queue.update((each_child[0], actions + [each_child[1]], each_child[2] + cost),
                                          each_child[2] + cost + heuristic_value)
    return []


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
