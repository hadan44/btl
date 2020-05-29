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
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE ***"
    stack = util.Stack()
    visited = []

    #push startState and initial path
    stack.push((problem.getStartState(), []))
    
    while not stack.isEmpty():
        # get currentState and path
        currentState, path = stack.pop()

        # check is the problem is at goal state
        if problem.isGoalState(currentState):
            return path

        # check if currentState not visited then visit it
        if currentState not in visited:
            visited.append(currentState)
            
            # check all neighbor of currentState in graph and update path if neighbor not visited
            neighbors = problem.getSuccessors(currentState)
            for neighbor in neighbors:
                neighborState = neighbor[0]
                if neighborState not in visited:
                    action = [neighbor[1]]
                    newPath = path + action
                    stack.push((neighborState, newPath))
    
    # util.raiseNotDefined()

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    queue = util.Queue()
    visited = []

    # push startState and initial path
    queue.push((problem.getStartState(), []))

    # first visit the stateState
    visited.append(problem.getStartState())

    while not queue.isEmpty():
        # get currentState and path
        currentState, path = queue.pop()

        # check is the problem is at goal state
        if problem.isGoalState(currentState):
            return path

        # check all neighbor of currentState in graph and visit neighbor if not visited then update the path
        neighbors = problem.getSuccessors(currentState)
        for neighbor in neighbors:
            neighborState = neighbor[0]
            if neighborState not in visited:
                visited.append(neighborState)
                action = [neighbor[1]]
                newPath = path + action
                queue.push((neighborState, newPath))

    # util.raiseNotDefined()

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    priorityQueue = util.PriorityQueue()
    visited = []

    # push startState and initial path
    priorityQueue.push((problem.getStartState(), []), 0)

    while not priorityQueue.isEmpty():
        # get currentState and path
        currentState, path = priorityQueue.pop()
       
        # check is the problem is at goal state
        if problem.isGoalState(currentState):
            return path
        
        # check if currentState not visited then visit it
        if currentState not in visited:
            visited.append(currentState)

            # check all neighbor of currentState in graph and if not visited then update the path
            neighbors = problem.getSuccessors(currentState)
            for neighbor in neighbors:
                neighborState = neighbor[0]
                if neighborState not in visited:
                    action = [neighbor[1]]
                    newPath = path + action

                    oldCost = problem.getCostOfActions(path)
                    newCost = problem.getCostOfActions(newPath)

                    # check if neighborState is not in queue then push the new path with it priority ( new cost )
                    if neighborState not in priorityQueue.heap:
                        priorityQueue.push((neighborState, newPath), newCost)
                    
                    # if it is in queue then check if current path is less cost than the previous one then update the new path with its priority
                    else:
                        if oldCost > newCost:
                            priorityQueue.update((neighborState, newPath), newCost)


    # util.raiseNotDefined()

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    # priotiryFunction to calculate f = g + h where g is the cost and h is heuristic function
    # state[1] is the path, and state[0] is the state
    def priorityFunction(state):
        return problem.getCostOfActions(state[1]) + heuristic(state[0], problem)

    priorityQueue = util.PriorityQueueWithFunction(priorityFunction)

    visited = []

    priorityQueue.push((problem.getStartState(), []))

    while not priorityQueue.isEmpty():
        # get currentState and path
        currentState, path = priorityQueue.pop()

        # check is the problem is at goal state
        if problem.isGoalState(currentState):
            return path
        
        # check if currentState not visited then visit it
        if currentState not in visited:
            visited.append(currentState)

            # check all neighbor of currentState in graph and if not visited then update the path
            neighbors = problem.getSuccessors(currentState)
            for neighbor in neighbors:
                neighborState = neighbor[0]
                if neighborState not in visited:
                    action = [neighbor[1]]
                    newPath = path + action
                    priorityQueue.push((neighborState, newPath))

    # util.raiseNotDefined()


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
