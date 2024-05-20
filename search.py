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

def depthFirstSearch(problem: SearchProblem):
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
    #util.raiseNotDefined()
    
    #Notes:
    
    # The exploration order might not be exactly what you expect intuitively, because DFS explores deeply along one branch before backtracking. Pacman does not necessarily go through all the explored squares on his way to the goal, as DFS might explore many branches that do not lead to the goal before finding the correct path.

    # The DFS solution is not a least cost solution. DFS does not guarantee the shortest path or least cost because it does not take the cost of the path into consideration; it simply explores as deeply as possible along each branch before backtracking, then it can end up with a longer path.
    
    # Initialize the stack with the start state
    fringe = util.Stack()
    start_state = problem.getStartState()
    # Push the start state onto the stack along with an empty path (since we are just starting)
    fringe.push((start_state, []))  # (current state, path to current state)

    # Set to keep track of visited states (we need that to ensure that we do not revisit states, avoiding loops)
    visited = set() 

    while not fringe.isEmpty():
        current_state, path = fringe.pop()

        # If the current state is the goal, return the path to this state
        if problem.isGoalState(current_state):
            return path # checks if we have reached the goal state

        # If the current state has not been visited, expand it
        if current_state not in visited:
            visited.add(current_state)

            # Get successors and push them onto the stack
            for successor, action, stepCost in problem.getSuccessors(current_state):
                if successor not in visited:
                    new_path = path + [action]
                    fringe.push((successor, new_path))

    return []

def breadthFirstSearch(problem: SearchProblem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    #util.raiseNotDefined()
    # Initialize the queue with the start state
    fringe = util.Queue()
    start_state = problem.getStartState()
    # Push the start state onto the queue along with an empty path (since we are just starting)
    fringe.push((start_state, []))  # (current state, path to current state)

    # Set to keep track of visited states
    visited = set()

    while not fringe.isEmpty():
        # Dequeue the earliest enqueued state from the queue
        current_state, path = fringe.pop()

        # If the current state is the goal, return the path to this state
        if problem.isGoalState(current_state):
            return path

        # If the current state has not been visited, expand it
        if current_state not in visited:
            visited.add(current_state)

            # Get successors and enqueue them
            for successor, action, stepCost in problem.getSuccessors(current_state):
                if successor not in visited:
                    # Create a new path by appending the action to the current path
                    new_path = path + [action]
                    fringe.push((successor, new_path))

    # Return an empty path if no solution is found
    return []

def uniformCostSearch(problem: SearchProblem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    #util.raiseNotDefined()
    # Initialize the priority queue with the start state
    fringe = util.PriorityQueue()
    start_state = problem.getStartState()
    # Push the start state onto the priority queue with a cost of 0
    fringe.push((start_state, []), 0)  # (current state, path to current state), cost

    # Dictionary to keep track of the best cost to reach each state
    best_cost = {start_state: 0}

    while not fringe.isEmpty():
        # Pop the state with the lowest accumulated cost
        current_state, path = fringe.pop()

        # If the current state is the goal, return the path to this state
        if problem.isGoalState(current_state):
            return path

        # Get successors and push them onto the priority queue
        for successor, action, stepCost in problem.getSuccessors(current_state):
            new_cost = best_cost[current_state] + stepCost
            if successor not in best_cost or new_cost < best_cost[successor]:
                # If this is the best cost to reach the successor, update and push onto the priority queue
                best_cost[successor] = new_cost
                new_path = path + [action]
                fringe.push((successor, new_path), new_cost)

    return []

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    #util.raiseNotDefined()
    # Initialize the priority queue with the start state
    fringe = util.PriorityQueue()
    start_state = problem.getStartState()
    # Push the start state onto the priority queue with the combined cost and heuristic
    fringe.push((start_state, []), 0)

    # Dictionary to keep track of the best cost to reach each state
    best_cost = {start_state: 0}

    while not fringe.isEmpty():
        # Pop the state with the lowest combined cost and heuristic
        current_state, path = fringe.pop()

        # If the current state is the goal, return the path to this state
        if problem.isGoalState(current_state):
            return path

        # Get successors and push them onto the priority queue
        for successor, action, stepCost in problem.getSuccessors(current_state):
            new_cost = best_cost[current_state] + stepCost
            heuristic_cost = new_cost + heuristic(successor, problem)
            if successor not in best_cost or new_cost < best_cost[successor]:
                # If this is the best cost to reach the successor, update and push onto the priority queue
                best_cost[successor] = new_cost
                new_path = path + [action]
                fringe.push((successor, new_path), heuristic_cost)

    return []

    #In openMaze:
    # Depth-First Search (DFS): may perform poorly, exploring deep into one branch without regard to the optimal path, leading to many backtracks and a longer path.
    # Breadth-First Search (BFS): will find the shortest path in terms of the number of actions, but it can be slower and less efficient in terms of the number of nodes expanded.
    # Uniform Cost Search (UCS): guarantees the least cost path and performs better than BFS in terms of path cost but might explore many nodes with equal costs, making it slower.
    # A* Search:  typically performs the best in terms of finding the optimal solution efficiently by using a heuristic to guide the search, resulting in fewer nodes expanded compared to UCS.


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
