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
import string

class GraphSearchType:
    BFS = 'BreadthFirstSearch'
    DFS = 'DepthFirstSearch'
    UCS = 'UniformCostSearch'
    ASTAR = 'AStar'

class SearchState:
    def __init__(self, state):
        self.state = state
    
    def getState(self):
        return self.state

def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

class Fringe:
    "A container for aggregating node paths during the search process"
    def __init__(self, searchType):
        self.searchType = searchType
        if searchType == GraphSearchType.DFS:
            self.container = util.Stack()
        elif searchType == GraphSearchType.BFS:
            self.container = util.Queue()
        elif searchType == GraphSearchType.UCS or searchType == GraphSearchType.ASTAR:
            self.container = util.PriorityQueue()
        else:
            raise ValueError("No container type defined for fringe")

    def push(self, path, successor, priority=1):
        if isinstance(path, SearchState):
            path = [path, successor]
        else:
            path = path[:] + [successor]
        if not isinstance(self.container, util.PriorityQueue):
            self.container.push(path)
        else:
            self.container.push(path, priority)

    def isEmpty(self):
        return self.container.isEmpty()

    def pop(self):
        return self.container.pop()

def getNode(searchType):
    if not isinstance(searchType, SearchState):
        return searchType[0]
    else:
        return searchType.getState()

class GraphSearch:
    "A generic algorithm to solve graph search problems"

    def __init__(self, searchType):
        self.fringe = Fringe(searchType)
        self.solution = []

    def solve(self, problem, heuristic=nullHeuristic):
        "Run the selected search algorithm"

        # Initialize fringe
        start = SearchState(problem.getStartState())
        successors = problem.getSuccessors(start.getState())
        exploredNodes = [getNode(start)]
        for successor in successors:
            expectedCost = successor[2] + heuristic(successor[0], problem)
            self.fringe.push(start, successor, expectedCost)
        
        while True:
            if self.fringe.isEmpty():
                return []

            # Pop from fringe
            path = self.fringe.pop()
            if len(path) == 0:
                continue
            node = getNode(path[-1])

            # Get cost so far
            costSoFar = 0
            for element in path:
                if not isinstance(element, SearchState):
                    costSoFar = costSoFar + element[2]

            # Goal check
            if problem.isGoalState(node):
                path.append(node)

                # Remove 0=start and -1=goal
                del path[0]
                del path[-1]
                for location in path:
                    self.solution = self.solution[:] + [location[1]]
                return self.solution

            # Explore only new nodes
            if node not in exploredNodes:
                exploredNodes.append(node)
                for successor in problem.getSuccessors(node):
                    if successor[0] not in exploredNodes:
                        expectedCost = costSoFar + successor[2] + heuristic(successor[0], problem)
                        self.fringe.push(path, successor, expectedCost)

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
    dfs = GraphSearch(GraphSearchType.DFS)
    return dfs.solve(problem, nullHeuristic)

def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    bfs = GraphSearch(GraphSearchType.BFS)
    return bfs.solve(problem, nullHeuristic)

def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    ucs = GraphSearch(GraphSearchType.UCS)
    return ucs.solve(problem, nullHeuristic)

def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    aStar = GraphSearch(GraphSearchType.ASTAR)
    return aStar.solve(problem, heuristic)

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
