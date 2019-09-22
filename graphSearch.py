import string
import util

class GraphSearchType:
    BFS = 'BreadthFirstSearch'
    DFS = 'DepthFirstSearch'
    UCS = 'UniformCostSearch'
    ASTAR = 'AStar'

# class SearchAction:
#     def __init__(self, action):
#         self.action = action

#     def getAction(self):
#         return self.action

class SearchState:
    def __init__(self, state):
        self.state = state
    
    def getState(self):
        return self.state

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
        # if not isinstance(path, list):
        #     path = [path]
        # path = path[:] + [successor]
        if not isinstance(self.container, util.PriorityQueue):
            self.container.push(path)
        else:
            self.container.push(path, priority)

    def isEmpty(self):
        return self.container.isEmpty()

    def pop(self):
        return self.container.pop()

def nullHeuristic(state, problem=None):
    return 0

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
        # print("Starting at node {0}".format(getNode(start)))
        for successor in successors:
            expectedCost = successor[2] + heuristic(successor[0], problem)
            # self.fringe.push(start, successor, expectedCost)
            # print("Adding successor {0} to fringe".format(successor[0]))
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
                # print("Exploring node {0}".format(node))
                exploredNodes.append(node)
                for successor in problem.getSuccessors(node):
                    if successor[0] not in exploredNodes:
                        # print("Adding successor {0} to fringe".format(successor[0]))
                        expectedCost = costSoFar + successor[2] + heuristic(successor[0], problem)
                        self.fringe.push(path, successor, expectedCost)