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
from contextlib import closing

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
    return [s, s, w, s, w, w, s, w]


def depthFirstSearch(problem):
    """
    Search the deepest nodes in the search tree first.

    Your search algorithm needs to return a list of actions that reaches the
    goal. Make sure to implement a graph search algorithm.

    To get started, you might want to try some of these simple commands to
    understand the search problem that is being passed in:

    print "Start:", problem.getStartState()
    print "Is the start a goal?", problem.isGoalState(problem.getStartState())
    print "Start's successors:", problem.getSuccessors(problem.getStartState())
    """
    "*** YOUR CODE HERE ***"
    closed = set()
    fringe = util.Stack()

    start = problem.getStartState()
    fringe.push(start)
    edges = {start: None}

    while True:
        if fringe.isEmpty():
            return []
        current = fringe.pop()
        if problem.isGoalState(current):
            break
        if current not in closed:
            closed.add(current)
            successors = problem.getSuccessors(current)
            for s in successors:
                state, action, cost = s
                if state not in closed:
                    fringe.push(state)
                    edges[state] = (current, action)
    result = []
    while edges[current] is not None:
        previous, action = edges[current]
        result.append(action)
        current = previous

    result.reverse()
    return result


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE ***"
    closed = set()
    fringe = util.Queue()
    start = problem.getStartState()
    fringe.push(start)

    edges = {start: None}

    while True:
        if fringe.isEmpty():
            return []
        current = fringe.pop()
        if problem.isGoalState(current):
            break
        if current not in closed:
            closed.add(current)
            successors = problem.getSuccessors(current)
            for s in successors:
                state, action, cost = s
                if state not in closed:
                    fringe.push(state)
                if state not in edges:
                    edges[state] = (current, action)
    result = []
    while edges[current] is not None:
        previous, action = edges[current]
        result.append(action)
        current = previous

    result.reverse()
    return result


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE ***"
    closed = set()
    fringe = util.PriorityQueue()
    start = problem.getStartState()
    fringe.push(start, 0)

    edges = {start: (None, None, 0)}

    while True:
        if fringe.isEmpty():
            return []
        current = fringe.pop()
        if problem.isGoalState(current):
            break
        if current not in closed:
            closed.add(current)
            successors = problem.getSuccessors(current)
            for s in successors:
                state, action, cost = s
                if state not in closed:
                    previous_cost = edges[current][2]
                    total_cost = previous_cost + cost
                    fringe.update(state, total_cost)
                if state not in edges or edges[state][2] > total_cost:
                    edges[state] = (current, action, total_cost)
    result = []
    while edges[current][0] is not None:
        previous, action, cost = edges[current]
        result.append(action)
        current = previous
    result.reverse()

    return result


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE ***"
    closed = set()
    fringe = util.PriorityQueue()
    start = problem.getStartState()

    heuristicValue = heuristic(start, problem)
    priority = 0 + heuristicValue
    fringe.push(start, priority)

    edges = {start: (None, None, 0)}

    while True:
        if fringe.isEmpty():
            return []
        current = fringe.pop()
        if problem.isGoalState(current):
            break
        if current not in closed:
            closed.add(current)
            successors = problem.getSuccessors(current)
            _, _, previous_cost = edges[current]
            for s in successors:
                if s not in closed:
                    state, action, cost = s
                    total_cost = previous_cost + cost
                    priority = total_cost + heuristic(state, problem)
                    fringe.push(state, priority)
                if state not in edges or edges[state][2] > total_cost:
                    edges[state] = (current, action, total_cost)
    result = []
    while edges[current][0] is not None:
        previous, action, cost = edges[current]
        result.append(action)
        current = previous
    result.reverse()

    return result


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
