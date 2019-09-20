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
import itertools

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

    print("Start:", problem.getStartState())
    print("Is the start a goal?", problem.isGoalState(problem.getStartState()))
    print("Start's successors:", problem.getSuccessors(problem.getStartState()))
    """
    "*** YOUR CODE HERE IF YOU WANT TO PRACTICE ***"
    util.raiseNotDefined()


def breadthFirstSearch(problem):
    """Search the shallowest nodes in the search tree first."""
    "*** YOUR CODE HERE IF YOU WANT TO PRACTICE ***"
    util.raiseNotDefined()


def uniformCostSearch(problem):
    """Search the node of least total cost first."""
    "*** YOUR CODE HERE IF YOU WANT TO PRACTICE ***"
    util.raiseNotDefined()


def nullHeuristic(state, problem=None):
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0


def aStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has the lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE IF YOU WANT TO PRACTICE ***"
    # util.raiseNotDefined()
    open = util.PriorityQueue()
    open.push(item=(problem.getStartState(), [], 0), priority=0)  # open.insert(make-root-node(init()))
    closed = []
    while True:
        if open.isEmpty():
            return False
        food_state, action, former_cost = open.pop()
        if problem.isGoalState(food_state):
            return action
        if food_state not in closed:  # if state(σ) ∉ closed or g(σ) < best-g(state(σ)):
            closed.append(food_state)  # closed := closed ∪ {state(σ)}
            for successor, next_action, succ_cost in problem.getSuccessors(food_state):
                g_state = succ_cost + former_cost
                h_state = heuristic(successor, problem)
                # not weighted
                open.push(item=(successor, action + [next_action], succ_cost + former_cost), priority=g_state + h_state)


def iterativeDeepeningSearch(problem):
    """Search the deepest node in an iterative manner."""
    "*** YOUR CODE HERE FOR TASK 1 ***"
    fringe = util.Queue()
    # fringe = util.Stack()
    depth = -1
    while True:
        depth += 1
        node_visited = []
        fringe.push((problem.getStartState(), [], 0))
        (pre_state, pre_action, pre_cost) = fringe.pop()
        node_visited.append(pre_state)
        while not problem.isGoalState(pre_state):
            successors = problem.getSuccessors(pre_state)
            for new_node in successors:
                new_state, new_action, new_cost = new_node
                if (not new_node[0] in node_visited) and (len(pre_action) < depth):
                # if not new_node[0] in node_visited:
                    fringe.push((new_state, pre_action + [new_action], pre_cost + new_cost))
                    node_visited.append(new_state)
            if fringe.isEmpty():
                break
            (pre_state, pre_action, pre_cost) = fringe.pop()
        if problem.isGoalState(pre_state):
            return pre_action


def waStarSearch(problem, heuristic=nullHeuristic):
    """Search the node that has has the weighted (x 2) lowest combined cost and heuristic first."""
    "*** YOUR CODE HERE FOR TASK 2 ***"
    # util.raiseNotDefined()
    open = util.PriorityQueue()
    open.push(item=(problem.getStartState(), [], 0), priority=0)  # open.insert(make-root-node(init()))
    closed = []
    while True:
        if open.isEmpty():
            return False
        food_state, action, former_cost = open.pop()
        if problem.isGoalState(food_state):
            return action
        if food_state not in closed:  # if state(σ) ∉ closed or g(σ) < best-g(state(σ)):
            closed.append(food_state)  # closed := closed ∪ {state(σ)}
            for successor, next_action, succ_cost in problem.getSuccessors(food_state):
                g_state = succ_cost + former_cost
                h_state = heuristic(successor, problem)
                w = 2
                # not weighted
                open.push(item=(successor, action + [next_action], succ_cost + former_cost),
                          priority=g_state + w * h_state)


# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
ids = iterativeDeepeningSearch
wastar = waStarSearch
