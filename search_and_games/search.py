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
from game import Directions
from typing import List

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




def tinyMazeSearch(problem: SearchProblem) -> List[Directions]:
    """
    Returns a sequence of moves that solves tinyMaze.  For any other maze, the
    sequence of moves will be incorrect, so only use this for tinyMaze.
    """
    s = Directions.SOUTH
    w = Directions.WEST
    return  [s, s, w, s, w, w, s, w]

def depthFirstSearch(problem: SearchProblem) -> List[Directions]:
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
    frontier = util.Stack()

    frontier.push((problem.getStartState(), []))

    visited = set()

    while not frontier.isEmpty(): 
        state, path = frontier.pop()

        if state in visited: 
            continue

        visited.add(state)

        if problem.isGoalState(state): 
            return path
        
        for nextState, move, cost in problem.getSuccessors(state): 
            if nextState in visited: 
                continue
            frontier.push((nextState, path + [move]))

    return []


def breadthFirstSearch(problem: SearchProblem) -> List[Directions]:
    """Search the shallowest nodes in the search tree first."""
    frontier = []

    parent_map = {problem.getStartState(): None}
    frontier.append(problem.getStartState())

    while frontier:
        current = frontier.pop(0)

        if problem.isGoalState(current):
            path = []
            while parent_map[current] is not None:
                path.append(parent_map[current][1])
                current = parent_map[current][0]
            return path[::-1]
        
        for neighbor in problem.getSuccessors(current):
            if neighbor[0] not in parent_map:
                parent_map[neighbor[0]] = [current, neighbor[1]]
                frontier.append(neighbor[0])

def iterativeDeepeningSearch(problem):
    """
    Perform DFS with increasingly larger depth. Begin with a depth of 1 and increment depth by 1 at every step.
    """
    def depthLimitedSearch(current, depth, path, visited):
        if problem.isGoalState(current):
            return path
        if depth == 0:
            return None
        for neighbor in problem.getSuccessors(current):
            state = neighbor[0]
            if state not in visited:
                out = depthLimitedSearch(state, depth-1, path + [neighbor[1]], visited | {state})
                if out is not None:
                    return out
        return None
            
    depth = 0
    while(1):
        final_path = depthLimitedSearch(problem.getStartState(), depth, [], {problem.getStartState()})
        if final_path is not None:
            return final_path
        depth += 1

def uniformCostSearch(problem: SearchProblem) -> List[Directions]:
    """Search the node of least total cost first."""
    frontier = util.PriorityQueue()

    frontier.push((problem.getStartState(), [], 0), 0)

    costIdeal = {}

    while not frontier.isEmpty():
        state, path, cost = frontier.pop()

        if state in costIdeal and cost >= costIdeal[state]: 
            continue

        if problem.isGoalState(state): 
            return path
        
        costIdeal[state] = cost

        for nextState, move, nextCost in problem.getSuccessors(state): 
            updatedCost = cost + nextCost
            
            if nextState in costIdeal and updatedCost >= costIdeal[nextState]: 
                continue
            frontier.push((nextState, path + [move], updatedCost), updatedCost)

    return []

def nullHeuristic(state, problem=None) -> float:
    """
    A heuristic function estimates the cost from the current state to the nearest
    goal in the provided SearchProblem.  This heuristic is trivial.
    """
    return 0

def aStarSearch(problem: SearchProblem, heuristic=nullHeuristic) -> List[Directions]:
    """Search the node that has the lowest combined cost and heuristic first."""
    frontier = util.PriorityQueue()
    start = problem.getStartState()
    frontier.push((start, [], 0), heuristic(start, problem))
    best_g = {}  # state -> best g-cost at which state was expanded

    while not frontier.isEmpty():
        state, actions, g = frontier.pop()

        # Skip stale entries: already expanded this state with a better or equal g
        if state in best_g and best_g[state] <= g:
            continue
        best_g[state] = g

        if problem.isGoalState(state):
            return actions

        for successor, action, stepCost in problem.getSuccessors(state):
            new_g = g + stepCost
            # Only push if this is a strictly better path than any previously seen
            if successor not in best_g or new_g < best_g[successor]:
                new_f = new_g + heuristic(successor, problem)
                frontier.push((successor, actions + [action], new_g), new_f)

    return []

# Abbreviations
bfs = breadthFirstSearch
dfs = depthFirstSearch
astar = aStarSearch
ucs = uniformCostSearch
ids = iterativeDeepeningSearch
