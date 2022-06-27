## EX NO:03
## DATE:10.05.2022
# Dijkstra's Shortest Path Algorithm
## AIM

To develop a code to find the shortest route from the source to the destination point using Dijkstra's shortest path algorithm.

## THEORY
Dijkstra algorithm is a single-source shortest path algorithm. Here, single-source means that only one source is given, and we have to find the shortest path from the source to all the destination nodes.

## DESIGN STEPS

### STEP 1:
Identify a location in the google map: Asaripallam

### STEP 2:
Select a specific number of nodes with distance

### STEP 3: 
Create a dictionary with all the node pairs (keys) and their respective distances as the values

### STEP 4: 
Implement the search algorithm by passing any two nodes/places to find a best route.

### STEP 5: 
Display the route sequence.

## ROUTE MAP

<img width="960" alt="map" src="https://user-images.githubusercontent.com/75235150/167455836-be704b0f-1a48-446c-9c2c-ef32a135cdee.png">

## PROGRAM
Developed by: A Graham stanes
Register  No:  212220230020
```
%matplotlib inline
import matplotlib.pyplot as plt
import random
import math
import sys
from collections import defaultdict, deque, Counter
from itertools import combinations
import heapq

class Problem(object):
    """The abstract class for a formal problem. A new domain subclasses this,
    overriding `actions` and `results`, and perhaps other methods.
    The default heuristic is 0 and the default action cost is 1 for all states.
    When yiou create an instance of a subclass, specify `initial`, and `goal` states 
    (or give an `is_goal` method) and perhaps other keyword args for the subclass."""

    def __init__(self, initial=None, goal=None, **kwds): 
        self.__dict__.update(initial=initial, goal=goal, **kwds) 
        
    def actions(self, state):        
        raise NotImplementedError
    def result(self, state, action): 
        raise NotImplementedError
    def is_goal(self, state):        
        return state == self.goal
    def action_cost(self, s, a, s1): 
        return 1
    
    def __str__(self):
        return '{0}({1}, {2})'.format(
            type(self).__name__, self.initial, self.goal)

class Node:
    "A Node in a search tree."
    def __init__(self, state, parent=None, action=None, path_cost=0):
        self.__dict__.update(state=state, parent=parent, action=action, path_cost=path_cost)

    def __str__(self): 
        return '<{0}>'.format(self.state)
    def __len__(self): 
        return 0 if self.parent is None else (1 + len(self.parent))
    def __lt__(self, other): 
        return self.path_cost < other.path_cost

failure = Node('failure', path_cost=math.inf) # Indicates an algorithm couldn't find a solution.
cutoff  = Node('cutoff',  path_cost=math.inf) # Indicates iterative deepening search was cut off.

def expand(problem, node):
    "Expand a node, generating the children nodes."
    s = node.state
    for action in problem.actions(s):
        s1 = problem.result(s, action)
        cost = node.path_cost + problem.action_cost(s, action, s1)
        yield Node(s1, node, action, cost)
        

def path_actions(node):
    "The sequence of actions to get to this node."
    if node.parent is None:
        return []  
    return path_actions(node.parent) + [node.action]


def path_states(node):
    "The sequence of states to get to this node."
    if node in (cutoff, failure, None): 
        return []
    return path_states(node.parent) + [node.state]

class PriorityQueue:
    """A queue in which the item with minimum f(item) is always popped first."""

    def __init__(self, items=(), key=lambda x: x): 
        self.key = key
        self.items = [] # a heap of (score, item) pairs
        for item in items:
            self.add(item)
         
    def add(self, item):
        """Add item to the queuez."""
        pair = (self.key(item), item)
        heapq.heappush(self.items, pair)

    def pop(self):
        """Pop and return the item with min f(item) value."""
        return heapq.heappop(self.items)[1]
    
    def top(self): return self.items[0][1]

    def __len__(self): return len(self.items)

def best_first_search(problem, f):
    "Search nodes with minimum f(node) value first."
    node = Node(problem.initial)
    frontier = PriorityQueue([node], key=f)
    reached = {problem.initial: node}
    while frontier:
        node=frontier.pop()
        if problem.is_goal(node.state):
            return node
        for child in expand(problem,node):
            s=child.state
            if s not in reached or child.path_cost<reached[s].path_cost:
                reached[s]=child
                frontier.add(child)
    return failure

def g(n):
    return n.path_cost
    cost = 1
    return cost

class RouteProblem(Problem):
    """A problem to find a route between locations on a `Map`.
    Create a problem with RouteProblem(start, goal, map=Map(...)}).
    States are the vertexes in the Map graph; actions are destination states."""
    
    def actions(self, state): 
        """The places neighboring `state`."""
        return self.map.neighbors[state]
    
    def result(self, state, action):
        """Go to the `action` place, if the map says that is possible."""
        return action if action in self.map.neighbors[state] else state
    
    def action_cost(self, s, action, s1):
        """The distance (cost) to go from s to s1."""
        return self.map.distances[s, s1]
    
    def h(self, node):
        "Straight-line distance between state and the goal."
        locs = self.map.locations
        return straight_line_distance(locs[node.state], locs[self.goal])

class Map:
    """A map of places in a 2D world: a graph with vertexes and links between them. 
    In `Map(links, locations)`, `links` can be either [(v1, v2)...] pairs, 
    or a {(v1, v2): distance...} dict. Optional `locations` can be {v1: (x, y)} 
    If `directed=False` then for every (v1, v2) link, we add a (v2, v1) link."""

    def __init__(self, links, locations=None, directed=False):
        if not hasattr(links, 'items'): # Distances are 1 by default
            links = {link: 1 for link in links}
        if not directed:
            for (v1, v2) in list(links):
                links[v2, v1] = links[v1, v2]
        self.distances = links
        self.neighbors = multimap(links)
        self.locations = locations or defaultdict(lambda: (0, 0))

        
def multimap(pairs) -> dict:
    "Given (key, val) pairs, make a dict of {key: [val,...]}."
    result = defaultdict(list)
    for key, val in pairs:
        result[key].append(val)
    return result

nearby_locations = Map(
    {('Kottar', 'Asaripallam'):  7.5, ('Kottar', 'Mandakadal'):  10.5, ('Kottar', 'Erachakulam'):  10.8, ('Kottar', 'Ganapathipuram'):  12.5,
('Kottar', 'Mylaudy'):  7.7, ('Kottar', 'Thengamputhur'):  7, ('Mandakadal', 'Asaripallam'):  6.6, ('Mandakadal', 'Madavilagum'):  4.1,
('Mandakadal', 'Kumarakoil'):  7, ('Thuckalay', 'Kumarakoil'):  4.8, ('Thuckalay', 'Allieamandaram'):  3.4, ('Thuckalay', 'Madavilagum'):  5.9, 
('Karungal', 'Madavilagum'):  11.3, ('Niruogudi', 'Madavilagum'):  11.5, ('Mandaikadu', 'Madavilagum'):  8.5,
('Karungal', 'Allieamandaram'):  10, ('Karungal', 'Niruogudi'):  3.7, ('Colachel', 'Niruogudi'):  4.7, ('Colachel', 'Mandaikadu'): 4.7, 
('Ganapathipuram', 'Mandaikadu'):  8.8, ('Ganapathipuram', 'Thengaputhur'):  15, ('Keelanmanipudi', 'Thengaputhur'):  4.3,
('Keelanmanipudi', 'Kanyakumari'):  7.3, ('Vattakottai', 'Kanyakumari'):  6.9, ('Vattakottai', 'Mylaudy'):  8.6,
('Vattakottai', 'Kavalkinaru'):  20.9, ('Aralvaimozhi', 'Kavalkinaru'):  7.6 ,('Aralvaimozhi', 'Boothapandi'):  10.7,('Erachakulam', 'Boothapandi'):  7.1,
 ('Erachakulam', 'Kottar'):  10.8})


r0 = RouteProblem('Vattakottai', 'Karungal', map=nearby_locations)
r1 = RouteProblem('Karungal', 'Colachel', map=nearby_locations)
r2 = RouteProblem('Aralvaimozhi', 'Karungal', map=nearby_locations)
r3 = RouteProblem('Asaripallam', 'Kanyakumari', map=nearby_locations)
r4 = RouteProblem('Asaripallam', 'Kavalkinaru', map=nearby_locations)

goal_state_path_0=best_first_search(r0,g)
goal_state_path_1=best_first_search(r1,g)
goal_state_path_2=best_first_search(r2,g)
goal_state_path_3=best_first_search(r3,g)
goal_state_path_4=best_first_search(r4,g)

print("GoalStateWithPath:{0}".format(goal_state_path_0))
path_states(goal_state_path_0)
print("Total Distance={0} Kilometers".format(goal_state_path_0.path_cost))

print("GoalStateWithPath:{0}".format(goal_state_path_1))
path_states(goal_state_path_1)
print("Total Distance={0} Kilometers".format(goal_state_path_1.path_cost))

print("GoalStateWithPath:{0}".format(goal_state_path_2))
path_states(goal_state_path_2)
print("Total Distance={0} Kilometers".format(goal_state_path_2.path_cost))

print("GoalStateWithPath:{0}".format(goal_state_path_3))
path_states(goal_state_path_3)
print("Total Distance={0} Kilometers".format(goal_state_path_3.path_cost))

print("GoalStateWithPath:{0}".format(goal_state_path_4))
path_states(goal_state_path_4)
print("Total Distance={0} Kilometers".format(goal_state_path_4.path_cost))
```


## OUTPUT:

![1](https://user-images.githubusercontent.com/75235150/167853839-6343a281-3ef7-4114-914b-cb8951b0f164.jpg)

![2](https://user-images.githubusercontent.com/75235150/167853860-3ec2a5f4-886f-4919-ab74-c4c6246a410f.jpg)

![3](https://user-images.githubusercontent.com/75235150/167853885-8eed421b-756c-4f7a-9bdf-165dfa0c734a.jpg)

![4](https://user-images.githubusercontent.com/75235150/167853905-b388f842-d127-43a8-95fc-64f3f786f3ee.jpg)

![5](https://user-images.githubusercontent.com/75235150/167853920-7eb8ce66-f42e-4e05-a614-6191169e0d4a.jpg)

![6](https://user-images.githubusercontent.com/75235150/167853936-ceb13fad-69d7-42d5-9b86-ea7b86ebab61.jpg)

![7](https://user-images.githubusercontent.com/75235150/167853952-f6c54f6b-7227-4eff-83c0-00a2f8c9f5ec.jpg)

![8](https://user-images.githubusercontent.com/75235150/167853969-2b52ecae-47d4-4f2a-ad5e-b71558c2d1d1.jpg)

![9](https://user-images.githubusercontent.com/75235150/167853981-9d720803-1963-4016-9b48-3e0354b7eed8.jpg)

![10](https://user-images.githubusercontent.com/75235150/167853993-39ecaf49-bb21-4661-b4ad-1170481bd42c.jpg)

![11](https://user-images.githubusercontent.com/75235150/167854028-dbb11833-5824-4a83-ac21-bdbcdcda96e9.jpg)

![12](https://user-images.githubusercontent.com/75235150/167854044-094ed8d9-36f1-4934-be68-9af9f6798e0d.jpg)

![13](https://user-images.githubusercontent.com/75235150/167854066-cac95872-06f3-4a80-9fda-658aab1d13a0.jpg)

![14](https://user-images.githubusercontent.com/75235150/167854084-09069ff7-8704-4625-ba37-d6ebbb5026aa.jpg)

![15](https://user-images.githubusercontent.com/75235150/167854092-600c620e-8c22-4067-9ca9-f92a18f9590f.jpg)


## SOLUTION JUSTIFICATION:
Once the algorithm has been carried out you can find the least weight path to all permanently labelled nodes. We don’t need a new diagram for each pass.

## RESULT:

Hence, Dijkstra's shortest path algorithm was implemented for a route finding problem.
