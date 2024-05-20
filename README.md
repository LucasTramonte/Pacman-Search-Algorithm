# Pacman-Search-Algorithm
Build general search algorithms and apply them to Pacman scenarios

https://centralesupelec.edunao.com/pluginfile.php/423959/course/section/60097/p2-search.html?time=1714677059278

http://ai.berkeley.edu.

## Contents
- [Question1](#Question1)
- [Question2](#Question2)
- [Question3](#Question3)
- [Question4](#Question4)
- [Question5](#Question5)
- [Question6](#Question6)
- [Question7](#Question7)
- [Question8](#Question8)

## Question1

Finding a Fixed Food Dot using Depth First Search

**File:** `search.py`  
**Function:** `depthFirstSearch(problem)`

To run DFS:
```bash
python pacman.py -l tinyMaze -p SearchAgent -a fn=dfs
```

## Question2

Breadth First Search

**File:** search.py
**Function:** breadthFirstSearch(problem)

To run BFS:

```bash
python pacman.py -l mediumMaze -p SearchAgent -a fn=bfs
```

## Question3

Varying the Cost Function:

**File:** search.py
**Function:** uniformCostSearch(problem)

To run UCS:

```bash
python pacman.py -l mediumDottedMaze -p StayEastSearchAgent
```

## Question4

A* search:

**File:** search.py
**Function:** aStarSearch(problem, heuristic=nullHeuristic)

To run A* with Manhattan Heuristic:

```bash
python pacman.py -l bigMaze -z .5 -p SearchAgent -a fn=astar,heuristic=manhattanHeuristic
```

## Question5
Finding All the Corners

**File:** searchAgents.py
**Function:** CornersProblem

To run the Corner Problem:

```bash
python pacman.py -l mediumCorners -p SearchAgent -a fn=bfs,prob=CornersProblem
```

## Question6
Corners Problem: Heuristic

**File:** searchAgents.py
**Function:** cornersHeuristic(state, problem)

## Question7
Eating All The Dots

**File:** searchAgents.py
**Function:** foodHeuristic(state, problem)

```bash
python pacman.py -l trickySearch -p AStarFoodSearchAgent
```

## Question8
Suboptimal Search

**File:** searchAgents.py
**Function:** findPathToClosestDot(gameState)

```bash
python pacman.py -l bigSearch -p ClosestDotSearchAgent -z .5
```