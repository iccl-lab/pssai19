#!/usr/bin/env python
import argparse as ap
import math
import heapq as hq
import io


def load_maze_file(maze_filename):
    '''
    Simply read in the maze as 2-dim array.
    :param maze_filename:
    :return: a two-dim array containing the same characters as in the maze file.
    '''
    maze = []

    with io.open(maze_filename, 'r') as mazefile:
        for row in mazefile:
            if not row.startswith("#"):
                maze.append(row.strip().split(sep=','))

    return maze


def euclidian_distance(x, y):
    return math.sqrt((y[0]-x[0])**2 + (y[1]-x[1])**2)


def manhatten_distance(x, y):
    return math.fabs(y[0]-x[0]) + math.fabs(y[1]-x[1])


def neighbors(node, maze):
    '''
    Considering walls and the outer frame, returns the neighbors of given corrdinate.
    :param node:
    :param maze:
    :return: list of neighbor coordinate pairs
    '''
    (x, y) = node
    neighbor_pos = []

    if x > 0 and maze[x-1][y] != 'W':
        neighbor_pos.append((x-1, y))
    if y > 0 and maze[x][y-1] != 'W':
        neighbor_pos.append((x, y-1))
    if 0 <= x < len(maze)-1 and maze[x+1][y] != 'W':
        neighbor_pos.append((x+1, y))
    if 0 <= y < len(maze)-1 and maze[x][y+1] != 'W':
        neighbor_pos.append((x, y+1))

    return neighbor_pos


def reconstruct_path(origin, current):
    path = []

    while current in origin.keys():
        current = origin[current]
        path.append(current)

    return path


def astar_search(maze, heuristic=manhatten_distance):
    '''

    :param maze:
    :param heuristic: a function that takes two corrdinate pairs an returns an approx. distance
    :return: shortest path found ...list of coord. pairs
    '''
    prio_queue    = [] # priority queue of nodes to expand ...contains pairs (d, (x,y)) with d = f((x,y))
    visited = [] # list of coordinates (x,y) already visited
    origin  = {} # final shortest path
    g_score = {} # g(X) = the actual cost for reaching the node X
    f_score = {} # f(X) = g(X) + h(X) , the value on which we choose the next node to expand

    # mouse and cheese positions ;)
    GOAL_X  = 0
    GOAL_Y  = len(maze[0])-1
    START_X = len(maze[0])-1
    START_Y = 0
    GOAL    = (GOAL_X, GOAL_Y)
    START   = (START_X, START_Y)

    print("h(START, GOAL) = " + str(heuristic(START, GOAL)))

    # init g and f scores for START
    g_score[START] = 0
    f_score[START] = heuristic(START, GOAL)

    # add the start node with pure heuristic distance to prio_queue node list
    prio_queue.append((f_score[START], START))

    while prio_queue:
        # get the node with least f-score
        (f_current, current) = hq.heappop(prio_queue)

        print("Next node is " + str(current))

        if current == GOAL:
            print("Optimal path found ...")
            print("Cost: " + str(g_score[current]))
            return reconstruct_path(origin, current)

        visited.append(current)

        for neighbor in neighbors(current, maze):
            if neighbor in visited:
                continue

            # replacing 1 by manhatten_distance would also work ...but not needed here
            new_g_val = g_score[current] + 1

            # this try-catch is also the test whether neighbor
            # is already in "prio_queue"
            try:
                if new_g_val < g_score[neighbor]:
                    print("found better g value for neighbor " + str(neighbor))
                    prio_queue.remove((g_score[neighbor], neighbor))
            except KeyError:
                print(str(neighbor) + " is new...")

            g_score[neighbor] = new_g_val
            origin[neighbor] = current
            f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, GOAL)
            hq.heappush(prio_queue, (f_score[neighbor], neighbor))

    print("No path found? ...something is wrong here.")
    return []


def main():
    parser = ap.ArgumentParser(description="A Maze Solver based on AStar.")
    parser.add_argument("mazefile", type=str, help="filename of the the maze file to load")
    args = parser.parse_args()

    # 1. load the maze
    maze = load_maze_file(args.mazefile)

    # 2. call AStar with heuristic=euclidian_distance ...replace with manhatten_distance to see the effects ;)
    shortest_path = astar_search(maze, heuristic=euclidian_distance)

    print("Path in reverse order:")
    print(shortest_path)


if __name__ == "__main__":
    main()
