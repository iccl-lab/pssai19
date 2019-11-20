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


def manhatten_distance(p1, p2):
    return math.fabs(p2[0]-p1[0]) + math.fabs(p2[1]-p1[1])


def neighbors(node, maze):
    '''
    Considering walls and the outer frame, returns the possible neighbor nodes of a given corrdinate.
    :param node: the node as (x,y) tuple/coordinate for which possible neighbors shall be determined
    :param maze: a two-dim array containing the characters as in the maze file.
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


def reconstruct_path(origins, node):
    '''
    Starting from node, reconstruct the chosen path 
    :param origins: a dict that stores for each node, the node from which it has been reached
    :param nod: the node from which the path shall be reconstructed
    '''
    path = [node]

    while node in origins.keys():
        node = origins[node]
        path.append(node)

    path.reverse()

    return path


def astar_search(maze, heuristic=manhatten_distance):
    '''
    :param maze: a two-dim array containing the characters as in the maze file.
    :param heuristic: a function that takes two corrdinate pairs an returns an approx. distance
    :return: shortest path found ...list of coord. pairs
    '''
    prio_queue    = [] # priority queue of nodes to expand ...contains pairs (d, (x,y)) with d = f((x,y))
    visited = [] # list of coordinates (x,y) already visited
    origin  = {} # look-up table: key (x,y) -> origin (x',y') "from which (x',y') was (x,y)"
    g_scores = {} # g(X) = the actual cost for reaching the node X
    f_scores = {} # f(X) = g(X) + h(X) , the value on which we choose the next node to expand

    # mouse and cheese positions ;)
    GOAL_X  = 0
    GOAL_Y  = len(maze[0])-1
    START_X = len(maze[0])-1
    START_Y = 0
    GOAL    = (GOAL_X, GOAL_Y)
    START   = (START_X, START_Y)

    # for convenience
    def update_heuristic(node, g_score):
        g_scores[node] = g_score
        f_scores[node] = g_scores[node] + heuristic(node, GOAL)

    print("h(START, GOAL) = " + str(heuristic(START, GOAL)))

    # init g and f scores for START
    g_scores[START] = 0
    f_scores[START] = heuristic(START, GOAL)

    # add the start node with pure heuristic distance to prio_queue node list
    prio_queue.append((f_scores[START], START))

    # we keep exploring nodes until the priority queue is empty,
    # OR we found the GOAL
    while prio_queue:
        # pop the node with least f-score
        (f_current, current) = hq.heappop(prio_queue)

        print("Next node is " + str(current) + " with f-score " + str(f_current))

        # stop and reconstruct the path if we reached the goal
        if current == GOAL:
            print("Optimal path found ...")
            print("Cost: " + str(g_scores[current]))
            return reconstruct_path(origin, GOAL)

        visited.append(current)

        for neighbor in neighbors(current, maze):
            if neighbor in visited:
                continue

            # replacing 1 by manhatten_distance would also work ...but not needed here
            g_neighbor = g_scores[current] + 1

            # test whether we have seen that neighbor already and know its g-score
            if neighbor in g_scores:
                if g_neighbor < g_scores[neighbor]:
                    print("found better g value for neighbor " + str(neighbor))
                    # remove neighbor from the prio queue and update f-score
                    prio_queue.remove((f_scores[neighbor], neighbor))
                    update_heuristic(neighbor, g_neighbor)
                    # update the origin
                    origin[neighbor] = current
                    # add again with updated f-score
                    hq.heappush(prio_queue, (f_scores[neighbor], neighbor))
                else:
                    print(str(neighbor) + " has been seen already with better g-score, skipping " + str(neighbor))
                    pass
            else:
                # new neighbor, compute g-score, f-score and record the where we came from
                print(str(neighbor) + " is new.")
                update_heuristic(neighbor, g_neighbor)
                origin[neighbor] = current
                hq.heappush(prio_queue, (f_scores[neighbor], neighbor))
            
    print("No path found? ...something is wrong here.")
    return []


def main():
    parser = ap.ArgumentParser(description="A Maze Solver based on AStar.")
    parser.add_argument("mazefile", type=str, help="filename of the the maze file to load")
    args = parser.parse_args()

    # 1. load the maze
    maze = load_maze_file(args.mazefile)

    # 2. call AStar with heuristic=euclidian_distance ...replace with manhatten_distance to see the effects ;)
    shortest_path = astar_search(maze, heuristic=manhatten_distance)

    print("Path:")
    print(shortest_path)

    print_maze(maze, shortest_path)

def print_maze(maze, path=None):
    out = ""
    for i in range(0, len(maze)):
        for j in range(0, len(maze[i])):
            if path:
                if (i,j) in path:
                    out += " X "
                else:
                    out += " " + str(maze[i][j]) + " "
            else:
                out += " " + str(maze[i][j]) + " "
        out += "\n"
    print(out)

if __name__ == "__main__":
    main()
