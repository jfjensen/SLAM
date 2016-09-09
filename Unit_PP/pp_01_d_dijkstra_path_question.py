# Dijkstra path planning.
# Fourth version: Reconstruct the path from start to goal.
# pp_01_d_dijkstra_path
# (c) Claus Brenner, 17 JAN 2014
from heapq import heappush, heappop
import numpy as np
import traceback
import gui
import common

# The world extents in units.
world_extents = (200, 150)

# The obstacle map.
# Obstacle = 255, free space = 0.
world_obstacles = np.zeros(world_extents, dtype=np.uint8)

# The array of visited cells during search.
visited_nodes = None

# The optimal path between start and goal. This is a list of (x,y) pairs.
optimal_path = []

# Functions for GUI functionality.
def add_obstacle(pos):
    common.set_obstacle(world_obstacles, pos, True)
    common.draw_background(gui, world_obstacles, visited_nodes, optimal_path)
def remove_obstacle(pos):
    common.set_obstacle(world_obstacles, pos, False)
    common.draw_background(gui, world_obstacles, visited_nodes, optimal_path)
def clear_obstacles():
    global world_obstacles
    world_obstacles = np.zeros(world_extents, dtype=np.uint8)
    update_callback()
def update_callback(pos = None):
    # Call path planning algorithm.
    start, goal = gui.get_start_goal()
    if not (start==None or goal==None):
        global optimal_path
        global visited_nodes
        try:
            optimal_path, visited_nodes = dijkstra(start, goal, world_obstacles)
        except Exception, e:
            print traceback.print_exc()
    # Draw new background.
    common.draw_background(gui, world_obstacles, visited_nodes, optimal_path)

# --------------------------------------------------------------------------
# Dijkstra algorithm.
# --------------------------------------------------------------------------

# Allowed movements and costs on the grid.
# Each tuple is: (movement_x, movement_y, cost).
s2 = np.sqrt(2)
movements = [ # Direct neighbors (4N).
              (1,0, 1.), (0,1, 1.), (-1,0, 1.), (0,-1, 1.),
              # Diagonal neighbors.
              # Comment this out to play with 4N only (faster).
              (1,1, s2), (-1,1, s2), (-1,-1, s2), (1,-1, s2),
            ]

def dijkstra(start, goal, obstacles):
    """Dijkstra's algorithm. Fourth version also returns optimal path."""
    # In the beginning, the start is the only element in our front.
    # The first element is the cost of the path from the start to the point.
    # The second element is the position (cell) of the point.
    # The third component is the position we came from when entering the tuple
    #   to the front.
    front = [ (0.001, start) ]  # CHANGE 01_d: Add None to this tuple.

    # In the beginning, no cell has been visited.
    extents = obstacles.shape
    visited = np.zeros(extents, dtype=np.float32)

    # Also, we use a dictionary to remember where we came from.
    came_from = {}  # CHANGE 01_d: Add this line to your implementation.

    # While there are elements to investigate in our front.
    while front:
        # Get smallest item and remove from front.
 
        # Check if this has been visited already.
        cost, pos, previous = element  # CHANGE 01_d: add 'previous' (as shown).

        # Now it is visited. Mark with cost.

        # Also remember that we came from previous when we marked pos.
        # CHANGE 01_d: enter 'previous' (value) into the 'came_from' dictionary
        #   at index (key) 'pos'.

        # Check if the goal has been reached.
        if pos == goal:
            break  # Finished!

        # Check all neighbors.
        for dx, dy, deltacost in movements:
            # Determine new position and check bounds.

            # Add to front if: not visited before and no obstacle.
            new_pos = (new_x, new_y)
            # CHANGE 01_d: When push'ing the new tuple to the heap, make
            #   sure it is a 3-tuple, with the last component of the tuple
            #   being the position 'we came from' (which is 'pos').


    # CHANGE 01_d: Make sure to include the following code, which 'unwinds'
    #   the path from goal to start, using the came_from dictionary.
    #   Make sure to include the (modified!) return statement, otherwise
    #   the path will not show up in the visualization.

    # Reconstruct path, starting from goal.
    path = []
    if pos == goal:  # If we reached the goal, unwind backwards.
        while pos:
            path.append(pos)
            pos = came_from[pos]
        path.reverse()  # Reverse so that path is from start to goal.

    return (path, visited)


# Main program.
if __name__ == '__main__':
    # Link functions to buttons.
    callbacks = {"update": update_callback,
                 "button_1_press": add_obstacle,
                 "button_1_drag": add_obstacle,
                 "button_1_release": update_callback,
                 "button_2_press": remove_obstacle,
                 "button_2_drag": remove_obstacle,
                 "button_2_release": update_callback,
                 "button_3_press": remove_obstacle,
                 "button_3_drag": remove_obstacle,
                 "button_3_release": update_callback,
                 }
    # Extra buttons.
    buttons = [("Clear", clear_obstacles)]

    # Init GUI.
    gui = gui.GUI(world_extents, 4, callbacks,
                  buttons, "on",
                  "Simple Dijkstra Algorithm (finally shows the optimal path "
                  "from start to goal).")

    # Start GUI main loop.
    gui.run()
