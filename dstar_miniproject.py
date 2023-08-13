import math
from sys import maxsize
import matplotlib.pyplot as plt
import cv2
import numpy as np

show_animation = False

class State:

    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.state = "."
        self.t = "new"  # tag for state
        self.h = 0
        self.k = 0

    def cost(self, state):
        if self.state == "#" or state.state == "#":
            return maxsize

        return math.sqrt(math.pow((self.x - state.x), 2) +
                         math.pow((self.y - state.y), 2))

    def set_state(self, state):
        """
        .: new
        #: obstacle
        e: oparent of current state
        *: closed state
        s: current state
        """
        if state not in ["s", ".", "#", "e", "*"]:
            return
        self.state = state


class Map:

    def __init__(self, row, col):
        self.row = row
        self.col = col
        self.map = self.init_map()

    def init_map(self):
        map_list = []
        for i in range(self.row):
            tmp = []
            for j in range(self.col):
                tmp.append(State(i, j))
            map_list.append(tmp)
        return map_list

    def get_neighbors(self, state):
        state_list = []
        for i in [-1, 0, 1]:
            for j in [-1, 0, 1]:
                if i == 0 and j == 0:
                    continue
                if state.x + i < 0 or state.x + i >= self.row:
                    continue
                if state.y + j < 0 or state.y + j >= self.col:
                    continue
                state_list.append(self.map[state.x + i][state.y + j])
        return state_list

    def set_obstacle(self, point_list):
        for x, y in point_list:
            if x < 0 or x >= self.row or y < 0 or y >= self.col:
                continue

            self.map[x][y].set_state("#")


class Dstar:
    def __init__(self, maps):
        self.map = maps
        self.open_list = set()

    def process_state(self):
        x = self.min_state()

        if x is None:
            return -1

        k_old = self.get_kmin()
        self.remove(x)

        if k_old < x.h:
            for y in self.map.get_neighbors(x):
                if y.h <= k_old and x.h > y.h + x.cost(y):
                    x.parent = y
                    x.h = y.h + x.cost(y)
        elif k_old == x.h:
            for y in self.map.get_neighbors(x):
                if y.t == "new" or y.parent == x and y.h != x.h + x.cost(y) \
                        or y.parent != x and y.h > x.h + x.cost(y):
                    y.parent = x
                    self.insert(y, x.h + x.cost(y))
        else:
            for y in self.map.get_neighbors(x):
                if y.t == "new" or y.parent == x and y.h != x.h + x.cost(y):
                    y.parent = x
                    self.insert(y, x.h + x.cost(y))
                else:
                    if y.parent != x and y.h > x.h + x.cost(y):
                        self.insert(y, x.h)
                    else:
                        if y.parent != x and x.h > y.h + x.cost(y) \
                                and y.t == "close" and y.h > k_old:
                            self.insert(y, y.h)
        return self.get_kmin()

    def min_state(self):
        if not self.open_list:
            return None
        min_state = min(self.open_list, key=lambda x: x.k)
        return min_state

    def get_kmin(self):
        if not self.open_list:
            return -1
        k_min = min([x.k for x in self.open_list])
        return k_min

    def insert(self, state, h_new):
        if state.t == "new":
            state.k = h_new
        elif state.t == "open":
            state.k = min(state.k, h_new)
        elif state.t == "close":
            state.k = min(state.h, h_new)
        state.h = h_new
        state.t = "open"
        self.open_list.add(state)

    def remove(self, state):
        if state.t == "open":
            state.t = "close"
        self.open_list.remove(state)

    def modify_cost(self, x):
        if x.t == "close":
            self.insert(x, x.parent.h + x.cost(x.parent))

    def run(self, start, end):

        self.rx = []
        self.ry = []

        self.insert(end, 0.0)

        while True:
            self.process_state()
            if start.t == "close":
                break

        start.set_state("s")
        s = start
        s = s.parent
        s.set_state("e")
        tmp = start

        while tmp != end:
            tmp.set_state("*")
            self.rx.append(tmp.x)
            self.ry.append(tmp.y)
            if show_animation:
                plt.plot(self.rx, self.ry, "-r")
                plt.pause(0.01)
            if tmp.parent.state == "#":
                self.modify(tmp)
                continue
            tmp = tmp.parent
        tmp.set_state("e")

        return self.rx, self.ry

    def modify(self, state):
        self.modify_cost(state)
        while True:
            k_min = self.process_state()
            if k_min >= state.h:
                break
    def get_final_path(self):
        return self.rx, self.ry 

def create_obstacle_gridmap(image_path, obstacle_value, free_value):
    img1 = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)  # Read in grayscale
    # Vertically flip the image to ensure that the obstacles positions are extraced from the image accurately
    img = cv2.flip(img1, 0)
    height, width = img.shape
    
    # Create an empty grid map
    obstacle_map = [[0 for _ in range(width)] for _ in range(height)]
    
    # Process the image to create the obstacle grid map
    for y in range(height):
        for x in range(width):
            pixel_value = img[y, x]
            if pixel_value == 0:  # Black pixel (obstacle)
                obstacle_map[y][x] = obstacle_value
            else:  # White pixel (free space)
                obstacle_map[y][x] = free_value
    
    obstacle_positions = []
    for y in range(height):
        for x in range(width):
            if obstacle_map[y][x] == 1:
                obstacle_positions.append((x, y))
    return height, width, obstacle_positions
    
def main():
    image_path = 'map.png'
    obstacle_value=1 
    free_value=0
    height, width, obstacle_positions = create_obstacle_gridmap(image_path, obstacle_value, free_value)
    m = Map(height, width)
    m.set_obstacle(obstacle_positions)            
    start = [10, 10] #set the starting point
    goal = [90, 90] # set the goal point
    
    if show_animation:
        for x, y in obstacle_positions:
            plt.plot(x, y, ".k")
        plt.plot(start[0], start[1], "og")
        plt.plot(goal[0], goal[1], "xb")
        plt.axis("equal")
        plt.show()

    start = m.map[start[0]][start[1]]
    end = m.map[goal[0]][goal[1]]
    dstar = Dstar(m)
    rx, ry = dstar.run(start, end)

    if show_animation:
        plt.plot(rx, ry, "-r")
        plt.show()
    
    final_rx, final_ry = dstar.get_final_path()

    print("Start Point:", start.x, start.y)
    print("End Point:", end.x, end.y)
    for x, y in obstacle_positions:
        plt.plot(x, y, ".k")
    plt.plot(start.x, start.y, "og")
    plt.plot(end.x, end.y, "xb")
    plt.plot(final_rx, final_ry, "-r")  # Plot the final path
        
    plt.title('D* Path Planning')
    plt.axis("equal")
    plt.show()


if __name__ == '__main__':
    main()