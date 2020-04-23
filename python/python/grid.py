import numpy as np
from utils import get_movements_4n, get_movements_8n
from typing import Dict, List

OBSTACLE = 255
UNOCCUPIED = 0


class OccupancyGridMap:
    def __init__(self, x_dim, y_dim, exploration_setting='8N'):
        """
        set initial values for the map occupancy grid
        |----------> y, column
        |           (x=0,y=2)
        |
        V (x=2, y=0)
        x, row
        :param x_dim: dimension in the x direction
        :param y_dim: dimension in the y direction
        """
        self.x_dim = x_dim
        self.y_dim = y_dim

        # the map extents in units [m]
        self.map_extents = (x_dim, y_dim)

        # the obstacle map
        self.occupancy_grid_map = np.zeros(self.map_extents, dtype=np.uint8)

        # obstacles
        self.visited = {}
        self.exploration_setting = exploration_setting

    def get_map(self):
        """
        :return: return the current occupancy grid map
        """
        return self.occupancy_grid_map

    def set_map(self, new_ogrid):
        """
        :param new_ogrid:
        :return:
        """
        self.occupancy_grid_map = new_ogrid

    def is_unoccupied(self, pos: (int,int)) -> bool:
        """
        :param pos: cell position we wish to check
        :return: True if cell is occupied with obstacle, False else
        """
        (x, y) = (round(pos[0]), round(pos[1]))  # make sure pos is int
        (row, col) = (x, y)

        # if not self.in_bounds(cell=(x, y)):
        #    raise IndexError("Map index out of bounds")

        return self.occupancy_grid_map[row][col] == UNOCCUPIED

    def in_bounds(self, cell: (int, int)) -> bool:
        """
        Checks if the provided coordinates are within
        the bounds of the grid map
        :param cell: cell position (x,y)
        :return: True if within bounds, False else
        """
        (x, y) = cell
        return 0 <= x < self.x_dim and 0 <= y < self.y_dim

    def filter_bounds(self, neighbors: List):
        filtered = [node for node in neighbors if self.in_bounds(node)]
        return filtered

    def succ(self, vertex: (int, int)) -> list:
        """
        :param vertex:
        :return:
        """
        (x, y) = vertex

        if self.exploration_setting == '4N':  # change this
            movements = get_movements_4n(x=x, y=y)
        else:
            movements = get_movements_8n(x=x, y=y)

        if (x + y) % 2 == 0: movements.reverse()

        filtered_movements = self.filter_bounds(neighbors=movements)
        return list(filtered_movements)

    def set_obstacle(self, pos: (int, int)):
        """
        :param pos: cell position we wish to set obstacle
        :return: None
        """
        (x, y) = (round(pos[0]), round(pos[1]))  # make sure pos is int
        (row, col) = (x, y)
        self.occupancy_grid_map[row, col] = OBSTACLE

    def remove_obstacle(self, pos: (int, int)):
        """
        :param pos:
        :return:
        """
        (x, y) = (round(pos[0]), round(pos[1]))  # make sure pos is int
        (row, col) = (x, y)
        self.occupancy_grid_map[row, col] = UNOCCUPIED

    def update_global_from_local_grid(self, local_grid: Dict) -> List:
        changed_costs = []
        for node, value in local_grid.items():
            if value == OBSTACLE:
                if self.is_unoccupied(node):
                    # if not obstacle before, but is now
                    changed_costs.append(node)
                    # add the obstacle
                    self.set_obstacle(node)
            else:
                if not self.is_unoccupied(node):
                    # if obstacle before, but not now
                    changed_costs.append(node)
                    # remove the obstacle
                    self.remove_obstacle(node)
        return changed_costs

    def local_observation(self, global_position: (int, int), view_range: int = 2) -> Dict:
        """
        :param global_position:
        :param view_range:
        :return:obs
        """
        (px, py) = global_position
        nodes = [(x, y) for x in range(px - view_range, px + view_range + 1)
                 for y in range(py - view_range, py + view_range + 1)
                 if self.in_bounds((x, y))]
        return {node: UNOCCUPIED if self.is_unoccupied(pos=node) else OBSTACLE for node in nodes}