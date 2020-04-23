from gui import *
#from dstar_lite import *
from grid import *

OBSTACLE = 255
UNOCCUPIED = 0

if __name__ == '__main__':

    """
    set initial values for the map occupancy grid
    |----------> y, column
    |           (x=0,y=2)
    |
    V (x=2, y=0)
    x, row
    """
    x_dim = 100
    y_dim = 80
    start = (10, 10)
    goal = (20, 70)

    gui = Animation(title="D* Lite Path Planning",
                    width=10,
                    height=10,
                    margin=0,
                    x_dim=x_dim,
                    y_dim=y_dim,
                    start=start,
                    goal=goal,
                    viewing_range=5)

    ground_truth_world = gui.world

    new_position = start
    last_position = start
    new_observation = None
    type = OBSTACLE

    #dstar = DstarLite(world=ground_truth_world,
    #                  s_start=start,
    #                  s_goal=goal,
    #                  view_range=5)

    queue = []
    #path = [p for p, o in dstar.move_and_rescan(position=new_position)]

    while not gui.done:
        # update the map
        # print(path)
        # drive gui
        gui.run_game()

        #new_position = gui.current
        new_observation = gui.observation

        if new_observation is not None:
            if new_observation["type"] == OBSTACLE:
                pass
                #dstar.gt_global_map.set_obstacle(pos=new_observation["pos"])
            if new_observation["pos"] == UNOCCUPIED:
                print("else {}".format(new_observation["pos"]))
                #dstar.gt_global_map.remove_obstacle(pos=new_observation["pos"])

        if gui.current != last_position:
            last_position = new_position
            #path = [p for p, o in dstar.move_and_rescan(position=gui.current)]