from irsim.util.util import time_it
import irsim

@time_it("test_all_objects")
def test_all_objects():

    env1 = irsim.make("test_collision_avoidance.yaml", save_ani=False, full=False, display=False)

    for i in range(50):
        env1.step()
        env1.render(0.01)

        env1.draw_trajectory(env1.robot.trajectory, show_direction=True)
        
        if env1.done():
            print('done')

    env1.end()
    
    env2 = irsim.make('test_all_objects.yaml', display=False)

    env2.robot.get_lidar_points()
    env2.robot.get_lidar_scan()
    env2.robot.get_lidar_offset()

    for i in range(2):
        env2.step()
        env2.render(0.01)
    env2.end()

    env3 = irsim.make('test_render.yaml', save_ani=True, display=False)

    for i in range(3):
        env3.step()
        env3.render(0.01)
    env3.end()

    env4 = irsim.make('test_collision_world.yaml', save_ani=False, display=False)

    for i in range(4):
        env4.step()
        env4.render(0.01)
    env4.end()

    env5 = irsim.make('test_multi_objects_world.yaml', save_ani=False, display=False)

    env5.random_obstacle_position()

    for i in range(5):
        env5.step()
        env5.render(0.01)
    env5.end()

    env6 = irsim.make('test_grid_map.yaml', save_ani=False, display=False)

    for i in range(6):
        env6.step()
        env6.render(0.01)
    env6.end()

if __name__ == "__main__":
    test_all_objects()
