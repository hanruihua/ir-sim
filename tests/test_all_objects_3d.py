from irsim.util.util import time_it
from unittest.mock import Mock
import irsim
from pynput import keyboard
import matplotlib.pyplot as plt

plt.close('all')


@time_it("test_all_objects")
def test_all_objects():
    env1 = irsim.make("test_collision_avoidance.yaml", save_ani=False, full=False, display=False, projection='3d')

    for i in range(50):
        env1.step()
        env1.render(0.01)

        env1.draw_trajectory(env1.robot.trajectory, show_direction=True)

        if env1.done():
            print('done')

    env1.end()

    env2 = irsim.make('test_all_objects.yaml', display=False, projection='3d')

    env2.random_polygon_shape()
    temp_points = env2.robot.get_lidar_points()
    env2.draw_points(temp_points)

    env2.robot.get_lidar_scan()
    env2.robot.get_lidar_offset()
    env2.robot.get_init_Gh()

    for i in range(10):
        env2.step()
        env2.render(0.01)
    env2.end()

    env3 = irsim.make('test_render.yaml', save_ani=True, display=False, projection='3d')

    for i in range(3):
        env3.step()
        env3.render(0.01)
    env3.end(ani_name='test_animation')

    env4 = irsim.make('test_collision_world.yaml', save_ani=False, display=False, projection='3d')

    for i in range(4):
        env4.step()
        env4.render(0.01)
    env4.end()

    env5 = irsim.make('test_multi_objects_world.yaml', save_ani=False, display=False, projection='3d')

    env5.random_obstacle_position()

    for i in range(5):
        env5.step()
        env5.render(0.01)
    env5.end()

    env6 = irsim.make('test_grid_map.yaml', save_ani=False, display=False, projection='3d')

    for i in range(6):
        env6.step()
        env6.render(0.01)

    env6.robot.get_init_Gh()
    env6.end()

    env7 = irsim.make('test_keyboard_control.yaml', save_ani=False, display=False, projection='3d')
    key_list = ['w', 'a', 's', 'd', 'q', 'e', 'z', 'c', 'r']

    mock_key_list = []
    for c in key_list:
        mock_key = Mock(spec=keyboard.Key)
        mock_key.char = c
        mock_key_list.append(mock_key)

    for i in range(30):

        for mock_key in mock_key_list:
            env7._on_press(mock_key)
            env7._on_release(mock_key)

        env7.step()
        env7.render(0.01)

    env7.end()

    env8 = irsim.make('custom_behavior.yaml', projection='3d')
    env8.load_behavior("custom_behavior_methods")

    for i in range(10):
        env8.step()
        env8.render(0.01)

    env8.end()

    env9 = irsim.make('test_multi_objects_world.yaml', save_ani=False, display=False, projection='3d')

    env9.random_obstacle_position(ids=[3, 4, 5, 6, 7])

    for i in range(5):
        env9.step()
        env9.render(0.01)
    env9.end()

    plt.close('all')


if __name__ == "__main__":
    test_all_objects()
