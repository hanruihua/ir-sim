import irsim
from pynput import keyboard
from unittest.mock import Mock

env = irsim.make('test_keyboard_control.yaml', save_ani=False, display=False)

key_list = ['w', 'a', 's', 'd', 'q', 'e', 'z', 'c', 'r']

mock_key_list = []
for c in key_list:
    mock_key = Mock(spec=keyboard.Key)
    mock_key.char = c
    mock_key_list.append(mock_key)


for i in range(30):

    for mock_key in mock_key_list:
        env._on_press(mock_key)
        env._on_release(mock_key)

    env.step()
    env.render(0.01)

env.end()