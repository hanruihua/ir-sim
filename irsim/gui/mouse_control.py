from matplotlib.backend_bases import MouseButton
import matplotlib.pyplot as plt

class MouseControl:
    def __init__(self):

        binding_id = plt.connect('motion_notify_event', self.on_move)
        plt.connect('button_press_event', self.on_click)

        self.mouse_pos = None

    def on_move(self, event):

        if event.inaxes:
            self.mouse_pos = (event.xdata, event.ydata)
        else:
            self.mouse_pos = None

    def on_click(self, event):
        if event.button is MouseButton.LEFT:
            print('disconnecting callback')
            # plt.disconnect(binding_id)