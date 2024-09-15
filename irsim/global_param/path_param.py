import sys
from dataclasses import dataclass
import os
import irsim

@dataclass
class PathManager:

    '''
    Module for managing the path of the project.
        root_path: path of the irsim package
        ani_buffer_path: path of the animation buffer
        ani_path: path of the animation
        fig_path: path of the saved figure
    '''

    root_path: str = os.path.dirname(irsim.__file__)
    ani_buffer_path: str = sys.path[0] + "/animation_buffer"
    ani_path: str = sys.path[0] + "/animation"
    fig_path: str = sys.path[0] + "/figure"


path_manager = PathManager()
