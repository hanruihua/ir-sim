User Guide
===============

Practical, task-focused guides for building and running IR-SIM scenarios. Browse the highlights below, or jump to any page from the contents.

.. grid:: 2 3 3 3
    :gutter: 2
    :class-container: usage-gallery

    .. grid-item-card:: Make environment
        :img-top: https://raw.githubusercontent.com/IR-SIM/IR-SIM-GIFs/main/robots_obstacles/robot_obstacle.gif
        :link: make_environment
        :link-type: doc
        :text-align: center
        :shadow: md

        Build a world and drive the step → render → done loop.

    .. grid-item-card:: Robots & obstacles
        :img-top: https://raw.githubusercontent.com/IR-SIM/IR-SIM-GIFs/main/robots_obstacles/multi_objects.gif
        :link: configure_robots_obstacles
        :link-type: doc
        :text-align: center
        :shadow: md

        Add robots and obstacles with different kinematics and shapes.

    .. grid-item-card:: Sensors
        :img-top: https://raw.githubusercontent.com/IR-SIM/IR-SIM-GIFs/main/sensors/lidar2d.gif
        :link: configure_sensor
        :link-type: doc
        :text-align: center
        :shadow: md

        Attach 2D LiDAR, FMCW LiDAR, and field-of-view detectors.

    .. grid-item-card:: Behaviors
        :img-top: https://raw.githubusercontent.com/IR-SIM/IR-SIM-GIFs/main/behavior/rvo.gif
        :link: configure_behavior
        :link-type: doc
        :text-align: center
        :shadow: md

        dash, RVO, ORCA, and Social Force Model behaviors.

    .. grid-item-card:: Keyboard & mouse
        :img-top: https://raw.githubusercontent.com/IR-SIM/IR-SIM-GIFs/main/keyboard_mouse/keyboard.gif
        :link: configure_keyboard_Mouse_control
        :link-type: doc
        :text-align: center
        :shadow: md

        Drive and interact with robots by keyboard or mouse.

    .. grid-item-card:: Social Force Model
        :img-top: https://raw.githubusercontent.com/IR-SIM/IR-SIM-GIFs/main/behavior/sfm_world.gif
        :link: configure_behavior
        :link-type: doc
        :text-align: center
        :shadow: md

        Pedestrian-style crowd navigation with the Social Force Model.

    .. grid-item-card:: Grid maps
        :img-top: https://raw.githubusercontent.com/IR-SIM/IR-SIM-GIFs/main/grid_map/grid_map.gif
        :link: configure_grid_map
        :link-type: doc
        :text-align: center
        :shadow: md

        Build occupancy grids from images or Perlin noise.

    .. grid-item-card:: Path planning
        :img-top: gif/path_planning.png
        :link: configure_path_planning
        :link-type: doc
        :text-align: center
        :shadow: md

        Plan collision-free paths with A*, JPS, RRT, RRT*, and PRM.

    .. grid-item-card:: Dynamic scenes
        :img-top: https://raw.githubusercontent.com/IR-SIM/IR-SIM-GIFs/main/dynamic_random_env/random_obstacles.gif
        :link: configure_dynamic_random_env
        :link-type: doc
        :text-align: center
        :shadow: md

        Spawn randomized dynamic obstacles on every run.

Contents
--------

.. toctree::
   :maxdepth: 2
   :caption: Basic Usage

   make_environment
   configure_robots_obstacles
   configure_sensor
   configure_keyboard_Mouse_control
   save_animation

.. toctree::
   :maxdepth: 2
   :caption: Advanced Usage

   multiple_environments
   configure_behavior
   configure_grid_map
   configure_path_planning
   configure_dynamic_random_env
