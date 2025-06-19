Configure keyboard/Mouse control
==========================

IR-SIM support reading the keyboard and mouse input to control the robot manually.  

## Keyboard Control Configuration Parameters

In the keyboard control mode, the behavior of the robot is controlled by the user and the settings in the YAML file will be ignored. Please make sure the dependencies are installed before running the simulation with keyboard. The dependencies can be installed using the following command:

```bash
pip install ir-sim[keyboard]
```

To start with the keyboard control, you can simply to specify the `control_mode` parameter in the `world` section as `keyboard`. The example of the keyboard control is shown below:

```python
import irsim

env = irsim.make()

for i in range(1000):

    env.step()
    env.render(0.05)

    if env.done():
        break

env.end()
```

```yaml
world:
  height: 50  
  width: 50   
  control_mode: 'keyboard'  
  obstacle_map: 'cave.png'
  mdownsample: 2

robot:
  - kinematics: {name: 'acker'} 
    shape: {name: 'rectangle', length: 4.6, width: 1.6, wheelbase: 3}
    state: [5, 5, 0, 0]
    goal: [40, 40, 0]
    vel_max: [4, 1]
    plot:
      show_trail: True
      traj_color: 'g'
      show_trajectory: True
      show_goal: False

    sensors: 
      - type: 'lidar2d'
        range_min: 0
        range_max: 20
        angle_range: 3.14
        number: 100
        alpha: 0.4
```

The demonstration controlled by the keyboard is shown below:

```{image} gif/keyboard.gif
:alt: Select Parameters
:width: 400px
:align: center
```

## Keyboard Control Key Mapping

| Key      | Function                        |
|----------|---------------------------------|
| `w`      | Forward                         |
| `s`      | Back Forward                    |
| `a`      | Turn Left                       |
| `d`      | Turn Right                      |
| `q`      | Decrease Linear Velocity        |
| `e`      | Increase Linear Velocity        |
| `z`      | Decrease Angular Velocity       |
| `c`      | Increase Angular Velocity       |
| `alt+num`| Change Current Control Robot ID |
| `r`      | Reset the Environment           |
| `space`  | Toggle Pause/Resume Environment |

### Environment Status Control

The space key provides toggle functionality for controlling the simulation state:

- **First press**: Pauses the environment (status changes to "Pause")
- **Second press**: Resumes the environment (status changes to "Running")
- **Subsequent presses**: Continue toggling between pause and resume states

You can access the current environment status through the `env.status` attribute:

```python
import irsim

env = irsim.make('your_config.yaml', control_mode='keyboard')

for i in range(1000):
    env.step()
    env.render(0.05)
    
    # Check current status
    print(f"Environment status: {env.status}")
    
    if env.done():
        break

env.end()
```

## Mouse Control

IR-SIM supports the mouse control to zoom in and out the environment and track the mouse position. The mouse control is enabled by default. For example, you can use mouse left click to set the goal of the robot, as shown in the following figure.

```{image} gif/mouse.gif
:alt: Select Parameters
:width: 400px
:align: center
```

Python script:

```python
env = irsim.make(save_ani=False, full=False)

for i in range(10000):
    env.step()
    env.render(0.05, show_goal=False)
    
    if env.mouse_left_pos is not None:
        env.robot.set_goal(env.mouse_left_pos)

    if env.done():
        break

env.end(3)
```

YAML file:

```yaml
world:
  height: 50 
  width: 50  
  step_time: 0.1 
  sample_time: 0.1 
  offset: [0, 0] 
  collision_mode: 'stop' 
  control_mode: 'auto' 
  plot:
    saved_figure:
      bbox_inches: null

robot:
  - kinematics: {name: 'diff'} 
    shape: {name: 'circle', radius: 1}
    state: [5, 5, 0]
    vel_max: [4, 1]
    behavior: {name: 'dash'} 
    plot:
      show_trajectory: True
      traj_color: 'g'
      show_goals: True

    sensors: 
      - type: 'lidar2d'
        range_min: 0
        range_max: 20
        angle_range: 3.14
        number: 100
        noise: False
        std: 1   
        angle_std: 0.2
        offset: [0, 0, 0]
        alpha: 0.4


obstacle:
  - number: 10
    distribution: {name: 'manual'}
    shape:
      - {name: 'polygon', random_shape: true, center_range: [5, 10, 40, 30], avg_radius_range: [0.5, 2]} 
```


### Mouse Control Key Mapping

| Mouse Action | Function |
|--------------|----------|
| Mouse Movement | Track mouse position and update display coordinates |
| Middle Click | Reset zoom to default view |
| Scroll Up | Zoom in (centered on mouse position) |
| Scroll Down | Zoom out (centered on mouse position) |

### Mouse Position Attributes

| Attribute | Type | Description |
|-----------|------|-------------|
| `mouse_left_pos` | `tuple` | Position of left click (x, y) |
| `mouse_right_pos` | `tuple` | Position of right click (x, y) |
| `mouse_pos` | `tuple` | Current mouse position (x, y) |
