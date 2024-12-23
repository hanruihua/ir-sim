Render and Save Animation
==============

## Render the environment

You can render the environment by calling the [env.render()](#irsim.env.env_base.EnvBase.render) function and change the arguments of this function to control the rendering of each frame. The main parameters are:

- **`interval`:** The time interval between frames. Default 0.05. This parameter is used to control the speed of visualization.
- **`figure_kwargs`:** The parameters of the figures. Such as the transparent, bbox_inches, dpi, etc. Default {}. See [savefig](https://matplotlib.org/stable/api/_as_gen/matplotlib.pyplot.savefig.html) for more details.
- **`kwargs`:** These additional parameters are passed to the all the [object.plot](#irsim.world.object_base.ObjectBase.plot) function for default settings. 

## Save the animation

You can save the animation of the simulation as a gif file very easily by setting the `save_ani` to `True` in the `make()` function:

```python

env = irsim.make(save_ani=True)

for i in range(300):

    env.step()
    env.render(0.05)

    if env.done():
        break

env.end(ending_time=3)
```

`ending_time` denotes how long the figure will be closed. The animation generation is also performed in this `env.end()` function. You can set the additional arguments in this function to control the animation generation. Details of the parameters can be found in the [env.end()](#irsim.env.env_base.EnvBase.end). 

Some common parameters for `GIF` format you may use are: 

- **`loop`:** The number of times the GIF should loop. Default 0 (meaning loop indefinitely).
- **`duration `:** The duration (in seconds) of each frame. Either specify one value that is used for all frames, or one value for each frame. 
- **`fps`:** The number of frames per second. If duration is not given, the duration for each frame is set to 1/fps. Default 10.
- **`subrectangles `:** If True, will try and optimize the GIF by storing only the rectangular parts of each frame that change with respect to the previous. Default True.

:::{tip}
The principle of the animation generation is to save the images of each frame and then combine them into a gif file.
:::

## 3D Plot

You can simply set the `projection` parameter to `3d` in `irsim.make` function to render the 3D plot of the simulation. The example is shown below:

```python

env = irsim.make(projection='3d')

for i in range(300):

    env.step()
    env.render(0.05)

    if env.done():
        break

env.end(3)
```

```{image} gif/3d_plot.gif
:alt: Select Parameters
:width: 400px
:align: center
```

:::{note}
Currently, the 3D plot only visualizes the 2D objects in the 3D space. The 3D objects are not supported yet. 
:::


