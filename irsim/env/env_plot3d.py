from .env_plot import EnvPlot
import matplotlib.pyplot as plt


class EnvPlot3D(EnvPlot):

    def __init__(
        self,
        grid_map=None,
        objects=[],
        x_range=[0, 10],
        y_range=[0, 10],
        saved_figure=dict(),
        saved_ani=dict(),
        dpi: int = 100,
        figure_pixels: list = [1920, 1080],
        **kwargs,
    ):
        super().__init__(
            grid_map,
            objects,
            x_range,
            y_range,
            saved_figure,
            saved_ani,
            dpi,
            figure_pixels,
            **kwargs,
        )

        self.fig, self.ax = plt.subplots(
            figsize=(figure_pixels[0] / dpi, figure_pixels[1] / dpi),
            dpi=dpi,
            subplot_kw={"projection": "3d"},
        )
