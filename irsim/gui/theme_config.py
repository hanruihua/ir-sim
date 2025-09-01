try:
    import tomllib
except ModuleNotFoundError:
    import tomli as tomllib


class ThemeConfig:
    def __init__(self, theme_file: str):
        tomllib.load(self.theme_file)
