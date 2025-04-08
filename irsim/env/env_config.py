import yaml
from irsim.util.util import file_check
from irsim.global_param import env_param


class EnvConfig:
    """
    The base class of environment parameters read from yaml file.
        basic categories: world, plot, robot, obstacle
        See for detail
    """

    def __init__(self, world_name) -> None:

        world_file_path = file_check(world_name)

        self._kwargs_parse = {
            "world": dict(),
            "plot": dict(),
            "keyboard": dict(),
            "robot": None,
            "obstacle": None,
        }

        if world_file_path != None:

            with open(world_file_path) as file:
                com_list = yaml.load(file, Loader=yaml.FullLoader)

                for key in com_list.keys():
                    if key in self._kwargs_parse.keys():
                        self._kwargs_parse[key] = com_list[key]
                    else:
                        self.logger.error(
                            f"There are invalid key: '{key}' in {world_name} file!"
                        )
                        raise KeyError

        else:
            self.logger.warning(f"{world_name} YAML File not found!, using default world config.")

    @property
    def parse(self):
        """
        The parsed kwargs from the yaml file.
        """
        return self._kwargs_parse
    
    @property
    def logger(self):
        '''
        Get the logger of the env_param.
        '''
        return env_param.logger
