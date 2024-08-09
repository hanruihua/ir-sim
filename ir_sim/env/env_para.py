import yaml
from ir_sim.util.util import file_check

class EnvPara:
    '''
    The base class of environment parameters read from yaml file.
        basic categories: world, plot, robot, obstacle, robots, obstacles, plot
    '''


    def __init__(self, world_name) -> None:
        
        world_file_path = file_check(world_name)

        self._kwargs_parse = { 'world': dict(), 'plot': dict(), 'keyboard': dict(), 'robot': None, 'robots': None, 'obstacle': None, 'obstacles': None}

        if world_file_path != None:
           
            with open(world_file_path) as file:
                com_list = yaml.load(file, Loader=yaml.FullLoader)

                for key in com_list.keys():
                    if key in self._kwargs_parse.keys():
                        self._kwargs_parse[key] = com_list[key]
                    else:
                        print(f'Invalid key: {key} in {world_name} file!')

        else:
            print('File not found!')

    @property
    def parse(self):
        return self._kwargs_parse


    



