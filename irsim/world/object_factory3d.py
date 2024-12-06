from irsim.world import ObjectFactory
from irsim.world import (
    RobotAcker3D,
    RobotDiff3D,
    RobotOmni3D,
    ObstacleAcker3D,
    ObstacleDiff3D,
    ObstacleOmni3D,
    ObjectStatic3D,
)

class ObjectFactory3D(ObjectFactory):
    
        
    def create_robot(self, kinematics=dict(), **kwargs):
        """
        Create a robot based on kinematics.

        Args:
            kinematics (dict): Kinematics configuration.
            **kwargs: Additional parameters for robot creation.

        Returns:
            Robot: An instance of a robot.
        """
        kinematics_name = kinematics.get("name", None)

        if kinematics_name == "diff":
            return RobotDiff3D(kinematics=kinematics, **kwargs)
        elif kinematics_name == "acker":
            return RobotAcker3D(kinematics=kinematics, **kwargs)
        elif kinematics_name == "omni":
            return RobotOmni3D(kinematics=kinematics, **kwargs)
        elif kinematics_name == "static" or kinematics_name is None:
            return ObjectStatic3D(kinematics=kinematics, role="robot", **kwargs)
        else:
            raise NotImplementedError(
                f"Robot kinematics {kinematics_name} not implemented"
            )

    def create_obstacle(self, kinematics=dict(), **kwargs):
        """
        Create a obstacle based on kinematics.

        Args:
            kinematics (dict): Kinematics configuration.
            **kwargs: Additional parameters for robot creation.

        Returns:
            Obstacle: An instance of an obstacle.
        """
        kinematics_name = kinematics.get("name", None)

        if kinematics_name == "diff":
            return ObstacleDiff3D(kinematics=kinematics, **kwargs)
        elif kinematics_name == "acker":
            return ObstacleAcker3D(kinematics=kinematics, **kwargs)
        elif kinematics_name == "omni":
            return ObstacleOmni3D(kinematics=kinematics, **kwargs)
        elif kinematics_name == "static" or kinematics_name is None:
            return ObjectStatic3D(kinematics=kinematics, role="obstacle", **kwargs)
        else:
            raise NotImplementedError(
                f"Robot kinematics {kinematics_name} not implemented"
            )