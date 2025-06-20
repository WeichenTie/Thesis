from dataclasses import dataclass
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
import numpy as np
import requests
import time

@dataclass
class Config:
    home_position_j: tuple
    scanning_position_l: tuple
    gripper_position_l_high: tuple
    gripper_position_l: tuple


class URController:
    def __init__(self):
        self.rtde_control = RTDEControlInterface("192.168.0.100")
        self.rtde_receive = RTDEReceiveInterface("192.168.0.100")  # Add this line
        self.config = Config(
            home_position_j=(0.0, -1.309, 1.5708, -1.8326, -1.5708, 0),
            scanning_position_l=(-0.5344, -0.159, 0.75115, 2.204,2.202, 0),
            gripper_position_l_high=(-0.90936, -0.56774, 0.51818, 2.219, 2.218, 0),
            gripper_position_l=(-0.90936, -0.56774, 0.23318, 2.219, 2.218, 0),
        )

    def set_config(self, config: Config):
        self.config = config

    def home(self):
        print('[URController] moving to home position')
        self.rtde_control.moveJ(self.config.home_position_j)

    def move_into_gripper(self):
        self.home()
        print('[URController] moving to gripper position')
        self.rtde_control.moveL(self.config.gripper_position_l_high)
        self.rtde_control.moveL(self.config.gripper_position_l)

    def move_away_from_gripper(self):
        print('[URController] moving away from gripper position')
        self.rtde_control.moveL(self.config.gripper_position_l_high)

    def move_to_scanning_position(self):
        print('[URController] moving to scanning position')
        self.rtde_control.moveL(self.config.scanning_position_l)

    def moveL(self, pose: tuple):
        print('[URController] moving to gripper position')
        self.rtde_control.moveL(pose)

    def moveL_relative(self, offset: tuple):
        current_pose = self.rtde_receive.getActualTCPPose()
        new_pose = tuple(np.add(current_pose, offset))
        self.rtde_control.moveL(new_pose)
    
    def clamp(self):
        requests.get("http://192.168.1.1/api/dc/rgxp2/set_width/0/60/8")
        time.sleep(2.5)

    def unclamp(self):
        requests.get("http://192.168.1.1/api/dc/rgxp2/set_width/0/100/8")
        time.sleep(2.5)
    
    def power_cycle(self):
        requests.get('http://192.168.1.1/api/dc/reset_tool_power')

if __name__=='__main__':
    controller = URController()
    time.sleep(1)
    controller.home()
    time.sleep(1)
    time.sleep(1)
    controller.move_to_scanning_position()
    time.sleep(1)
    controller.home()