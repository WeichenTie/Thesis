import time
import random
from ur_controller import URController
from cv_controller import RemoteVisionController
from gripper_controller import GripperController

def run():
    ur_controller = URController()
    cv_controller = RemoteVisionController()
    gripper_controller = GripperController()

    gripper_controller.unclamp()
    ur_controller.unclamp()

    ur_controller.home()
    ur_controller.move_to_scanning_position()

    uv = cv_controller.detect_OOI_position(retry=False)
    while uv is False:
        uv = cv_controller.detect_OOI_position(retry=False)
        ur_controller.moveL_relative((0.02 * (random.random() - 0.5),0.02 * (random.random() - 0.5),0.02 * (random.random() - 0.5),0,0,0))
    position = cv_controller.pixel_to_cam(uv[0], uv[1])

    ur_controller.moveL_relative((-position[1],-position[0], 0 , 0,0,0))
    ur_controller.moveL_relative((0,0, -position[2] + 0.4, 0,0,0))

    for i in range(5):
        uv = cv_controller.detect_OOI_position()
        position = cv_controller.pixel_to_cam(uv[0], uv[1])
        ur_controller.moveL_relative((-position[1],-position[0], 0 , 0,0,0))
    
    ur_controller.moveL_relative((-0.095, 0.03, 0 , 0,0,0))
    ur_controller.moveL_relative((0, 0, -0.23 , 0,0,0))

    ur_controller.clamp()
    ur_controller.move_into_gripper()

    gripper_controller.clamp()
    ur_controller.unclamp()

    ur_controller.move_away_from_gripper()
    
    time.sleep(2)
    ur_controller.move_into_gripper()
    ur_controller.clamp()
    gripper_controller.unclamp()

    ur_controller.home()
    

   
if __name__=="__main__":
    run()