import time
import random
from ur_controller import URController
from cv_controller import RemoteVisionController
from gripper_controller import GripperController



ur_controller = URController()
cv_controller = RemoteVisionController()
gripper_controller = GripperController()

def setup():
    ur_controller.power_cycle()
    time.sleep(1)
    gripper_controller.unclamp()
    gripper_controller.await_command_complete()
    ur_controller.unclamp()
    ur_controller.home()

def detect_all_orange_uv_positions():
    ur_controller.move_to_scanning_position()
    uvs = cv_controller.detect_OOI_positions(retry=False)
    while uvs is False or len(uvs) == 0:
        uvs = cv_controller.detect_OOI_positions(retry=False)
        ur_controller.moveL_relative((0.02 * (random.random() - 0.5),0.02 * (random.random() - 0.5),0,0,0,0))
    positions = []
    id = 0
    for uv in uvs:
        positions.append((id, cv_controller.pixel_to_cam(uv[0], uv[1])))
        id = id + 1
    return (ur_controller.get_position(), positions)

def get_calibrated_position(robot_position, uv):
    ur_controller.home()

    ur_controller.moveL((
        robot_position[0] - uv[1],
        robot_position[1] - uv[0],
        robot_position[2] - uv[2] + 0.4,
        robot_position[3],
        robot_position[4],
        robot_position[5]))
    for i in range(5):
        uv = cv_controller.detect_OOI_position()
        position = cv_controller.pixel_to_cam(uv[0], uv[1])
        ur_controller.moveL_relative((-position[1],-position[0], 0 , 0,0,0))
    
    new_position = ur_controller.get_position()
    new_position[0] += -0.095
    new_position[1] += 0.03
    new_position[2] += -0.225
    return new_position

def stash_from_gripper():
    ur_controller.move_away_from_gripper()
    ur_controller.move_into_gripper()
    ur_controller.clamp()
    gripper_controller.unclamp()
    gripper_controller.await_command_complete()

def run():

    setup()
    robot_initial_scan_position, uv_positions = detect_all_orange_uv_positions()
    calibratedPositions = dict()
    while uv_positions:
        ur_controller.home()
        (id, uv) = uv_positions.pop()
        print(id)
        print(uv)
        if ((id not in calibratedPositions)):
            position = get_calibrated_position(robot_initial_scan_position, uv)
        else:
            position = calibratedPositions.get(id)

        position_high = position[:]
        position_high[2] = position_high[2] + 0.23
        ur_controller.moveL(position_high)
        ur_controller.moveL(position)

        ur_controller.clamp()
        ur_controller.move_away_from_gripper()
        ur_controller.move_into_gripper()

        gripper_controller.clamp()
        gripper_controller.await_command_complete()

        ur_controller.unclamp()

        ur_controller.move_away_from_gripper()
 
        gripper_controller.scan()
        
        while not gripper_controller.is_command_complete():
            pass

        ur_controller.move_away_from_gripper()
        ur_controller.move_into_gripper()
        ur_controller.clamp()
        gripper_controller.unclamp()
        gripper_controller.await_command_complete()
        ur_controller.move_away_from_gripper()
        ur_controller.move_to_completed_high()
        ur_controller.move_to_completed()
        ur_controller.unclamp()
        ur_controller.move_to_completed_high()
    

   
if __name__=="__main__":
    run()