import pybullet as p
import pybullet_data
import time
import math

p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = p.loadURDF("plane.urdf")
robot = p.loadURDF("arm.urdf")
p.resetDebugVisualizerCamera(cameraDistance=1.7, cameraYaw=50, cameraPitch=-35, cameraTargetPosition=[0, 0, 0])

p.setGravity(0,0,-10)
num_joints = p.getNumJoints(robot)
joint_sliders = []

def get_positions(dist):
    """
    If we have a position, x, we can think of this as an equillateral triangle. 

    Thus, use the law of cosines to find the angles of the triangle.
    """
    arm_length = 0.5
    # Using the law of cosines to find the third side and angles
    a = arm_length
    b = arm_length
    c = 1 - dist

    # Calculate the angle opposite to side c
    angle_c = math.acos((a**2 + b**2 - c**2) / (2 * a * b))

    # Calculate the angles opposite to sides a and b
    angle_a = math.acos((b**2 + c**2 - a**2) / (2 * b * c))
    angle_b = math.acos((a**2 + c**2 - b**2) / (2 * a * c))

    return angle_a, angle_c

for i in range(num_joints):
    joint_info = p.getJointInfo(robot, i)
    joint_name = joint_info[1].decode('utf-8')
    joint_sliders.append(p.addUserDebugParameter(joint_name, -3, 3, 0))
get_positions(0.3)

distance_slider = p.addUserDebugParameter("Distance", 0, 1, 0.5)

while True:
    a = get_positions(p.readUserDebugParameter(distance_slider))
    # target_positions = [p.readUserDebugParameter(slider) for slider in joint_sliders]
    target_positions = [p.readUserDebugParameter(joint_sliders[0]), a[0], a[1]]
    p.setJointMotorControlArray(robot, range(num_joints), p.POSITION_CONTROL, targetPositions=target_positions)
    print(a)
    p.stepSimulation()
    time.sleep(1./240)

