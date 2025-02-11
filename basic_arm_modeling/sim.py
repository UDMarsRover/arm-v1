import pybullet as p
import pybullet_data
import time
import math

p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
plane = p.loadURDF("plane.urdf")
robot = p.loadURDF("basic_arm_modeling/arm.urdf")
p.resetDebugVisualizerCamera(cameraDistance=1.7, cameraYaw=50, cameraPitch=-35, cameraTargetPosition=[0, 0, 0])
L = 0.5
p.setGravity(0,0,-10)
num_joints = p.getNumJoints(robot)
joint_sliders = []


def base_link1_angle(angle):
    return math.pi/2 - angle

def link1_link2_angle(angle):
    return math.pi - angle

def angles_by_extent(r):
    # Ensure the value is within the valid range for acos
    value = (r/2)/L
    value = max(min(value, 1), -1)
    angle_1 = math.acos(value)
    angle_2 = 2 * (math.pi/2 - angle_1)
    return angle_1, angle_2

def angle_height_adjust(extent, height):
    return math.atan2(height, extent)

def cartesian_to_polar(x, y):
    r = math.sqrt(x**2 + y**2)
    theta = math.atan2(y, x)
    return r, theta

def angle_2_fixed(angle_1, angl1_2):
    return math.pi - angle_1 - angle_2

rotation_slider = p.addUserDebugParameter("rotation", -math.pi, math.pi, 0)
extent_slider = p.addUserDebugParameter("extent", 0, 2*L, 0)
height_slider = p.addUserDebugParameter("height", -1*L, 1, 0)
wrist_slider = p.addUserDebugParameter("wrist", -math.pi, math.pi, 0)
hand_slider = p.addUserDebugParameter("hand", -math.pi, math.pi, 0)

r_last = 0
while True:
    try:
        rotation_angle = p.readUserDebugParameter(rotation_slider)
        height = p.readUserDebugParameter(height_slider)
        extent = p.readUserDebugParameter(extent_slider)
        wrist_angle = p.readUserDebugParameter(wrist_slider)
        hand_angle = p.readUserDebugParameter(hand_slider)
        r, theta = cartesian_to_polar(extent, height)
        angle_1, angle_2 = angles_by_extent(r)
        angle_1 += theta
        angle_joint_1 = angle_1
        angle_joint_2 = angle_2
        angle_wrist = angle_2_fixed(angle_1, angle_2) + wrist_angle
        
        print(f"r: {r:.2f}, theta: {theta:.2f}, angle_joint_1: {angle_joint_1:.2f}, angle_joint_2: {angle_joint_2:.2f}")
        target_positions = [rotation_angle, base_link1_angle(angle_joint_1), link1_link2_angle(angle_joint_2), -angle_wrist, -hand_angle]
        p.setJointMotorControlArray(robot, range(num_joints), p.POSITION_CONTROL, targetPositions=target_positions)
        
    
    except:
        pass
    p.stepSimulation()
    time.sleep(1./60)


