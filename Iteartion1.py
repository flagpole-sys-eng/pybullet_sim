import pybullet as p
import pybullet_data
import time
import os

# Connect to PyBullet
physics_client = p.connect(p.GUI)

# Optional: set real-time simulation off (enables manual stepping)
p.setRealTimeSimulation(0)

# Set gravity
p.setGravity(0, 0, -9.81)

# Add PyBullet's default data path (to load the plane URDF)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load a simple ground plane
plane_id = p.loadURDF("plane.urdf")

# Also add the current directory to the search path
current_dir = os.path.dirname(os.path.abspath(__file__))
p.setAdditionalSearchPath(current_dir)

# Load the cart (flagpole URDF). 
# Set useFixedBase=False so it can move.
cart_start_pos = [0, 0, 0.1]  # Adjust height if needed so wheels aren't sunk in the plane
cart_id = p.loadURDF("flagpole.urdf", cart_start_pos, useFixedBase=False)

# Disable default motor control for all joints, to apply our own controls
num_joints = p.getNumJoints(cart_id)
for j in range(num_joints):
    p.setJointMotorControl2(
        bodyUniqueId=cart_id,
        jointIndex=j,
        controlMode=p.VELOCITY_CONTROL,
        targetVelocity=0,
        force=0
    )

# Identify which joint indices correspond to wheels
wheel_names = ["tl_base_joint", "bl_base_joint", "tr_base_joint", "br_base_joint"]
wheel_indices = []
for j in range(num_joints):
    info = p.getJointInfo(cart_id, j)
    joint_name = info[1].decode("utf-8")
    if joint_name in wheel_names:
        wheel_indices.append(j)

# Simple constants for velocity
FORWARD_VEL = 5.0
BACKWARD_VEL = -5.0
WHEEL_FORCE = 10.0  # Max force/torque to apply at the wheels

print("Use UP/DOWN arrow keys to move forward/back. Press Ctrl+C in the console to exit.")

while True:
    # Optional small delay for stable simulation timing
    time.sleep(1.0 / 240.0)

    # Get keyboard input
    keys = p.getKeyboardEvents()

    # Default: no movement
    target_velocity = 0.0

    # If UP arrow pressed
    if p.B3G_UP_ARROW in keys and (keys[p.B3G_UP_ARROW] & p.KEY_IS_DOWN):
        target_velocity = FORWARD_VEL

    # If DOWN arrow pressed
    if p.B3G_DOWN_ARROW in keys and (keys[p.B3G_DOWN_ARROW] & p.KEY_IS_DOWN):
        target_velocity = BACKWARD_VEL

    # Apply the velocity to each wheel joint
    for wj in wheel_indices:
        p.setJointMotorControl2(
            bodyUniqueId=cart_id,
            jointIndex=wj,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=target_velocity,
            force=WHEEL_FORCE
        )

    # Step the simulation
    p.stepSimulation()
