import pybullet as p
import pybullet_data
import time

# Connect to PyBullet with GUI
physics_client = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Add built-in data path

# Load the cart-pole URDF
pendulum = p.loadURDF("cartpole.urdf")  # Ensure correct path

# Set simulation parameters
p.setGravity(0, 0, -9.8)

# Joint indices
cart_joint_index = 0  # Prismatic joint (cart motion)
pole_joint_index = 1  # Revolute joint (pendulum)

# Disable default motor control (VERY IMPORTANT for free motion)
p.setJointMotorControl2(pendulum, cart_joint_index, controlMode=p.VELOCITY_CONTROL, force=0)
p.setJointMotorControl2(pendulum, pole_joint_index, controlMode=p.VELOCITY_CONTROL, force=0)

# Remove damping so the pendulum swings freely
p.changeDynamics(pendulum, pole_joint_index, linearDamping=0, angularDamping=0, jointDamping=0)

# Define force values
cart_force = 200  # Force for moving cart left/right

# Main simulation loop
try:
    while True:
        # Step the simulation
        p.stepSimulation()

        # Get keyboard events
        keys = p.getKeyboardEvents()

        # Apply force to move the cart
        if ord('a') in keys or p.B3G_LEFT_ARROW in keys:
            p.setJointMotorControl2(pendulum, cart_joint_index, controlMode=p.TORQUE_CONTROL, force=-cart_force)
        
        elif ord('d') in keys or p.B3G_RIGHT_ARROW in keys:
            p.setJointMotorControl2(pendulum, cart_joint_index, controlMode=p.TORQUE_CONTROL, force=cart_force)
        
        else:
            p.setJointMotorControl2(pendulum, cart_joint_index, controlMode=p.TORQUE_CONTROL, force=0)  # Stop cart if no key is pressed

        # Quit simulation
        if ord('q') in keys:
            print("Quitting simulation.")
            break

        time.sleep(0.01)

except KeyboardInterrupt:
    pass

# Disconnect from PyBullet
p.disconnect()
