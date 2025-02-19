import pybullet as p
import pybullet_data
import time
import os
import math

class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint=0.0, output_limits=(-10.0, 10.0)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint

        self.integral = 0.0
        self.previous_error = 0.0
        self.output_limits = output_limits  # (min, max)

    def compute(self, measurement, dt):
        """
        Compute the PID control effort.
        
        Parameters:
          measurement: The current measurement (e.g. joint angle in radians).
          dt: Time step duration.
        
        Returns:
          control output (clamped to output_limits)
        """
        # Compute the error between the setpoint and the measurement.
        error = self.setpoint - measurement
        
        # Integrate the error.
        self.integral += error * dt
        
        # Compute the derivative of the error.
        derivative = (error - self.previous_error) / dt
        
        # Compute the PID output.
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        
        # Update for next derivative calculation.
        self.previous_error = error

        # Clamp the output to the specified limits.
        output = max(self.output_limits[0], min(output, self.output_limits[1]))
        return output

def main():
    # Initialize PID controller for pole stabilization.
    # These gains are an example; you might need to tune them for your system.
    pid = PIDController(Kp=150.0, Ki=5.0, Kd=30.0, setpoint=0.0, output_limits=(-100.0, 100.0))

    # Time step
    dt = 1.0 / 240.0

    # Connect to PyBullet and configure the environment.
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Set gravity.
    p.setGravity(0, 0, -9.81)

    # Load the ground plane.
    plane_id = p.loadURDF("plane.urdf")

    # Add the current directory to the search path for your custom URDF.
    current_dir = os.path.dirname(os.path.abspath(__file__))
    p.setAdditionalSearchPath(current_dir)

    # Load the cart-pole (flagpole) URDF.
    cart_start_pos = [0, 0, 0.1]  # Slightly above the ground to prevent sinking.
    cart_start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    cart_id = p.loadURDF("flagpole.urdf", cart_start_pos, cart_start_orientation, useFixedBase=False)

    # Disable the default motor control for all joints so we can apply our custom torque.
    num_joints = p.getNumJoints(cart_id)
    for j in range(num_joints):
        p.setJointMotorControl2(
            bodyUniqueId=cart_id,
            jointIndex=j,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=0,
            force=0
        )

    # Identify wheel joints and the pole joint by name.
    wheel_names = ["tl_base_joint", "bl_base_joint", "tr_base_joint", "br_base_joint"]
    wheel_indices = []
    pole_joint_index = None
    for j in range(num_joints):
        info = p.getJointInfo(cart_id, j)
        joint_name = info[1].decode("utf-8")
        if joint_name in wheel_names:
            wheel_indices.append(j)
        elif joint_name == "pole_base_joint":
            pole_joint_index = j

    if pole_joint_index is None:
        print("Error: 'pole_base_joint' not found. Check URDF joint names.")
        p.disconnect()
        return

    if len(wheel_indices) != 4:
        print("Error: Could not find all wheel joints. Check URDF joint names.")
        p.disconnect()
        return

    print("Wheel joint indices:", wheel_indices)
    print("Pole joint index:", pole_joint_index)

    # Optionally, apply an initial tilt to the pole so that the PID controller has a nonzero error to correct.
    initial_tilt = 0.1  # radians (adjust as desired)
    p.resetJointState(cart_id, pole_joint_index, initial_tilt)

    # Simulation loop.
    print("Simulation running. Press Ctrl+C to exit.")
    try:
        while True:
            start_time = time.time()

            # Get the pole's current angle from the joint state.
            joint_state = p.getJointState(cart_id, pole_joint_index)
            pole_angle = joint_state[0]  # Joint position (angle in radians)
            # (The joint velocity is available as joint_state[1], but our PID
            #  derivative is computed using the change in error.)

            # Compute control effort using the PID controller.
            control_effort = pid.compute(pole_angle, dt)

            # Debug output.
            print(f"Pole Angle (rad): {pole_angle:.4f}, Control Effort: {control_effort:.2f}")

            # Apply the computed control effort as torque to all wheel joints.
            for wj in wheel_indices:
                p.setJointMotorControl2(
                    bodyUniqueId=cart_id,
                    jointIndex=wj,
                    controlMode=p.TORQUE_CONTROL,
                    force=control_effort
                )

            # Step the simulation.
            p.stepSimulation()

            # Maintain the loop rate.
            elapsed = time.time() - start_time
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
    except KeyboardInterrupt:
        print("Simulation terminated by user.")
    finally:
        p.disconnect()

if __name__ == "__main__":
    main()
