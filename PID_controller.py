import pybullet as p
import pybullet_data
import time
import os
import math
import matplotlib.pyplot as plt

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
        Compute the PID control output and return (control_output, current_error).

        Parameters:
          measurement: The current measurement (e.g., the angle of the pole in radians).
          dt: Time step.

        Returns:
          (control_output, error)
        """
        # Calculate the error: measured value minus the setpoint (0 means the pole should be upright).
        error = measurement - self.setpoint
        
        # Accumulate the integral term.
        self.integral += error * dt
        
        # Compute the derivative term.
        derivative = (error - self.previous_error) / dt
        
        # PID output.
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        
        # Update the previous error.
        self.previous_error = error

        # Clamp the output to the specified limits.
        output = max(self.output_limits[0], min(output, self.output_limits[1]))
        return output, error

def main():
    # Initialize the PID controller. Adjust parameters as needed.
    pid = PIDController(Kp=6, Ki=0.3, Kd=0.39, setpoint=0.0, output_limits=(-100.0, 100.0))

    # Time step.
    dt = 1.0 / 240.0

    # Connect to PyBullet and configure the environment.
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # Set gravity.
    p.setGravity(0, 0, -9.81)

    # Load the ground plane.
    plane_id = p.loadURDF("plane.urdf")

    # Add the current directory to the search path (for loading custom URDFs).
    current_dir = os.path.dirname(os.path.abspath(__file__))
    p.setAdditionalSearchPath(current_dir)

    # Load the cart-pole (flagpole) URDF.
    cart_start_pos = [0, 0, 0.1]  # Slightly above the ground to prevent sinking.
    cart_start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    cart_id = p.loadURDF("flagpole.urdf", cart_start_pos, cart_start_orientation, useFixedBase=False)

    # Disable default motor control for all joints.
    num_joints = p.getNumJoints(cart_id)
    for j in range(num_joints):
        p.setJointMotorControl2(
            bodyUniqueId=cart_id,
            jointIndex=j,
            controlMode=p.VELOCITY_CONTROL,
            targetVelocity=0,
            force=0
        )

    # Identify the wheel joints and the pole joint by name.
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

    # Optional: apply an initial tilt to the pole so the PID has a non-zero error to correct.
    initial_tilt = 0.1  # in radians
    p.resetJointState(cart_id, pole_joint_index, initial_tilt)

    # For recording time and error data.
    time_data = []
    error_data = []
    sim_time = 0.0

    # Limit the simulation time to 10 seconds.
    max_sim_time = 20.0

    print("Simulation running for 10 seconds.")
    try:
        while sim_time < max_sim_time:
            start_time = time.time()

            # Get the current pole angle.
            joint_state = p.getJointState(cart_id, pole_joint_index)
            pole_angle = joint_state[0]  # in radians

            # Compute the PID control output and the error.
            control_effort, error = pid.compute(pole_angle, dt)

            # Record the error and current time.
            error_data.append(error)
            time_data.append(sim_time)
            sim_time += dt

            # Debug output.
            print(f"Time: {sim_time:.2f}s, Pole Angle: {pole_angle:.4f} rad, Error: {error:.4f}, Control Effort: {control_effort:.2f}")

            # Apply the control torque to all wheel joints.
            for wj in wheel_indices:
                p.setJointMotorControl2(
                    bodyUniqueId=cart_id,
                    jointIndex=wj,
                    controlMode=p.TORQUE_CONTROL,
                    force=control_effort
                )

            # Update the camera position to follow the cart.
            cart_pos, _ = p.getBasePositionAndOrientation(cart_id)
            p.resetDebugVisualizerCamera(
                cameraDistance=3,
                cameraYaw=50,
                cameraPitch=-35,
                cameraTargetPosition=cart_pos
            )

            # Step the simulation.
            p.stepSimulation()

            # Maintain the loop frequency.
            elapsed = time.time() - start_time
            sleep_time = dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
    except KeyboardInterrupt:
        print("Simulation terminated by user.")
    finally:
        p.disconnect()
        # Plot the error over time.
        plt.figure(figsize=(10, 5))
        plt.plot(time_data, error_data, label='Error (rad)', color='red')
        plt.xlabel('Time (s)')
        plt.ylabel('Error (rad)')
        plt.title('PID Error Over Time')
        plt.legend()
        plt.grid(True)
        plt.show()

if __name__ == "__main__":
    main()
