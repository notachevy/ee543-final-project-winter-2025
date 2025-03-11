import time
import numpy as np
from scipy.spatial import ConvexHull
import serial
import sys
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from forward_k import *

np.set_printoptions(precision=2, suppress=False)
np.set_printoptions(formatter={'all': lambda x: f'{x:.2f}'})

class robot_controller():
    def __init__(self) -> None:
        #define robot parameter
        #self.simulation_mode = True
        self.joint_num = 4
        self.joints_goto_tolerance = 10e-3

        #define robot state
        self.robotstate_joint_poses = np.zeros(self.joint_num)
        self.robotstate_joint_vels = np.zeros(self.joint_num)
        self.robotState_endeffector_orientation = np.zeros(3)
        self.robotstate_endeffector_pose = np.zeros(3)
        self.robotstate_gripper_close = False

        self.workspaceX = np.array([])
        self.workspaceY = np.array([])
        self.workspaceZ = np.array([])

        #define homing position in joint space
        self.robot_homing_joint_poses = np.zeros(self.joint_num)

        """
        ---------------------------------------------------------------
         Below are the parameters related to robot link geometry
        ---------------------------------------------------------------
        """
        #define the DH parameter for the arm link
        # [a, alpha, d, theta (will be replaced by joint_positions)]
        self.dh_params = [
            [0,    0,  62.8,    0],     # Joint 1 (revolute)
            [0,  90,      0,    0],     # Joint 2 (revolute)
            [101,  0,     0,    0],     # Joint 3 (revolute)
            [0,   90,  87.5,    0],     # Joint 4 (revolute)
            [0,    0,   125,    0]      # Fixed link from joint 4 to end-effector
        ]

        self.angle_offsets = np.array([0, 90, 90, 0]) # this is for 4 joints setting

        # the transformation matrices from first to last link 
        self.T_matrices = np.empty(self.joint_num) # no value when init

        #define the base frame
        self.base_frame = np.eye(self.joint_num)

        self.error = 

        """
        ---------------------------------------------------------------
         Below are the parameters related to hardware and communciation
        ---------------------------------------------------------------
        """

        #here define the specification for MG996R servo motors
        self.servo_angle_max = 90 #degree
        self.servo_angle_min = -90 #degree
        self.servo_pulse_max = 440 #+90 for mg996R, This is the 'maximum' pulse length count (out of 4096)
        self.servo_pulse_min = 70 #-90 for mg996R, This is the 'minimum' pulse length count (out of 4096)

        # Here define the operating parameters for sliding gripper
        # Slider gripper is also controlled by an MG996R servo motor
        self.gripper_open_angle = 0 # degree
        self.gripper_close_angle = -90 # degree

        #define the serial communication parameter
        self.com_port = '/dev/cu.usbserial-A5069RR4' # change it if needed
        self.com_baudrate = 115200 #bps
        self.com_frequency = 30 #Hz
        

    """
    ---------------------------------------------------------------
     Functions below set up the serial communication
    ---------------------------------------------------------------
    """

    def communication_begin(self):
        self.ser = serial.Serial(self.com_port, self.com_baudrate)
        # Reset input/output buffer and wait for initialization
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        time.sleep(1)

        # Wait for Arduino to initialize
        while True:
            if self.ser.read() == b'I':
                break

        # Send signaling byte
        self.ser.write(b'S')
        time.sleep(0.1)
    
    def communication_end(self):
        self.ser.close()

    """
    ---------------------------------------------------------------
     Functions below set up the visualization
    ---------------------------------------------------------------
    """
        
    """
    ---------------------------------------------------------------
     Functions below setup the transformation matrix for 
     forward kinematics
    ---------------------------------------------------------------
    """

    def objective_function(self, desired_pos):
        return np.linalg.norm(np.array(self.robotstate_endeffector_pose).astype(np.float64).flatten() -
                               np.array(desired_pos)) # error

    # input: DH parameters of a specific link, angle in degree, length in mm
    # output: the transformation matrix of that link
    def dh_to_transformation_matrix(self, alpha, a, d, theta, deg=True):
        if deg:
            alpha = np.deg2rad(alpha)
            theta = np.deg2rad(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        ct = np.cos(theta)
        st = np.sin(theta)
        T = np.array([
            [   ct,       -st,        0,         a],
            [st*ca,     ct*ca,      -sa,     -sa*d],
            [st*sa,     ct*sa,       ca,      ca*d],
            [    0,         0,        0,         1]
        ], dtype=float)

        return T
    
    def update_forward_kinematics(self):
        """Calculate forward kinematics for all joints"""
        dh = np.array(self.dh_params, dtype=float)
        for i in range(self.joint_num):
            dh[[i], [3]] = self.robotstate_joint_poses[i] + self.angle_offsets[i]
        T_final = np.eye(4)
        for row in dh:
            a, alpha, d, theta = row
            T_i = self.dh_to_transformation_matrix(alpha, a, d, theta)
            T_final = T_final @ T_i

        self.robotstate_endeffector_pose = T_final[0:3, 3]

        R = T_final[0:3, 0:3]
        beta = -np.asin(R[2,0])                      
        alpha =  np.atan2(R[2,1], R[2,2])            
        gamma =  np.atan2(R[1,0], R[0,0])            
        self.robotState_endeffector_orientation = np.array([
            np.degrees(alpha),
            np.degrees(beta),
            np.degrees(gamma)
        ])

    def monte_carlo_workspace(self, N=2000):
        """
        Approximate the robot's workspace by random sampling of joint angles.
        """
        X = []
        Y = []
        Z = []
        for _ in range(N):
            rand_angles = np.random.uniform(self.servo_angle_min, 
                                            self.servo_angle_max, 
                                            self.joint_num)
            self.robotstate_joint_poses = rand_angles
            self.update_forward_kinematics()
            ee_pos = self.robotstate_endeffector_pose
            X.append(ee_pos[0])
            Y.append(ee_pos[1])
            Z.append(ee_pos[2])
        self.workspaceX = np.array(X)
        self.workspaceY = np.array(Y)
        self.workspaceZ = np.array(Z)
        fig = plt.figure()
        ax = fig.add_subplot(projection='3d')
        ax.scatter(X, Y, Z, s=2)
        ax.set_xlabel('X (mm)')
        ax.set_ylabel('Y (mm)')
        ax.set_zlabel('Z (mm)')
        plt.title("Monte Carlo Approximation of Robot Workspace")
        plt.show()

    def check_test_points(self, points, plot=True):
        if len(self.workspaceX) == 0 or len(self.workspaceY) == 0 or len(self.workspaceZ) == 0:
            raise ValueError("Workspace points not initialized. Run monte_carlo_workspace first.")

        workspace_points = np.vstack((self.workspaceX, self.workspaceY, self.workspaceZ)).T
        workspace_hull = ConvexHull(workspace_points)

        def is_inside_hull(point, hull):
            return np.all([(np.dot(eq[:-1], point) + eq[-1]) <= 0 for eq in hull.equations])

        results = []
        inside_points = []
        outside_points = []

        for point in points:
            status = "INSIDE" if is_inside_hull(point, workspace_hull) else "OUTSIDE"
            results.append(f"{point} - {status}")
            if status == "INSIDE":
                inside_points.append(point)
            else:
                outside_points.append(point)

        for result in results:
            print(result)

        if plot:
            fig = plt.figure()
            ax = fig.add_subplot(projection='3d')

            ax.scatter(self.workspaceX, self.workspaceY, self.workspaceZ, s=2, alpha=0.3, label="Workspace")

            if inside_points:
                inside_points = np.array(inside_points)
                ax.scatter(inside_points[:, 0], inside_points[:, 1], inside_points[:, 2], 
                        c='g', marker='o', s=50, label="Inside Test Points")

            if outside_points:
                outside_points = np.array(outside_points)
                ax.scatter(outside_points[:, 0], outside_points[:, 1], outside_points[:, 2], 
                        c='r', marker='x', s=50, label="Outside Test Points")

            ax.set_xlabel('X (mm)')
            ax.set_ylabel('Y (mm)')
            ax.set_zlabel('Z (mm)')
            plt.title("Robot Workspace with Test Points")
            plt.legend()
            plt.show()

        return results

    def get_link_positions(self):

        dh = np.array(self.dh_params, dtype=float)
        for i in range(self.joint_num):
            dh[i, 3] = self.robotstate_joint_poses[i] + self.angle_offsets[i]

        T = np.eye(4)

        points = [T[0:3, 3].copy()]

        for row in dh:
            a, alpha, d, theta = row
            T_i = self.dh_to_transformation_matrix(alpha, a, d, theta)
            T = T @ T_i
            points.append(T[0:3, 3].copy())

        return np.array(points)

    def get_all_joint_transforms(self):
        dh = np.array(self.dh_params, dtype=float)
        for i in range(self.joint_num):
            dh[i, 3] = self.robotstate_joint_poses[i] + self.angle_offsets[i]

        T_base = np.eye(4)
        transforms = [T_base]

        for row in dh:
            a, alpha, d, theta = row
            T_i = self.dh_to_transformation_matrix(alpha, a, d, theta)
            T_base = T_base @ T_i
            transforms.append(T_base.copy())

        return transforms 
        

    """
    ---------------------------------------------------------------
     Functions below convert the joint command into proper form for
     serial communication
    ---------------------------------------------------------------
    """
    # convert the multiple joint poses in angle into pulse lengths array
    # map the angle from -90 to 90 degree to minimal till maximal servo pulse length
    def angle_to_pulse_length(self, angles):
        clipped_angles = np.clip(angles, self.servo_angle_min, self.servo_angle_max)
        pulse_lengths = ((clipped_angles - self.servo_angle_min) * (self.servo_pulse_max - self.servo_pulse_min) / (self.servo_angle_max - self.servo_angle_min) + self.servo_pulse_min).astype(int)
        return pulse_lengths

    # convert the multiple joint poses in pulse lengths into 8 bytes array
    # format will be JP1_H, JP1_L, ..., unit: length count
    def pulse_length_to_byte(self, pulse_lengths):
        # clipped_pulse_lengths = (list)(np.clip(pulse_lengths, self.servo_pulse_min, self.servo_pulse_max))
        clipped_pulse_lengths = (list)(pulse_lengths)
        ret = []
        for pulse_length in clipped_pulse_lengths:
            # convert the number into high and low bytes
            # pulse_length = (int)pulse_length
            pulse_length_byte = int(pulse_length).to_bytes(2, byteorder='big')
            ret.append(pulse_length_byte[0])
            ret.append(pulse_length_byte[1])
        return ret
    
    
    # Set the joint to the homing position
    # Cautious: The robot will move rapidly if this is executed
    def joints_homing(self):
        # reset robot state
        self.robotstate_joint_poses = self.robot_homing_joint_poses.copy()
        self.robotstate_gripper_close = False

        # compose command
        joint_pulse_lengthes = self.angle_to_pulse_length(self.robotstate_joint_poses)
        joint_pulse_lengthes = np.append(joint_pulse_lengthes,self.gripper_pulse_open)
        # print(joint_pulse_lengthes)
        numbers = self.pulse_length_to_byte(joint_pulse_lengthes)
        # print(numbers)
        # Poll for acknowledgement
        while self.ser.in_waiting == 0:
            continue
        # ser.reset_input_buffer()

        # # Send data if acknowledgement received
        if self.ser.read() == b'A':
            self.ser.write(numbers)
            self.ser.flush()


    
    # this is the goto function in joint space
    # input is the array of joint poses(in degree) and the arry of joint velocities(degree/s)  
    def joints_goto(self, goals, speeds):
        # get the current robot joint poses
        start_poses = self.robotstate_joint_poses.copy()
        # print("Start Poses: ", start_poses)
        # calculate the rotation direction of each joints
        angle_diff = goals - start_poses
        # print("angle difference: ", angle_diff)
        # calculate the angle increments under 20Hz update rates
        angle_increments = np.sign(angle_diff) * (speeds / self.com_frequency)
        

        reached_goal = False        
        # update the robot joint poses by adding the angle increments
        while not reached_goal:
            start = time.time()
            # print("Start Poses: ", start_poses)
            # print("angle difference: ", angle_diff)

            # Generate 8 uint8_t numbers
            self.robotstate_joint_poses += angle_increments
            # check if the individual joint reach the goal
            for i in range(self.joint_num):
                if goals[i] > start_poses[i]: # the angle is increasing
                    self.robotstate_joint_poses[i] = np.clip(self.robotstate_joint_poses[i], start_poses[i], goals[i])
                elif goals[i] < start_poses[i]: # the angle is decreasing
                    self.robotstate_joint_poses[i] = np.clip(self.robotstate_joint_poses[i], goals[i], start_poses[i])
                else:
                    self.robotstate_joint_poses[i] = start_poses[i].copy()
            # print("Robotstate: ",self.robotstate_joint_poses)
            # Set the desired print options
            sys.stdout.write('\r' + ' ' * 50 + '\r') # clear the line
            sys.stdout.write("\r" + "Robotstate: " + str(self.robotstate_joint_poses))
            sys.stdout.flush()    
            
            #check if the robot reach the goal joint poses
            if np.all(np.abs(self.robotstate_joint_poses - goals) <= self.joints_goto_tolerance):
                reached_goal = True

            #convert the joint_pose to pulse length
            joint_pulse_lengthes = self.angle_to_pulse_length(self.robotstate_joint_poses)

            #add one more byte in the pulse length array to as gripper command
            if self.robotstate_gripper_close:
                joint_pulse_lengthes = np.append(joint_pulse_lengthes,self.gripper_pulse_close)
            else:
                joint_pulse_lengthes = np.append(joint_pulse_lengthes,self.gripper_pulse_open)
            # print(joint_pulse_lengthes)
            numbers = self.pulse_length_to_byte(joint_pulse_lengthes)
            # print(numbers)   

            # Poll for acknowledgement
            while self.ser.in_waiting == 0:
                continue

            # Send data if acknowledgement received
            if self.ser.read() == b'A':
                self.ser.write(numbers)
                self.ser.flush()
                dur = time.time() - start
                time.sleep(np.clip((1/self.com_frequency)-dur-0.005, 0, (1/self.com_frequency)))#50Hz

    # The function below control the end effector
    def gripper_open(self):
        #modify the robot state
        self.robotstate_gripper_close = False

        #send out the command
        #convert the joint_pose to pulse length
        joint_pulse_lengthes = self.angle_to_pulse_length(self.robotstate_joint_poses)

        #add one more byte in the pulse length array to as gripper command
        if self.robotstate_gripper_close:
            joint_pulse_lengthes = np.append(joint_pulse_lengthes,self.gripper_pulse_close)
        else:
            joint_pulse_lengthes = np.append(joint_pulse_lengthes,self.gripper_pulse_open)
        # print(joint_pulse_lengthes)
        numbers = self.pulse_length_to_byte(joint_pulse_lengthes)
        # print(numbers)   

        # Poll for acknowledgement
        while self.ser.in_waiting == 0:
            continue

        # Send data if acknowledgement received
        if self.ser.read() == b'A':
            self.ser.write(numbers)
            self.ser.flush()

    def gripper_close(self):
        #modify the robot state
        self.robotstate_gripper_close = True
        #send out the command
        #convert the joint_pose to pulse length
        joint_pulse_lengthes = self.angle_to_pulse_length(self.robotstate_joint_poses)

        #add one more byte in the pulse length array to as gripper command
        if self.robotstate_gripper_close:
            joint_pulse_lengthes = np.append(joint_pulse_lengthes,self.gripper_pulse_close)
        else:
            joint_pulse_lengthes = np.append(joint_pulse_lengthes,self.gripper_pulse_open)
        # print(joint_pulse_lengthes)
        numbers = self.pulse_length_to_byte(joint_pulse_lengthes)
        # print(numbers)   

        # Poll for acknowledgement
        while self.ser.in_waiting == 0:
            continue

        # Send data if acknowledgement received
        if self.ser.read() == b'A':
            self.ser.write(numbers)
            self.ser.flush()