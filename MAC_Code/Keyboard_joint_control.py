"""""""""""""""""""""""""""""

University of Washington, 2024

Author: Tin Chiang

Note: Modified code from Haonan Peng's Raven keyboard controller

Original code: https://github.dev/uw-biorobotics/raven2_CRTK_Python_controller/blob/main/python_controller/run_r2_keyboard_controller.py
"""""""""""""""""""""""""""""
import time
import numpy as np
import sys
import os
from pynput import keyboard
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from robot_controller_win import robot_controller

# Global variable to track the last key pressed
last_key = None

def on_press(key):
    global last_key
    try:
        last_key = key.char
    except AttributeError:
        pass

def print_menu():
    os.system('clear')
    print('''
-----------------------------------------
EE543 Arm Keyboard Controller:
-----------------------------------------
[Exit]: 9
[Joint 1    +]: 1 | [Joint 1     -]: q
[Joint 2    +]: 2 | [Joint 2     -]: w
[Joint 3    +]: 3 | [Joint 3     -]: e
[Joint 4    +]: 4 | [Joint 4     -]: r
[Grasper Open]: 5 | [Grasper Close]: t
[Homing]: h
-----------------------------------------
Current command:''')

def print_no_newline(string):
    sys.stdout.write("\r" + string + " " * 20)  # Add padding to clear previous text
    sys.stdout.flush()

def plot_joint_frames(transforms, ax, axis_length=20):
    """
    For each transform ^0T_i in transforms,
    draw x,y,z arrows of some length (in mm).
    """
    for i, T in enumerate(transforms):
        origin = T[0:3, 3]
        R = T[0:3, 0:3]
        x_axis = R[:,0]
        y_axis = R[:,1]
        z_axis = R[:,2]

        # Plot a blue arrow for X
        ax.quiver(
            origin[0], origin[1], origin[2],
            x_axis[0], x_axis[1], x_axis[2],
            color='b', length=axis_length, normalize=True
        )
        # # Plot a green arrow for Y
        ax.quiver(
            origin[0], origin[1], origin[2],
            y_axis[0], y_axis[1], y_axis[2],
            color='g', length=axis_length, normalize=True
        )
        # Plot a red arrow for Z
        ax.quiver(
            origin[0], origin[1], origin[2],
            z_axis[0], z_axis[1], z_axis[2],
            color='r', length=axis_length, normalize=True
        )

def plot_arm_skeleton(transforms, ax):
    """
    transforms: list of 4x4 matrices [^0T_0, ^0T_1, ^0T_2, ... , ^0T_n].
    ax: the 3D subplot
    """
    points = [T[0:3, 3] for T in transforms]
    points = np.array(points)
    plt_x_lim = [-200, 200]
    plt_y_lim = [-200, 200]
    plt_z_lim = [0, 300]
    xs = points[:,0]
    ys = points[:,1]
    zs = points[:,2]
    ax.set_xlim(plt_x_lim)
    ax.set_ylim(plt_y_lim)
    ax.set_zlim(plt_z_lim)    
    ax.plot(xs, ys, zs, 'o-', color='k', linewidth=3, markersize=6)

def plot_arm(link_positions, ax):
    """
    link_positions: Nx3 array, with N=6 if you have 5 links
    ax: 3D axis
    """
    plt_x_lim = [-200, 200]
    plt_y_lim = [-200, 200]
    plt_z_lim = [0, 300]
    ax.set_xlim(plt_x_lim)
    ax.set_ylim(plt_y_lim)
    ax.set_zlim(plt_z_lim) 

    xs = link_positions[:,0]
    ys = link_positions[:,1]
    zs = link_positions[:,2]
    ax.cla()
    ax.plot(xs, ys, zs, marker='o', linewidth=2, markersize=5)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title("Robot Arm Visualization")

    max_range = max(xs.max()-xs.min(), ys.max()-ys.min(), zs.max()-zs.min())
    mid_x = (xs.max()+xs.min()) * 0.5
    mid_y = (ys.max()+ys.min()) * 0.5
    mid_z = (zs.max()+zs.min()) * 0.5
    ax.set_xlim(mid_x - 0.5*max_range, mid_x + 0.5*max_range)
    ax.set_ylim(mid_y - 0.5*max_range, mid_y + 0.5*max_range)
    ax.set_zlim(mid_z - 0.5*max_range, mid_z + 0.5*max_range)

def main():
    global last_key

    # Setup figure for plotting
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    plt.ion()

    # Initialize
    RC = robot_controller()
    RC.communication_begin()
    RC.joints_homing()

    # Some typical speeds
    keyboard_increment = 5.0  # move 5 deg at a time for big steps
    speeds = np.ones(RC.joint_num) * 80  # deg/s
    goals = RC.robot_homing_joint_poses.copy()

    # Start keyboard listener
    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    print_menu()
    print("Press 9 to exit, etc...")

    try:
        while True:
            if last_key is not None:
                current_key = last_key
                last_key = None
                command = False

                if current_key == '9':
                    break

                elif current_key == '1':
                    print_no_newline("Moving: Joint 1 +++")
                    goals[0] += keyboard_increment
                    command = True

                elif current_key == 'q':
                    print_no_newline("Moving: Joint 1 ---")
                    goals[0] -= keyboard_increment
                    command = True

                elif current_key == '2':
                    print_no_newline("Moving: Joint 2 +++")
                    goals[1] += keyboard_increment
                    command = True

                elif current_key == 'w':
                    print_no_newline("Moving: Joint 2 ---")
                    goals[1] -= keyboard_increment
                    command = True

                elif current_key == '3':
                    print_no_newline("Moving: Joint 3 +++")
                    goals[2] += keyboard_increment
                    command = True

                elif current_key == 'e':
                    print_no_newline("Moving: Joint 3 ---")
                    goals[2] -= keyboard_increment
                    command = True

                elif current_key == '4':
                    print_no_newline("Moving: Joint 4 +++")
                    goals[3] += keyboard_increment
                    command = True

                elif current_key == 'r':
                    print_no_newline("Moving: Joint 4 ---")
                    goals[3] -= keyboard_increment
                    command = True

                elif current_key == 'h':
                    print_no_newline("Homing...")
                    goals = RC.robot_homing_joint_poses.copy()
                    command = True

                elif current_key == '5':
                    print_no_newline(" Grasper Open....                  ")
                    RC.gripper_set_percentage(0)
                    
                elif current_key == 't':
                    print_no_newline(" Grasper Close....                  ")
                    RC.gripper_set_percentage(100)
                    
                # If we changed any joint angles, move real robot & update plot
                if command:
                    # Clip angles between -90 and +90
                    goals = np.clip(goals, RC.servo_angle_min, RC.servo_angle_max)

                    # Move the real robot29
                    RC.joints_goto(goals, speeds)

            # Update the simulation plot each loop 
            link_positions = RC.get_link_positions()   # make sure you have get_link_positions in robot_controller
            # each loop iteration:
            transforms = RC.get_all_joint_transforms()

            # 1) Clear axis
            ax.cla()

            # 2) Plot the skeleton
            plot_arm_skeleton(transforms, ax)

            # 3) Plot the joint frames
            plot_joint_frames(transforms, ax, axis_length=20)

            # 4) Set labels, etc.
            ax.set_xlabel("X")
            ax.set_ylabel("Y")
            ax.set_zlabel("Z")
            ax.set_title(f"End-effector Location (x, y, z): {transforms[-1][0:3,3]}")

            plt.draw()
            plt.pause(0.01)
            time.sleep(0.01)  # to prevent CPU hogging

    except KeyboardInterrupt:
        pass
    finally:
        listener.stop()
        RC.communication_end()
        plt.ioff()
        plt.show()


if __name__ == "__main__":

    RC = robot_controller()
    '''
    --------------------------------------------------------------------
    Normal Functions - Keyboard Joint Control
                     - Forward Kinematic Simulation
                     - Monte-Carlo Workspace with Test-Points
    -------------------------------------------------------------------- 
    '''
    # main()
    # #RC.robotstate_joint_poses = np.array([0, 0, 0, 0])
    # RC.update_forward_kinematics()
    # print("End Effector Position:", RC.robotstate_endeffector_pose)
    # print("End Effector Orientation (deg):", RC.robotState_endeffector_orientation)
    # RC.monte_carlo_workspace(N=10000)
    # test_points = [(70, 50, 100), (175, 150, 400), (180, 180, 200)]
    # RC.check_test_points(test_points) 

    '''
    --------------------------------------------------------------------
    Position Control in Cartesian Space - Home position to [x, y, z] 
    point within workspace 
    -------------------------------------------------------------------- 
    '''
    # RC.communication_begin()
    # RC.joints_homing()
    # RC.update_forward_kinematics()
    # time.sleep(1.5)

    #current_pos = RC.robotstate_endeffector_pose

    # desired_pos = [217.60, 125.63, -87.46]

    # # Solve IK
    # optimized_angles = RC.inverse_kinematics(desired_pos)

    # if optimized_angles is not None:
    #     # Move to the optimized joint angles with a speed of 10 degrees/sec per joint
    #     speeds = np.array([5, 5, 5, 5], dtype=float)
    #     RC.joints_goto(optimized_angles, speeds)
    #     # Update the end-effector position after movement
    #     RC.update_forward_kinematics()
    #     print("New end-effector position:", RC.robotstate_endeffector_pose)
    # else:
    #     print("IK solution not found.")

    # #joints = RC.inversek_N(RC.robotstate_endeffector_pose + [20, 0, 0])
    # joints = RC.inversek_N([200, 200, 200])
    # print(joints)
    # #joints = np.clip(joints, -90, 90)
    # #speeds = np.array([5, 5, 5, 5], dtype=float)
    # #RC.joints_goto(joints, speeds)
    '''
    --------------------------------------------------------------------
    Object Pickup & Drop Function - Home position to [x, y, z] point 
                                    within workspace to pick up the 
                                    object (Gripper Close)
                                  - [x, y, z] point to [a, b, c] point 
                                    within the workspace to drop the 
                                    object (Gripper Open)
    -------------------------------------------------------------------- 
    '''
    RC.communication_begin()
    RC.joints_homing()
    RC.update_forward_kinematics()
    time.sleep(1.5)

    pickup_pos = [215, 120, -60]
    dropoff_pos = [-350, -280, 250]

    RC.execute_pick_and_place(pickup_pos, dropoff_pos)
    RC.communication_end()