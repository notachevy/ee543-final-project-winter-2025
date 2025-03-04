"""""""""""""""""""""""""""""

University of Washington, 2024

Author: Tin Chiang

Note: Modified code from Haonan Peng's Raven keyboard controller

Original code: https://github.dev/uw-biorobotics/raven2_CRTK_Python_controller/blob/main/python_controller/run_r2_keyboard_controller.py
"""""""""""""""""""""""""""""

# import time
# import numpy as np
# from pynput import keyboard
# import sys, os
# from robot_controller import robot_controller


# def print_manu():
#     print('  ')
#     print('-----------------------------------------')
#     print('EE543 Arm Keyboard Controller:')
#     print('-----------------------------------------')
#     print('[Exit]: 9')
#     print('[Joint 1    +]: 1 | [Joint 1     -]: q')
#     print('[Joint 2    +]: 2 | [Joint 2     -]: w')
#     print('[Joint 3    +]: 3 | [Joint 3     -]: e')
#     print('[Joint 4    +]: 4 | [Joint 4     -]: r')
#     print('[Grasper Open]: 5 | [Grasper Close]: t')

#     print('-----------------------------------------')
#     print('-----------------------------------------')
#     print('Current command:\n')
#     return None

# def print_no_newline(string):
#     sys.stdout.write("\r" + string)
#     sys.stdout.flush()
#     return None


# # init the Robot Controller
# RC = robot_controller()
# RC.communication_begin()

# # Force homing the robot
# RC.joints_homing()

# keyboard_increment = 0.5

# goals = np.zeros(RC.joint_num)
# speeds = np.ones(RC.joint_num) * 80 # deg/s


# x = 0
# working = 1
# command = False
# print_manu()

# while working==1:

#     #get the keyboard input
#     input_key = keyboard.read_event().name

#     if input_key == '9':
#         # RC.communication_end()
#         os.system('cls' if os.name == 'nt' else 'clear')
#         sys.exit('Closing Keyboard controller')
        

#     elif input_key == '1':
#         print_no_newline(" Moving: Joint 1 +++         ")
#         goals[0] += keyboard_increment
#         command = True


#     elif input_key == 'q':
#         print_no_newline(" Moving: Joint 1 ---         ")
#         goals[0] -= keyboard_increment
#         command = True

#     elif input_key == '2':
#         print_no_newline(" Moving: Joint 2 +++         ")
#         goals[1] += keyboard_increment
#         command = True
              
#     elif input_key == 'w':
#         print_no_newline(" Moving: Joint 2 ---         ")
#         goals[1] -= keyboard_increment
#         command = True

#     elif input_key == '3':
#         print_no_newline(" Moving: Joint 3 +++         ")
#         goals[2] += keyboard_increment
#         command = True
        
#     elif input_key == 'e':
#         print_no_newline(" Moving: Joint 3 ---         ")
#         goals[2] -= keyboard_increment
#         command = True

#     elif input_key == '4':
#         print_no_newline(" Moving: Joint 4 +++         ")
#         goals[3] += keyboard_increment
#         command = True
        
#     elif input_key == 'r':
#         print_no_newline(" Moving: Joint 4 ---         ")
#         goals[3] -= keyboard_increment
#         command = True
    
#     elif input_key == 'h':
#         print_no_newline(" Homing....                  ")
#         goals = RC.robot_homing_joint_poses.copy()
#         command = True
        
#     elif input_key == '5':
#         print_no_newline(" Grasper Open....                  ")
#         RC.gripper_open()
#         # goals = RC.robot_homing_joint_poses.copy()
#         # command = True

#     elif input_key == 't':
#         print_no_newline(" Grasper Close....                  ")
#         RC.gripper_close()
#         # goals = RC.robot_homing_joint_poses.copy()
#         # command = True

#     else:
#         print_no_newline(' Unknown command             ')

    
#     if command:
#         # make sure the goals is within joint limit
#         goals = np.clip(goals, RC.servo_angle_min, RC.servo_angle_max) 
#         sys.stdout.write("\033[1B") # move curser down
#         RC.joints_goto(goals, speeds)
#         sys.stdout.write("\033[1A") # move curser up
#         command = False

import time
import numpy as np
import sys
import os
from pynput import keyboard
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from robot_controller import robot_controller

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
        R = T[0:3, 0:3]  # rotation portion

        # The 3 unit vectors in local frame i:
        x_axis = R[:,0]  # (dx, dy, dz) of local X
        y_axis = R[:,1]
        z_axis = R[:,2]

        # Plot a blue arrow for X
        ax.quiver(
            origin[0], origin[1], origin[2],
            x_axis[0], x_axis[1], x_axis[2],
            color='b', length=axis_length, normalize=True
        )
        # # Plot a green arrow for Y
        # ax.quiver(
        #     origin[0], origin[1], origin[2],
        #     y_axis[0], y_axis[1], y_axis[2],
        #     color='g', length=axis_length, normalize=True
        # )
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
    # Extract the (x,y,z) origin of each joint frame
    points = [T[0:3, 3] for T in transforms]
    points = np.array(points)  # shape: (n+1, 3)
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

    # Unpack coordinates for easy plotting
    xs = link_positions[:,0]
    ys = link_positions[:,1]
    zs = link_positions[:,2]
    ax.cla()

    # Plot it as a line from 0->1->2->3->4->5
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

    print_menu()  # If you want the menu
    print("Press 9 to exit, etc...")

    try:
        while True:
            if last_key is not None:
                current_key = last_key
                last_key = None  # Reset the global key after reading
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
                    print_no_newline("Grasper Open")
                    RC.gripper_open()

                elif current_key == 't':
                    print_no_newline("Grasper Close")
                    RC.gripper_close()

                # If we changed any joint angles, move real robot & update plot
                if command:
                    # Clip angles between -90 and +90
                    goals = np.clip(goals, RC.servo_angle_min, RC.servo_angle_max)

                    # Move the real robot29
                    RC.joints_goto(goals, speeds)

            # Update the simulation plot each loop 
            link_positio9ns = RC.get_link_positions()   # make sure you have get_link_positions in robot_controller
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
    main()
    RC.robotstate_joint_poses = np.array([0, 0, 0, 0])
    RC.update_forward_kinematics()
    print("End Effector Position:", RC.robotstate_endeffector_pose)
    print("End Effector Orientation (deg):", RC.robotState_endeffector_orientation)
    RC.monte_carlo_workspace(N=10000)
