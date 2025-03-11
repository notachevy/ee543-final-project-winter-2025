from robot_controller_win import robot_controller


def main():
    RC = robot_controller()
    RC.communication_begin()

    RC.joints_homing()

    RC.communication_end()


if __name__ == "__main__":
    main()