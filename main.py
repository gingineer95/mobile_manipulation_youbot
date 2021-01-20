import modern_robotics as mr
from Milestone1 import NextState
from Milestone2 import TrajectoryGenerator
from Milestone3 import FeedbackControl
import numpy as np
import matplotlib.pyplot as plt
import logging
import csv

""" This file incorporates all the milestone components and runs as one smooth program
    To undertsand what each milestone does, please check out the appropriate python files

    ** Uses Python3 **

    To Run:
    In your /code directory, run python3 main.py
    Uncomment the appropriate log and csv files starting on lines 20, 153 and 183
    Uncomment the approriate gains stating at line 212
    If you wish to change the box initial and final conditions, start at line 203
"""

# initalize log file
# file_name = "best.log"
file_name = "newTask.log"
# file_name = "overshoot.log"
logging.basicConfig(filename = file_name, level = logging.DEBUG)

def main(Tsc_0, Tsc_f, Kp, Ki, robot_config):
    """ This function takes a set of inputs and calls the NextState, TrajectoryGenerator and FeedbackControl
        functions to determine the error for a couple of different controller states.

        Input:
        :param Tsc_0: Inital configuration of a cube
        :param Tsc_f: Final configuration of a cube
        :param Kp, Ki: Gains for the feedback controller
        :param robot_config: Initial configuration of robot

        Output:
        :param robo_traj: List of 1600 x 13, each row corresponding to a trajectory list
        :param Xerr_list: List of 1599 x 6, each row corresponding to an error list
        - A log file
        - A trajectory csv
        - An error csv file

    """
    N = 6000
    speed_lim = 10
    dt = 0.01
    k = 1

    # The fixed offset from the chassis frame {b} to the base frame of the arm {0}
    Tb0 = np.array([[ 1, 0, 0, 0.1662],
                    [ 0, 1, 0,      0], 
                    [ 0, 0, 1, 0.0026], 
                    [ 0, 0, 0,      1]])

    # The initial configuration of the end-effector in the reference trajectory:
    Tse_0 = np.array([[ 0,0,1,  0],
                      [ 0,1,0,  0],
                      [-1,0,0,0.5],
                      [ 0,0,0,  1]])

    # The end-effector's configuration relative to the cube when it is grasping the cube
    Tce_g = np.array([[-np.sqrt(2)/2,0,np.sqrt(2)/2,0],
                      [0            ,1,           0,0],
                      [-np.sqrt(2)/2,0,-np.sqrt(2)/2,0],
                      [0            ,0,            0,1]])

    # The end-effector's standoff configuration above the cube, before and after grasping, relative to the cube
    Tce_s = np.array([[-np.sqrt(2)/2,0, np.sqrt(2)/2,   0],
                      [0            ,1,            0,   0],
                      [-np.sqrt(2)/2,0,-np.sqrt(2)/2, 0.1],
                      [0            ,0,            0,  1]])
    
    # At the home configuration...
    M0e = np.array([[ 1, 0, 0,  0.033],
                    [ 0, 1, 0,      0], 
                    [ 0, 0, 1, 0.6546], 
                    [ 0, 0, 0,      1]])
    
    B_list = np.array([[0,  0,  1,       0, 0.033, 0], 
                       [0, -1,  0, -0.5076,     0, 0], 
                       [0, -1,  0, -0.3526,     0, 0], 
                       [0, -1,  0, -0.2176,     0, 0], 
                       [0,  0,  1,       0,     0, 0]]).T

    # Create empty lists for trajectories and errors
    robo_traj = []
    Xerr_list = []

    integral = np.zeros((6,), dtype = float)
    robo_traj.append(robot_config.tolist())

    # Generate trajectories from Milestone1
    traj = TrajectoryGenerator(Tse_0, Tsc_0, Tsc_f, Tce_g, Tce_s, k)
    print("Working...")

    for i in range (N-1):
        # update joint angles
        theta_list = robot_config[3:8]

        theta = robot_config[0]
        x = robot_config[1]
        y = robot_config[2]
        z =  0.0963 # height of the {b} frame above the floor

        Tsb = np.array([[np.cos(theta), -np.sin(theta), 0, x], 
                        [np.sin(theta),  np.cos(theta), 0, y],
                        [            0,              0, 1, z],
                        [            0,              0, 0, 1]])

        T0e = mr.FKinBody(M0e, B_list, theta_list)

        T_be = np.dot(Tb0, T0e)
        X = np.dot(Tsb, T_be)

        curr_traj = traj[i]
        next_traj = traj[i+1]
        X_d = np.array([[curr_traj[0], curr_traj[1], curr_traj[2], curr_traj[9]] , 
                        [curr_traj[3], curr_traj[4], curr_traj[5], curr_traj[10]],
                        [curr_traj[6], curr_traj[7], curr_traj[8], curr_traj[11]],
                        [           0,            0,            0,             1]])

        X_d_nx = np.array([[next_traj[0], next_traj[1], next_traj[2], next_traj[9]] , 
                          [next_traj[3], next_traj[4], next_traj[5], next_traj[10]],
                          [next_traj[6], next_traj[7], next_traj[8], next_traj[11]],
                          [           0,            0,            0,             1]])

        # Calculate the control speeds and error value from Milestone3
        c_spds, Xerr = FeedbackControl(X, X_d, X_d_nx, Kp, Ki, dt, robot_config, integral)
        Xerr_list.append(Xerr.tolist())

        # Flip command speeds for the NextState
        w_spds = c_spds[:4]
        a_spds = c_spds[4:9]
        cntr_spds = np.concatenate((a_spds, w_spds), axis = None)

        curr_config = robot_config[:12]
        # Determine the next trajectory from Milestone2
        robot_config = NextState(curr_config, cntr_spds, dt, speed_lim)

        traj1 = traj[i][12]
        robo_curr_traj = np.concatenate((robot_config[:12], traj1), axis = None)
        robo_traj.append(robo_curr_traj.tolist())

    # Write the animation file
    logging.debug("Generating animation csv file")
    write_csv(robo_traj, N)
    print(f"Length of one traj is {len(robo_traj[100])}")

    # save the log file
    logging.debug("Generate the X error file")
    # np.savetxt("Xerr_best.csv", Xerr_list, delimiter = ",")
    np.savetxt("Xerr_newTask.csv", Xerr_list, delimiter = ",")
    # np.savetxt("Xerr_overshoot.csv", Xerr_list, delimiter = ",")

    # Plot error
    logging.debug("Plot error")
    x_axis = np.linspace(0, 14, N-1)
    y_axis = np.asarray(Xerr_list)

    plt.plot(x_axis, y_axis[:,0])
    plt.plot(x_axis, y_axis[:,1])
    plt.plot(x_axis, y_axis[:,2])
    plt.plot(x_axis, y_axis[:,3])
    plt.plot(x_axis, y_axis[:,4])
    plt.plot(x_axis, y_axis[:,5])

    plt.title("X error plot")
    plt.xlabel("Time in sec")
    plt.ylabel("Error")
    plt.legend([r'$Xerr[1]$',r'$Xerr[2]$',r'$Xerr[3]$',
                r'$Xerr[4]$',r'$Xerr[5]$',r'$Xerr[6]$'])
    plt.show()

    logging.debug("All done!")


def write_csv(traj_1, N):
    """
    Given a configuration, create a line in a csv file
    """
    # with open('best_traj.csv', 'w') as csvfile:
    with open('newTask_traj.csv', 'w') as csvfile:
    # with open('overshoot_traj.csv', 'w') as csvfile:
        csv_writer = csv.writer(csvfile)
        for i in range(N):
            csv_writer.writerow([traj_1[i][0], traj_1[i][1], traj_1[i][2], traj_1[i][3], 
                                 traj_1[i][4], traj_1[i][5], traj_1[i][6], traj_1[i][7], 
                                 traj_1[i][8], traj_1[i][9], traj_1[i][10], traj_1[i][11], traj_1[i][12]])

# for default box config
Tsc_0 = np.array([[1,0,0,1],
                  [0,1,0,0],
                  [0,0,1,0.025],
                  [0,0,0,1]])

Tsc_f = np.array([[0,1,0,0],
                 [-1,0,0,-1],
                 [0,0,1,0.025],
                 [0,0,0,1]])

# for new task box config
# Tsc_0 = np.array([[1,0,0,1.5],
#                   [0,1,0,0.5],
#                   [0,0,1,0.025],
#                   [0,0,0,1]])

# Tsc_f = np.array([[0,1,0,0.25],
#                  [-1,0,0,-1.25],
#                  [0,0,1,0.025],
#                  [0,0,0,1]])

# Best values
kp = 4
ki = 2

# newTask values
# kp = 7.2
# ki = 1

# Overshoot values
# kp = 2
# ki = 5

Kp = np.eye(6) * kp
Ki = np.eye(6) * ki

robot_config = np.array([0.1,0.1,0.2,0,0,0.2,-1.6, 0,0,0,0,0,0])

main(Tsc_0, Tsc_f, Kp, Ki, robot_config)