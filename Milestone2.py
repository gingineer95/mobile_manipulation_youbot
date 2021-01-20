import numpy as np
import modern_robotics as mr
from modern_robotics import ScrewTrajectory, CartesianTrajectory
import math
from numpy import transpose, dot
import csv

def TrajectoryGenerator(Tse_o, Tsc_o, Tsc_f, Tce_g, Tce_s, k):
    """
    Generates the reference trajectory for the end-effector frame {e} for eight concatenated trajectory segments

    :param Tse_o: Initial configuration of end effector
    :param Tsc_o: cube's initial configuration
    :param Tsc_f: cube's desired final configuration
    :param Tce_g: end-effector's configuration relative to the cube when it is grasping the cube
    :param Tce_s: end-effector's standoff configuration above the cube, before and after grasping, relative to the cube
    :param k: number of trajectory reference configurations per 0.01 seconds
    """
    tot_traj = []

    gripper_state = [] # Create an empty list for the gripper states
    Tf = 8 # Total time of motion in seconds
    N = int((Tf * k) / 0.01) # The number of points in the discrete representation of the trajectory
    method = 5 # The time-scaling method

    Tse_s_o = Tsc_o.dot(Tce_s) # Standoff ee frame realtive to space for inital box config
    standoff_init = Tse_s_o

    Tse_s_f = Tsc_f.dot(Tce_s) # Standoff ee frame realtive to space for final box config
    standoff_final = Tse_s_f

    Tse_g_o = Tsc_o.dot(Tce_g) # Gripper position relative to the inital cube config
    gripper_init = Tse_g_o

    Tse_g_f = Tsc_f.dot(Tce_g) # Gripper position relative to the final cube config
    gripper_final = Tse_g_f

    ##############  Move the gripper from its initial configuration to a "standoff" configuration a few cm above the block
    ## Segment1 ##  Start at the set 'home' position
    ##############  End at the ee standoff position
    Xstart_home = Tse_o
    Xend_standoff = standoff_init
    traj_1 = ScrewTrajectory(Xstart_home,Xend_standoff,Tf,N,method)
    gripper1 = 0
    gripper_state.append(gripper1)
    tot_traj = mat_list(tot_traj,traj_1,N,gripper1)

    ##############  Move the gripper down to the grasp position
    ## Segment2 ##  Start at the ee standoff position
    ##############  End at the ee gripper position
    Tf2 = 1
    Xstart_standoff = standoff_init
    Xend_gripper1 = gripper_init
    traj_2 = ScrewTrajectory(Xstart_standoff,Xend_gripper1,Tf2,N,method)
    gripper2 = 0
    gripper_state.append(gripper2)
    tot_traj = mat_list(tot_traj,traj_2,N,gripper2)

    ##############  Closing of the gripper
    ## Segment3 ##  Start at the ee gripper position
    ##############  End at the ee gripper position
    Xstart_gripper1 = gripper_init 
    Xend_gripper1 = gripper_init
    traj_3 = ScrewTrajectory(Xstart_gripper1,Xend_gripper1,Tf2,N,method)  
    gripper3 = 1
    gripper_state.append(gripper3)
    tot_traj = mat_list(tot_traj,traj_3,N,gripper3)

    ##############  A trajectory to move the gripper back up to the "standoff" configuration
    ## Segment4 ##  Start at the ee gripper position
    ##############  End at the ee standoff position 
    Xstart_gripper1 = gripper_init
    Xend_standoff = standoff_init
    traj_4 = ScrewTrajectory(Xstart_gripper1,Xend_standoff,Tf2,N,method)
    gripper4 = 1
    gripper_state.append(gripper4)
    tot_traj = mat_list(tot_traj,traj_4,N,gripper4)

    ##############  A trajectory to move the gripper to a "standoff" configuration above the final configuration
    ## Segment5 ##  Start at the initial ee standoff position
    ##############  End at the final ee standoff position
    Tf3 = 5
    Xstart_standoff1 = standoff_init
    Xend_standoff2 = standoff_final
    traj_5 = ScrewTrajectory(Xstart_standoff1,Xend_standoff2,Tf3,N,method)
    gripper5 = 1
    gripper_state.append(gripper5)
    tot_traj = mat_list(tot_traj,traj_5,N,gripper5)

    ##############  A trajectory to move the gripper to the final configuration of the object
    ## Segment6 ##  Start at the ee standoff position
    ##############  End at the ee gripper position
    Xstart_standoff2 = standoff_final
    Xend_gripper2 = gripper_final
    traj_6 = ScrewTrajectory(Xstart_standoff2,Xend_gripper2,Tf2,N,method)
    gripper6 = 1
    gripper_state.append(gripper6)
    tot_traj = mat_list(tot_traj,traj_6,N,gripper6)

    ##############  Opening of the gripper
    ## Segment7 ##  Start at the ee gripper position
    ##############  End at the ee gripper position
    Xstart_gripper2 = gripper_final
    Xend_gripper2 = gripper_final 
    traj_7 = ScrewTrajectory(Xstart_gripper2,Xend_gripper2,Tf2,N,method) 
    gripper7 = 0
    gripper_state.append(gripper7)
    tot_traj = mat_list(tot_traj,traj_7,N,gripper7)

    ##############  A trajectory to move the gripper to the final configuration of the object
    ## Segment8 ##  Start at the ee gripper position
    ##############  End at the ee standoff position
    Xstart_gripper2 = gripper_final
    Xend_standoff2 = standoff_final
    traj_8 = ScrewTrajectory(Xstart_gripper2,Xend_standoff2,Tf2,N,method)
    gripper8 = 0
    gripper_state.append(gripper8)
    tot_traj = mat_list(tot_traj,traj_8,N,gripper8)

    # write_csv(N, traj_1, traj_2, traj_3, traj_4, traj_5, traj_6, traj_7, traj_8, gripper_state)

    return tot_traj

def mat_list(tot_traj,ee_traj,N,gripper):
	'''
	Convert tranformation matrix to list
	'''
	foo = np.zeros((N,13),dtype = float)

	for i in range(N):
		foo[i][0] = ee_traj[i][0][0]
		foo[i][1] = ee_traj[i][0][1]
		foo[i][2] = ee_traj[i][0][2]
		foo[i][3] = ee_traj[i][1][0]
		foo[i][4] = ee_traj[i][1][1]
		foo[i][5] = ee_traj[i][1][2]
		foo[i][6] = ee_traj[i][2][0]
		foo[i][7] = ee_traj[i][2][1]
		foo[i][8] = ee_traj[i][2][2]
		foo[i][9] = ee_traj[i][0][3]
		foo[i][10] = ee_traj[i][1][3]
		foo[i][11] = ee_traj[i][2][3]
		foo[i][12] = gripper
		tot_traj.append(foo[i].tolist())
	
	return tot_traj


def write_csv(N, traj_1, traj_2, traj_3, traj_4, traj_5, traj_6, traj_7, traj_8, gripper_state):
    with open('ee_trajectory.csv', 'w') as csvfile:
        csv_writer = csv.writer(csvfile)
        for i in range(N):
            csv_writer.writerow([traj_1[i][0][0], traj_1[i][0][1], traj_1[i][0][2], 
                                 traj_1[i][1][0], traj_1[i][1][1], traj_1[i][1][2], 
                                 traj_1[i][2][0], traj_1[i][2][1], traj_1[i][2][2], 
                                 traj_1[i][0][3], traj_1[i][1][3], traj_1[i][2][3], gripper_state[0]])
        for i in range(N):
            csv_writer.writerow([traj_2[i][0][0], traj_2[i][0][1], traj_2[i][0][2], 
                                 traj_2[i][1][0], traj_2[i][1][1], traj_2[i][1][2], 
                                 traj_2[i][2][0], traj_2[i][2][1], traj_2[i][2][2], 
                                 traj_2[i][0][3], traj_2[i][1][3], traj_2[i][2][3], gripper_state[1]])
        for i in range(N):
            csv_writer.writerow([traj_3[i][0][0], traj_3[i][0][1], traj_3[i][0][2], 
                                 traj_3[i][1][0], traj_3[i][1][1], traj_3[i][1][2], 
                                 traj_3[i][2][0], traj_3[i][2][1], traj_3[i][2][2], 
                                 traj_3[i][0][3], traj_3[i][1][3], traj_3[i][2][3], gripper_state[2]])
        for i in range(N):
            csv_writer.writerow([traj_4[i][0][0], traj_4[i][0][1], traj_4[i][0][2], 
                                 traj_4[i][1][0], traj_4[i][1][1], traj_4[i][1][2], 
                                 traj_4[i][2][0], traj_4[i][2][1], traj_4[i][2][2], 
                                 traj_4[i][0][3], traj_4[i][1][3], traj_4[i][2][3], gripper_state[3]])
        for i in range(N):
            csv_writer.writerow([traj_5[i][0][0], traj_5[i][0][1], traj_5[i][0][2], 
                                 traj_5[i][1][0], traj_5[i][1][1], traj_5[i][1][2], 
                                 traj_5[i][2][0], traj_5[i][2][1], traj_5[i][2][2], 
                                 traj_5[i][0][3], traj_5[i][1][3], traj_5[i][2][3], gripper_state[4]])
        for i in range(N):
            csv_writer.writerow([traj_6[i][0][0], traj_6[i][0][1], traj_6[i][0][2],
                                 traj_6[i][1][0], traj_6[i][1][1], traj_6[i][1][2], 
                                 traj_6[i][2][0], traj_6[i][2][1], traj_6[i][2][2], 
                                 traj_6[i][0][3], traj_6[i][1][3], traj_6[i][2][3], gripper_state[5]])
        for i in range(N):
            csv_writer.writerow([traj_7[i][0][0], traj_7[i][0][1], traj_7[i][0][2], 
                                 traj_7[i][1][0], traj_7[i][1][1], traj_7[i][1][2], 
                                 traj_7[i][2][0], traj_7[i][2][1], traj_7[i][2][2], 
                                 traj_7[i][0][3], traj_7[i][1][3], traj_7[i][2][3], gripper_state[6]])
        for i in range(N):
            csv_writer.writerow([traj_8[i][0][0], traj_8[i][0][1], traj_8[i][0][2], 
                                 traj_8[i][1][0], traj_8[i][1][1], traj_8[i][1][2], 
                                 traj_8[i][2][0], traj_8[i][2][1], traj_8[i][2][2], 
                                 traj_8[i][0][3], traj_8[i][1][3], traj_8[i][2][3], gripper_state[7]])


if __name__ == '__main__':

    Tse_o = np.array([[0,0,1,0],
                      [0,1,0,0],
                      [-1,0,0,0.5],
                      [0,0,0,1]])

    Tsc_o = np.array([[1,0,0,1],
                      [0,1,0,0],
                      [0,0,1,0.025],
                      [0,0,0,1]])

    Tsc_f = np.array([[0,1,0,0],
                      [-1,0,0,-1],
                      [0,0,1,0.025],
                      [0,0,0,1]])

    Tce_g = np.array([[-np.sqrt(2)/2,0,np.sqrt(2)/2,0],
                      [0,1,0,0],
                      [-np.sqrt(2)/2,0,-np.sqrt(2)/2,0],
                      [0,0,0,1]])

    Tce_s = np.array([[-np.sqrt(2)/2,0,np.sqrt(2)/2,0],
                      [0,1,0,0],
                      [-np.sqrt(2)/2,0,-np.sqrt(2)/2,0.1],
                      [0,0,0,1]])

    k = 1

    TrajectoryGenerator(Tse_o, Tsc_o, Tsc_f, Tce_g, Tce_s, k)
