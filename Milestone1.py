
import modern_robotics as mr
import numpy as np 
import csv

def NextState(curr_config, cntr_spds, dt, speed_lim):
    """Given a current configuration, calculate a new configuration after a cetain time step
    Inputs
    :param curr_config: A 12-vector representing the current configuration of the robot 
                        (3 variables for the chassis configuration, 5 variables for the arm configuration, and 4 variables for the wheel angles).
    :param cntr_spds: A 9-vector of controls indicating the arm joint speeds \dot{\theta} (5 variables) and the wheel speeds u (4 variables).
    :param dt: A timestep Δt.
    :param speed_lim: A positive real value indicating the maximum angular speed of the arm joints and the wheels. 

    Output
    :param updated_config: A 12-vector representing the configuration of the robot time Δt later.
    """

    r = 0.0475 # The radius of each wheel is r = 0.0475 meters.
    w = 0.3/2 # The side-to-side distance between wheels is 2w = 0.3 meters
    l = 0.47/2 # The forward-backward distance between the wheels is 2l = 0.47 meters

    # Make sure the control speeds are withing the speed limit
    for j in range(len(cntr_spds)):
        if cntr_spds[j] > speed_lim:
            cntr_spds[j] = speed_lim
        
        elif cntr_spds[j] < -speed_lim:
            cntr_spds[j] = -speed_lim

    # Split the current config into seperate config arrays
    chass_config = np.array(curr_config[0:3]) # 3 for the chassis configuration
    arm_config = np.array(curr_config[3:8]) # 5 for the arm configuration
    wheel_config = np.array(curr_config[8:12]) # 4 variables for the wheel angles

    # Split joint speeds into arm and and wheel speeds
    arm_spds = np.array(cntr_spds[0:5]) # 5 for the arm joint speeds
    wheel_spds = np.array(cntr_spds[5:9]) # 4 for the wheel joint speeds

    # Use Euler Step to compute next arm joint and wheel angles
    new_arm_angs = arm_config + (arm_spds * dt)
    new_wheel_angs = wheel_config + (wheel_spds * dt)

    # Calculate new chassis config using odometry
    F = (r/4) * np.array([[(-1 / (l+w)), (1 / (l+w)), (1 / (l+w)), (-1 / (l+w))], 
                          [           1,           1,           1,            1], 
                          [          -1,           1,          -1,            1]])

    # Calculate the body twist
    delta_theta = wheel_spds * dt
    V_b = np.dot(F, delta_theta)

    # Create a 6 dimensional body twist
    wb_z = V_b[0]
    vb_x = V_b[1]
    vb_y = V_b[2]
    V_b6 = np.array([0, 0, wb_z, vb_x, vb_y, 0])

    # Calculate the SE(3) of the new chassis frame relative to the inital frame
    theta = curr_config[0]
    x = curr_config[1]
    y =curr_config[2]
    z =  0.0963 # height of the {b} frame above the floor
    T_sb = np.array([[np.cos(theta), -np.sin(theta), 0, x], 
                     [np.sin(theta),  np.cos(theta), 0, y],
                     [            0,              0, 1, z],
                     [            0,              0, 0, 1]])

    
    se_3 = mr.VecTose3(V_b6)
    T_bb = mr.MatrixExp6(se_3)
    T_sb_new = np.dot(T_sb, T_bb)

    # Create the new configuration
    updated_config = np.array([np.arccos(T_sb_new[0,0]), T_sb_new[0,3], T_sb_new[1,3], 
                               new_arm_angs[0], new_arm_angs[1], new_arm_angs[2], new_arm_angs[3], new_arm_angs[4], 
                               new_wheel_angs[0], new_wheel_angs[1], new_wheel_angs[2], new_wheel_angs[3], 0])

    return updated_config

def write_csv(traj_1, N):
    """
    Given a configuration, create a line in a csv file
    """
    with open('update_config.csv', 'w') as csvfile:
        csv_writer = csv.writer(csvfile)
        for i in range(N):
            csv_writer.writerow([traj_1[i][0], traj_1[i][1], traj_1[i][2], 
                                 traj_1[i][3], traj_1[i][4], traj_1[i][5], 
                                 traj_1[i][6], traj_1[i][7], traj_1[i][8], 
                                 traj_1[i][9], traj_1[i][10], traj_1[i][11]])

# Test inital conditions
curr_config = np.array([0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.])
cntr_spds = np.array([0., 0., 0., 0., 0., -10., 10., 10., -10.])
dt = 0.01
speed_lim = 12.3

# Call function
update = NextState(curr_config, cntr_spds, dt, speed_lim)

# Write to csv
total_traj = []
curr_traj = np.concatenate((curr_config, 0), axis = None)
# print(f"len of curr traj is {len(curr_traj)}")
total_traj.append(curr_traj.tolist())

N = int(1/dt)
for i in range(N):
    next_config = NextState(curr_config, cntr_spds, dt, speed_lim)
    curr_config = next_config
    curr_traj = curr_config
    total_traj.append(curr_traj.tolist())

# Write a csv file
# write_csv(total_traj, N)