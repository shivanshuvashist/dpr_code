import math
import numpy as np

# Har Har Mahadev
# This is a good Model for complex tracks
# LR 0.00001
# ENTROPY 0.001
# Experience 30

def replace_spikes(angle_list):
    for i in range(1,len(angle_list)-1):
        if(abs(angle_list[i])>30):
            angle_list[i] = angle_list[i-1]
    return angle_list
    
def average_of_consecutive_entries(input_list, factor):
    """
    Compute the average of consecutive entries in a list
    
    Parameters:
    input_list (list): Input list of values
    
    Returns:
    list: List containing the average of consecutive entries
    """
    input_list = [round(abs(val),4) for val in input_list]
    for i in range(0,factor-1):
        result = []
        for i in range(len(input_list) - 1):
            result.append((input_list[i] + input_list[i + 1]) / 2)
        result.append(input_list[-1])  # Append the last element unchanged
        input_list = result[:]
        
    return input_list
    
def calculate_speed(d_steering_angle, desired_speed, track_length):
    m = (desired_speed - 0.5)/(track_length/2)
    # k is max speed & m is rate of change of speed
    steering_angle = abs(d_steering_angle)
    
    speed = 4 - m * d_steering_angle
    
    # Limit the speed to the range of 0.5 to 4 units
    speed = max(speed, 1.5)
    
    return speed
    
  
def calculate_racing_line(waypoints):
    racing_line = []
    angle_list = []
    heading_angle_list = []
    
    for i in range(len(waypoints)):
        if i == 0:
            # Start and end points stay the same
            racing_line.append(waypoints[i])
            
            current_point = waypoints[i]
            next_point = waypoints[i + 1]
            
            vector_b = np.array(next_point) - np.array(current_point)
            angle = np.arctan2(vector_b[1], vector_b[0])
            angle = np.degrees(angle)
            
            if(abs(angle)>150 and abs(angle)<=180):
                angle = 180.0 - abs(angle)
            elif(abs(angle)>250 and abs(angle)<=360):
                angle = 360.0 - abs(angle)
            angle = round(angle,4)
            
            angle_list.append(angle)
            heading_angle_list.append(angle)
        elif i == len(waypoints) - 1:
            racing_line.append(waypoints[i])
            
            current_point = waypoints[i]
            prev_point = waypoints[i - 1]
            
            vector_a = np.array(current_point) - np.array(prev_point)
            angle = np.arctan2(vector_a[1], vector_a[0])
            angle = np.degrees(angle)
            
            if(abs(angle)>150 and abs(angle)<=180):
                angle = 180.0 - abs(angle)
            elif(abs(angle)>250 and abs(angle)<=360):
                angle = 360.0 - abs(angle)
            angle = round(angle,4)

            angle_list.append(angle)
            heading_angle_list.append(angle)
        else:
            prev_point = waypoints[i - 1]
            current_point = waypoints[i]
            next_point = waypoints[i + 1]
            
            # Calculate direction vectors
            vector_a = np.array(current_point) - np.array(prev_point)
            vector_b = np.array(next_point) - np.array(current_point)
            
            # Calculate angles
            angle = np.arctan2(vector_b[1], vector_b[0]) - np.arctan2(vector_a[1], vector_a[0])
            heading_angle_rad = np.arctan2(vector_a[1], vector_a[0])
            heading_angle = np.degrees(heading_angle_rad)
            angle = np.degrees(angle)
            
            if(abs(angle)>150 and abs(angle)<=180):
                angle = 180.0 - abs(angle)
            elif(abs(angle)>250 and abs(angle)<=360):
                angle = 360.0 - abs(angle)
            angle = round(angle,4)
            heading_angle = round(heading_angle, 4)


            angle_list.append(angle)
            heading_angle_list.append(heading_angle)
            
            # Adjust point slightly towards the inside of the curve
            if angle > 0:
                if(abs(angle) > 30):
                    adjusted_point = (
                        current_point[0] - 0.2 * (current_point[0] - prev_point[0]),
                        current_point[1] - 0.2 * (current_point[1] - prev_point[1])
                    )
                else:
                    adjusted_point = (
                        current_point[0] - 0.1 * (current_point[0] - prev_point[0]),
                        current_point[1] - 0.1 * (current_point[1] - prev_point[1])
                    )
            else:
                if(abs(angle) > 30):
                    adjusted_point = (
                        current_point[0] + 0.2 * (next_point[0] - current_point[0]),
                        current_point[1] + 0.2 * (next_point[1] - current_point[1])
                    )
                else:
                    adjusted_point = (
                        current_point[0] + 0.1 * (next_point[0] - current_point[0]),
                        current_point[1] + 0.1 * (next_point[1] - current_point[1])
                    )
            
            racing_line.append(adjusted_point)
    
    return racing_line, angle_list, heading_angle_list
    

def reward_function(params):
    racing_line = []
    reward = 1.0
    # Define the speed limits
    min_speed_limit = 1
    max_speed_limit = 4
    reward_factor = 1
    point_list = []
    non_abs_angles = []
    max_limit_distance = 0.55 #change it based on track width. Increase in tracks where you want to cut more
    temp_angle_list = []
    STEERING_THRESHOLD = 25  # Allow a bit more steering, change based on circuit

    # Define the waypoints for the track
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    
    # Extract necessary parameters from 'params'
    x = params['x']
    y = params['y']
    distance_from_center = params['distance_from_center']
    track_width = params['track_width']
    heading = round(params['heading'], 4)
    speed = round(params['speed'],4)
    steering_angle = round(params['steering_angle'],4)
    all_wheels_on_track = params['all_wheels_on_track']
    progress = params['progress']
    steps = params['steps']
    track_length = params['track_length']
    is_offtrack = params['is_offtrack']
    
    no_waypoints = len(waypoints)
    
    # Calculate the racing line based on waypoints
    racing_line, angle_list, heading_angle_list = calculate_racing_line(waypoints)
    
    angle_list = replace_spikes(angle_list)
    
    angle_list = average_of_consecutive_entries(angle_list, 3)
    
    angle_list[0] = 0.0
    angle_list[len(angle_list)-1] = 0.0
    angle_list[len(angle_list)-2] = 0.0
    angle_list[len(angle_list)-3] = 0.0
        
    print("waypoints", racing_line)
    print("angle_list", angle_list)
    print("heading_angle_list", heading_angle_list)
    
    vehicle_position = [x,y]
    
    # Find the closest point on the racing line
    min_dist = float('inf')
    min_dist_point = 0
    cnt = 0
    closest_racing_point = None
    for point in racing_line:
        cnt = cnt + 1
        dist = math.sqrt((x - point[0]) ** 2 + (y - point[1]) ** 2)
        if dist < min_dist:
            min_dist = dist
            closest_racing_point = point
            min_dist_point = cnt
    
    print("closest_racing_point", closest_racing_point)
    print("min_dist_point", min_dist_point)
    # Total num of steps we want the car to finish the lap, it will vary depends on the track length
    TOTAL_NUM_STEPS = len(waypoints) * steps
        
    steering_factor = 30/180
    zero_angle = min(angle_list[0:5])
    
    if(min_dist_point==0):
        current_angle = angle_list[min_dist_point+1]
        current_heading_angle = heading_angle_list[min_dist_point+1]
    elif(min_dist_point<(len(angle_list)-1)):
        current_angle = angle_list[min_dist_point]
        current_heading_angle = heading_angle_list[min_dist_point]
    else:
        current_angle = abs(zero_angle)
        current_heading_angle = abs(zero_angle)
    
    if(abs(current_angle)>abs(steering_angle)):
        angle_diff = abs(current_angle) - abs(steering_angle)
    else:
        angle_diff = abs(steering_angle) - abs(current_angle)
    
    
    desired_steering_reward = round(1 - (angle_diff/30),4)
    
    # Normalize angle difference
    if abs(current_heading_angle) > 180:
        current_heading_angle = 360 - abs(current_heading_angle)
        
    if(abs(current_heading_angle)>abs(heading)):
        heading_rew = 1 - (abs(current_heading_angle)-abs(heading))/180
    else:
        heading_rew = 1 - (abs(heading)-abs(current_heading_angle))/180
    
    # Calculate reward for following the racing line
    distance_reward = 1.0 - min_dist / (track_width * max_limit_distance)
    
    speed_req = calculate_speed(abs(current_angle), max_speed_limit, track_length)

    # Calculate the progress towards the racing line
    # Lap efficiency reward (no hardcoded track length)
    projected_steps = steps / (progress / 100.0 + 1e-6)
    # Use a baseline (e.g., 300 steps) as expected average lap
    baseline_steps = 150.0
    efficiency_reward = min(1.0, baseline_steps / projected_steps)

    # Calculate the difference between required_speed and current_speed
    if(speed_req>speed):
        speed_difference = abs(speed_req - speed)
    else:
        speed_difference = abs(speed - speed_req)

    speed_reward = 1 - (speed_difference/(max_speed_limit - min_speed_limit))
    
    if distance_from_center > (track_width * (max_limit_distance+0.2)) or is_offtrack:
        return 1e-3  # Penalize track widths outside the desired range
    else:
        # Give additional reward if the car pass every 100 steps faster than expected
        # if (steps % 100) == 0 and progress > ((steps * min_dist_point) / TOTAL_NUM_STEPS) * 100 and desired_steering_reward>0.8 and speed_reward>0.8:
        #     reward_factor = 1.1
        # else:
        #     reward_factor = 0.9
        
        if steps>0 and all_wheels_on_track:
            # Change it to average of all rewards and add a factor so that simar values are returned without multiple if else
            # reward_factor = 1.0
            reward_factor = max_limit_distance + ((desired_steering_reward + speed_reward + distance_reward + heading_rew)/4)

            # reward_factor_new = 1 - abs(desired_steering_reward - heading_rew)
            # reward_factor = reward_factor + reward_factor_new
            # if(desired_steering_reward>0.95 and speed_reward>0.95 and distance_reward>0.95 and heading_rew>0.95):
            #     reward_factor = 1.5
            # elif(desired_steering_reward>0.90 and speed_reward>0.90 and distance_reward>0.90 and heading_rew>0.90):
            #     reward_factor = 1.4
            # elif(desired_steering_reward>0.85 and speed_reward>0.85 and distance_reward>0.85 and heading_rew>0.85):
            #     reward_factor = 1.3
            # elif(desired_steering_reward>0.95 and speed_reward>0.95 and distance_reward>0.95):
            #     reward_factor = 1.2
            # elif(desired_steering_reward>0.90 and speed_reward>0.90 and distance_reward>0.90):
            #     reward_factor = 1.1

        # if(reward_factor_new>0.5 and distance_reward<(max_limit_distance-0.2)):
        #     reward_factor = 0.001
        if(desired_steering_reward<0.6 and distance_reward<(max_limit_distance-0.2)):
            reward_factor *= 0.5

        reward = reward_factor * (0.25 * speed_reward + 0.25 * distance_reward + 0.25 * desired_steering_reward + 0.25 * heading_rew)
    
        print("Steps ", steps)
        print("Steering angle ", steering_angle)
        print("zero angle ", zero_angle)
        print("Speed ", speed)
        print("speed_req ", speed_req)
        print("desired_steering_reward ", desired_steering_reward)
        print("desired_steering_angle ", current_angle)
        print("heading_angle ", current_heading_angle)
        print("heading ", heading)
        print("efficiency_reward ", efficiency_reward)
        print("reward_factor ", reward_factor)
        # print("reward_factor_new ", reward_factor_new)
        print("    " + str(reward_factor) + " + speed_reward " + str(speed_reward) + " + distance_reward " + str(distance_reward) + " + heading reward " + str(heading_rew))
        
        print("vehicle position + ", vehicle_position)
        print("Reward  ", reward)
    
    return float(reward)



# Tried Hyperparameter tuning

# Hyperparameter
# Value
# Gradient descent batch size
# 64
# Entropy
# 0.01
# Discount factor
# 0.98
# Loss type
# Huber
# Learning rate
# 0.0001
# Number of experience episodes between each policy-updating iteration
# 30
# Number of epochs
# 5
# min speed = 2
# Max speed = 4

