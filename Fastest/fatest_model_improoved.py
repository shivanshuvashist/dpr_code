import math
import numpy as np

# Har Har Mahadev
# This is a good Model for complex tracks
# LR 0.00001
# ENTROPY 0.001
# Experience 30
    
def calculate_speed(d_steering_angle, desired_speed, track_length):
    m = (desired_speed - 0.5)/(track_length/2)
    # k is max speed & m is rate of change of speed
    steering_angle = abs(d_steering_angle)
    
    speed = 4 - m * d_steering_angle
    
    # Limit the speed to the range of 0.5 to 4 units
    speed = max(speed, 0.5)
    
    return speed
    
  
def calculate_racing_line(waypoints):
    racing_line = []
    angle_list = []
    heading_angle_list = []
    increment_factor = 1.1
    prev_angle = None
    
    for i in range(len(waypoints)):
        if i == 0:
            # Start and end points stay the same
            current_point = waypoints[i]
            next_point = waypoints[i + 1]
            angle = 0.0
            angle_list.append(0.0)
            ratio = (abs(angle)/(30*50))*increment_factor
            adjusted_point = (
                current_point[0] - (0.005 * (next_point[0])),
                current_point[1] + (0.005 * (next_point[1]))
            )
            heading_angle_list.append(angle)
        elif i == len(waypoints) - 1:
            
            current_point = waypoints[i]
            prev_point = waypoints[i - 1]
            
            vector_a = np.array(current_point) - np.array(prev_point)
            angle = np.arctan2(vector_a[1], vector_a[0])
            angle = np.degrees(angle)
            
            angle = calculate_angles(angle)
            ratio = round((abs(angle)*increment_factor/(30*50)),4)
            angle = round(angle,4)

            adjusted_point = (
                current_point[0] - (0.005 * (prev_point[0])),
                current_point[1] + (0.005 * (prev_point[1]))
            )
            
            angle_list.append(angle)
            heading_angle_list.append(angle)
            increment_factor = increment_factor + 0.001
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
            
            angle = calculate_angles(angle)
            
            
            ratio = round((abs(angle)*increment_factor/(30*50)),4)

            # Adjust point slightly towards the inside of the curve
            if angle > 0:
                adjusted_point = (
                    current_point[0] - (0.005 * (next_point[0])),
                    current_point[1] + (0.005 * (next_point[1]))
                )
            else:
                adjusted_point = (
                    current_point[0] - (0.005 * (prev_point[0])),
                    current_point[1] + (0.005 * (prev_point[1]))
                )
                
            angle = round(angle,4)
            heading_angle = round(heading_angle, 4)

            angle_list.append(angle)
            heading_angle_list.append(heading_angle)
        prev_angle = angle
        racing_line.append(adjusted_point)
    
    return racing_line, angle_list, heading_angle_list
    
def calculate_angles(angle):
    if(abs(angle)>150 and abs(angle)<=180):
        if(angle>0):
            angle = 180.0 - angle
        else:
            angle = 180.0 + angle
    elif(abs(angle)>250 and abs(angle)<=360):
        if(angle>0):
            angle = 360.0 - angle
        else:
            angle = 360.0 + angle
    elif(abs(angle)>70 and abs(angle)<=91):
        if(angle>0):
            angle = 90.0 - angle
        else:
            angle = 90.0 + angle

    return angle
    
def reward_function(params):
    racing_line = []
    reward = 1.0
    # Define the speed limits
    min_speed_limit = 1
    max_speed_limit = 4
    reward_factor = 1
    point_list = []
    non_abs_angles = []
    max_limit_distance = 0.6 #change it based on track width. Increase in tracks where you want to cut more
    temp_angle_list = []
    STEERING_THRESHOLD = 20  # Allow a bit more steering, change based on circuit

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
#    racing_line_temp, angle_list, heading_angle_list = calculate_racing_line(racing_line)
    angle_list = [abs(round(angle,2)) for angle in angle_list]
        
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

    # Calculate the difference between required_speed and current_speed
    if(speed_req>speed):
        speed_difference = abs(speed_req - speed)
    else:
        speed_difference = abs(speed - speed_req)

    speed_reward = 1 - (speed_difference/(max_speed_limit - min_speed_limit))
    
    if distance_from_center > (track_width * (max_limit_distance)) or is_offtrack:
        return 1e-3  # Penalize track widths outside the desired range
    else:
        # Give additional reward if the car pass every 100 steps faster than expected
        # if (steps % 100) == 0 and progress > ((steps * min_dist_point) / TOTAL_NUM_STEPS) * 100 and desired_steering_reward>0.8 and speed_reward>0.8:
        #     reward_factor = 1.1
        # else:
        #     reward_factor = 0.9
        
        if steps>0 and all_wheels_on_track:
            if(desired_steering_reward>0.95 and speed_reward>0.95 and distance_reward>0.95):
                    reward_factor = 1.1
            elif(desired_steering_reward>0.90 and speed_reward>0.90 and distance_reward>0.90):
                    reward_factor = 1.05
            
        if(distance_reward<max_limit_distance):
                reward_factor = 0.001
        
        
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
        
        print("    " + str(reward_factor) + " + speed_reward " + str(speed_reward) + " + distance_reward " + str(distance_reward) + " + heading reward " + str(heading_rew))
        
        print("vehicle position + ", vehicle_position)
        print("Reward  ", reward)
    
    return float(reward)



