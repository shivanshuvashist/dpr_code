import numpy

def reward_function(params):
    
    # Constants
    STEERING_THRESHOLD = 15 # Adjust as needed
    SPEED_THRESHOLD_LOW = 0.5
    SPEED_THRESHOLD_HIGH = 3
    SPEED_THRESHOLD_RANGE = 2 #SPEED_THRESHOLD_HIGH - SPEED_THRESHOLD_LOW
    WAYPOINTS_COUNT = 1  # Number of waypoints to consider
    upcoming_turn = False
    speed_reward = 1.0
    steering_reward = .5
    
    # Read input parameters	
    steering_angle = abs(params['steering_angle']) # Absolute steering angle
    distance_from_center = params['distance_from_center']
    speed = params['speed']
    all_wheels_on_track = params['all_wheels_on_track']
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    heading = params['heading']
    waypoints_completed = params['steps']
    total_waypoints = len(waypoints)
    waypoints_remaining = total_waypoints - waypoints_completed

    # Calculate desired steering angle based on upcoming waypoints
    next_point = closest_waypoints[1]
    x0, y0 = waypoints[next_point]  # Current position
    lookahead_points = []
    slope_history = 0.0
#    if(waypoints_remaining > 10):

    for i in range(1, WAYPOINTS_COUNT+20):
        lookahead_point = waypoints[(next_point + i) % len(waypoints)]  # Wrap around to start if needed
        lookahead_points.append(lookahead_point)

    # Fit a polynomial to the lookahead points
    x_vals = [point[0] for point in lookahead_points]
    y_vals = [point[1] for point in lookahead_points]
    
    # Calculate slope and decide reward for speed
    slope = (y_vals[19]-y_vals[0])/(x_vals[19]-x_vals[0])
    
    if(slope_history<0):
        slope_history = 0 - slope_history
        if(slope < 0):
            slope = 0 - slope + slope_history
        else:
            slope = slope + slope_history
        
    percent_change = ((slope_history - slope)/(slope_history + 1e-3)) * 100
    
    print(str("percent_change: " + str(percent_change) + ":::"))
    if(abs(percent_change) > 80):
        upcoming_turn = True
    else:
        upcoming_turn = False
    
    slope_history = slope
    
    
#    if(abs(slope) > 50 or abs(slope) < .1):
#        upcoming_turn = True

    print(str("upcoming_turn: " + str(upcoming_turn) + ":::"))
    print(str("SLOPE: " + str(abs(slope)) + ":::"))
    
    desired_steering_angle = numpy.arctan(slope) * 180 / numpy.pi

    print(str("desired_steering_angle: " + str(desired_steering_angle)) + ":::" + str("Yaw : " + str(steering_angle) + ":::"))
    angle_diff = abs(desired_steering_angle - heading)

    # Normalize angle difference
    if angle_diff > 180:
        angle_diff = 360 - angle_diff
        
    print(str("angle_diff: " + str(angle_diff) + ":::"))

    if(abs(desired_steering_angle)>15):
        # Calculate steering angle reward based on angle difference
        steering_reward = 1.0 - (angle_diff / 180.0)
        # Apply calibration based on speed
        steering_to_speed_calibration = 1.0 - (0.5 * (speed / SPEED_THRESHOLD_HIGH))  # Adjust as needed
        steering_reward *= steering_to_speed_calibration

    #heading = (heading + 360) % 360  # Convert angle to positive value
    
    print(str("heading: " + str(heading) + ":::"))
    
    # Calculate distance from center reward
    MAX_REWARD_DISTANCE = 0.4 * params['track_width']

    distance_reward = 1.0 - (distance_from_center / MAX_REWARD_DISTANCE)
    
    # Calculate speed reward
    if upcoming_turn==False and all_wheels_on_track and SPEED_THRESHOLD_LOW <= speed <= SPEED_THRESHOLD_HIGH:
        speed_reward = (speed - SPEED_THRESHOLD_LOW) / SPEED_THRESHOLD_RANGE
    elif upcoming_turn==True and all_wheels_on_track and speed > 1.5:
        # Penalize if car steer too much to prevent zigzag
        ABS_STEERING_THRESHOLD = 20.0
        speed_reward *= 0.95  # Penalize if too fast or off track
        
    # Reward for being aligned with straight sections of the track
#    if angle_diff < 10:  # Define a threshold for straightness
#        speed_reward += 0.1  # Increase reward for being straight
        
    # Final reward is a combination of all rewards
    
    reward = 0.4 * steering_reward + 0.3 * distance_reward + 0.3 * speed_reward

    print(str("reward: " + str(reward)) + ":::" + "steering_reward: " + str(steering_reward) + ":::" + "distance_reward: " + str(distance_reward) + ":::" + "speed_reward: " + str(speed_reward))
    
    return float(reward)
