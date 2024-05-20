import numpy

def reward_function(params):
    
    # Constants
    STEERING_THRESHOLD = 15 # Adjust as needed
    SPEED_THRESHOLD_LOW = 0.5
    SPEED_THRESHOLD_HIGH = 3
    SPEED_THRESHOLD_RANGE = 1.5 #SPEED_THRESHOLD_HIGH - SPEED_THRESHOLD_LOW
    WAYPOINTS_COUNT = 10  # Number of waypoints to consider
    upcoming_turn = False
    
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
    slope_history = []
#    if(waypoints_remaining > 10):

    for i in range(1, WAYPOINTS_COUNT+20):
        lookahead_point = waypoints[(next_point + i) % len(waypoints)]  # Wrap around to start if needed
        lookahead_points.append(lookahead_point)

    # Fit a polynomial to the lookahead points
    x_vals = [point[0] for point in lookahead_points]
    y_vals = [point[1] for point in lookahead_points]
    
    # Calculate slope and decide reward for speed
    slope = (y_vals[19]-y_vals[0])/(x_vals[19]-x_vals[0])
    slope_history.append(slope)
    
    if(abs(slope) > 50 or abs(slope) < .1):
        upcoming_turn = True

    desired_steering_angle = numpy.arctan(slope) * 180 / numpy.pi

    angle_diff = abs(desired_steering_angle - heading)

    # Normalize angle difference
    if angle_diff > 180:
        angle_diff = 360 - angle_diff

    # Calculate steering angle reward based on angle difference
    steering_reward = 1.0 - (angle_diff / 180.0)

    # Apply calibration based on speed
    steering_to_speed_calibration = 1.0 - (0.5 * (speed / SPEED_THRESHOLD_HIGH))  # Adjust as needed
    steering_reward *= steering_to_speed_calibration

    # Calculate distance from center reward
    MAX_REWARD_DISTANCE = 0.5 * params['track_width']
    MIN_REWARD_DISTANCE = 0.3 * params['track_width']
    distance_reward = 1.0 - (distance_from_center / MAX_REWARD_DISTANCE)
    pen_distance_reward = 1.0 - (distance_from_center / MIN_REWARD_DISTANCE)
    
    # Calculate speed reward
    if upcoming_turn==False and all_wheels_on_track and SPEED_THRESHOLD_LOW <= speed <= SPEED_THRESHOLD_RANGE:
        speed_reward = (speed - SPEED_THRESHOLD_LOW) / SPEED_THRESHOLD_RANGE
    else:
        # Penalize if car steer too much to prevent zigzag
        ABS_STEERING_THRESHOLD = 20.0
        if steering_angle > ABS_STEERING_THRESHOLD and all_wheels_on_track and pen_distance_reward<0.5:
            steering_reward *= 0.8
        speed_reward = 0.7 * steering_reward  # Penalize if too fast or off track
        
    # Final reward is a combination of all rewards
    reward = 0.4 * steering_reward + 0.4 * distance_reward + 0.2 * speed_reward

    print(str(desired_steering_angle) + ":" +  str(steering_to_speed_calibration) + ":" + str(steering_reward) + ":" + str(distance_reward) + ":" + str(speed_reward) + ":" + str(waypoints_completed)+ ":" + str(waypoints_remaining) + ":" + str(slope))
    
    return float(reward)

