import numpy

# Define a function to fit a polynomial to the given data points
def custom_polyfit(x_vals, y_vals, degree):
    # Calculate the number of coefficients needed for the polynomial
    num_coefficients = degree + 1

    # Initialize the matrices for the equations
    A = []
    for i in range(num_coefficients):
        A_row = []
        for j in range(num_coefficients):
            A_row.append(sum(x ** (i + j) for x in x_vals))
        A.append(A_row)

    # Calculate the right-hand side of the equations
    B = []
    for i in range(num_coefficients):
        B.append(sum(y * (x ** i) for x, y in zip(x_vals, y_vals)))

    # Solve the system of equations to find the coefficients
    coefficients = []
    for i in range(num_coefficients):
        row_i = A[i]
        for j in range(i):
            factor = row_i[j] / A[j][j]
            for k in range(j, num_coefficients):
                row_i[k] -= factor * A[j][k]
        coefficients.append(B[i] / row_i[i])

    return coefficients

# Define a function to calculate the derivative of a polynomial
def custom_polyder(coefficients):
    # Calculate the degree of the derivative polynomial
    degree = len(coefficients) - 1
    if degree == 0:
        return [0]

    # Calculate the coefficients of the derivative polynomial
    derivative_coeffs = [i * coeff for i, coeff in enumerate(coefficients[1:], start=1)]

    return derivative_coeffs

def reward_function(params):
    
    # Constants
    STEERING_THRESHOLD = 15 # Adjust as needed
    SPEED_THRESHOLD_LOW = 0.5
    SPEED_THRESHOLD_HIGH = 3
    SPEED_THRESHOLD_RANGE = 1 #SPEED_THRESHOLD_HIGH - SPEED_THRESHOLD_LOW
    WAYPOINTS_COUNT = 10  # Number of waypoints to consider
    poly_derivative = 0
    polynomial_coeffs = 0
    curvature = 0
    avg_curvature = 0
    x_variance = 0
    x_std_dev = 0
    y_variance = 0
    y_std_dev = 0
    slope = 0
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
    if(waypoints_remaining > 20):
        for i in range(1, WAYPOINTS_COUNT+20):
            lookahead_point = waypoints[(next_point + i) % len(waypoints)]  # Wrap around to start if needed
            lookahead_points.append(lookahead_point)
    
        # Fit a polynomial to the lookahead points
        x_vals = [point[0] for point in lookahead_points]
        y_vals = [point[1] for point in lookahead_points]
    
        # Fit a polynomial using the polyfit method
        polynomial_coeffs = custom_polyfit(x_vals, y_vals, degree=min(3, len(x_vals)-1))
    
        # Calculate derivative of the polynomial to get curvature
        poly_derivative = custom_polyder(polynomial_coeffs)
        curvature = [abs(sum(coeff * (x ** i) for i, coeff in enumerate(poly_derivative))) for x in x_vals]
    
        # Calculate average curvature
        avg_curvature = sum(curvature) / len(curvature)
    
    
        # Calculate steering angle based on curvature
        avg_curvature = numpy.mean(curvature)
        
        # Calculate the variance and standard deviation of the data
        x_variance = calculate_variance(x_vals)
        y_variance = calculate_variance(y_vals)
        x_std_dev = calculate_std_dev(x_vals)
        y_std_dev = calculate_std_dev(y_vals)
        
            # Calculate slope and decide reward for speed
        slope = (y_vals[19]-y_vals[0])/(x_vals[19]-x_vals[0])
            
        if(abs(slope) > 50 or abs(slope) < .1):
            upcoming_turn = True
    #     # Calculate reward based on standard deviation & variance
    #    if (int(x_variance) in range(0,2)) and (int(y_variance) in range(3,4)):
    #        desired_steering_angle = heading  # Maximum reward for driving straight
#        if (x_std_dev < 1.0) or (y_std_dev < 1.0):
#            desired_steering_angle = heading  # Maximum reward for driving straight
#        else:
        desired_steering_angle = numpy.arctan(avg_curvature * params['track_width']) * 180 / numpy.pi
    else:
        desired_steering_angle = heading
        
    # Calculate angle difference between current heading and desired steering angle
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
        speed_reward = 1 * steering_reward  # Penalize if too fast or off track
        
#    if(waypoints_completed < 10):
#        reward = 0.3 * 1 + 0.3 * distance_reward + 0.4 * 1
#    else:
    # Final reward is a combination of all rewards
    reward = 0.4 * steering_reward + 0.4 * distance_reward + 0.2 * speed_reward

    print(str(poly_derivative) + ":" + str(curvature) + ":" +  str(desired_steering_angle) + ":" +  str(steering_to_speed_calibration) + ":" + str(steering_reward) + ":" + str(distance_reward) + ":" + str(speed_reward) + ":" + str(waypoints_completed) + ":" + str(x_variance) + ":" + str(x_std_dev) + ":" + str(y_variance) + ":" + str(y_std_dev) + ":" + str(waypoints_remaining) + ":" + str(slope))
    
    return float(reward)

# Calculate the variance of the data
def calculate_variance(data):
    mean = sum(data) / len(data)
    variance = sum((x - mean) ** 2 for x in data) / len(data)
    return variance

# Calculate the standard deviation of the data
def calculate_std_dev(data):
    return calculate_variance(data) ** 0.5


