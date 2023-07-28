import math
def check_direction(waypoints,closest_waypoints,heading,reward):
    next_point = waypoints[closest_waypoints[1]]
    prev_point = waypoints[closest_waypoints[0]]

    # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
    track_direction = math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0])
    # Convert to degree
    track_direction = math.degrees(track_direction)

    # Calculate the difference between the track direction and the heading direction of the car
    direction_diff = abs(track_direction - heading)
    if direction_diff > 180:
        direction_diff = 360 - direction_diff

    # Penalize the reward if the difference is too large
    DIRECTION_THRESHOLD = 10.0
    if direction_diff > DIRECTION_THRESHOLD:
        reward *= 0.7
        
    
    return reward

# #the card is permitted to take a left turn only at the start and at the 50-70% mmark of the track
# def reward_on_direction(reward,progress):
#     if (progress < 15 or (progress > 50 and progress < 70)) :
#         reward *= 2
#     return reward

def calculate_angle(p1,p2,p3):
    p12 = math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
    p13 = math.sqrt((p1[0]-p3[0])**2 + (p1[1]-p3[1])**2)
    p23 = math.sqrt((p3[0]-p2[0])**2 + (p3[1]-p2[1])**2)

    return math.acos((p12**2+p13**2-p23**2)/(2*p12*p13))

def reward_to_drive_faster_straight(reward,waypoints,closest_waypoints,speed,is_left_of_center):
    is_car_taking_a_left_turn = False
    is_car_taking_a_right_turn = False
    next_point = waypoints[closest_waypoints[1]]
    prev_point = waypoints[closest_waypoints[0]]
    nextest_point = waypoints[(closest_waypoints[1]+2)%len(waypoints)]
    angle = calculate_angle(next_point,prev_point,nextest_point)
    is_car_taking_a_left_turn = (angle>200 and angle < 250)
    is_car_taking_a_right_turn = angle < 160
    if not (is_car_taking_a_left_turn or is_car_taking_a_right_turn): # the car is not taking a turn
        if speed >= 3.3:
            reward *= 2 
    if (is_car_taking_a_right_turn and is_left_of_center) or (is_car_taking_a_left_turn and not is_left_of_center):
        reward *= 1.5
    
    return reward,is_car_taking_a_left_turn or is_car_taking_a_right_turn


def reward_function(params):

    # reading all inputs
    all_wheels_on_track = params['all_wheels_on_track']
    distance_from_center = params['distance_from_center']
    is_left_of_center = params['is_left_of_center']
    heading = params['heading']
    progress = params['progress']
    speed = params['speed']
    steering_angle = params['steering_angle']
    track_width = params['track_width']
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    
    marker_1,marker_2,marker_3 = 0,0,0

    reward = ((params["progress"] / params["steps"]) * 5) + params["speed"]

    if not all_wheels_on_track:
        reward = reward - 1
    
    reward = check_direction(waypoints,closest_waypoints,heading,reward)

    reward,is_car_taking_turn = reward_to_drive_faster_straight(reward,waypoints,closest_waypoints,speed,is_left_of_center)

    if not is_car_taking_turn:
        marker_1 = 0.1 * track_width
        marker_2 = 0.25 * track_width
        marker_3 = 0.5 * track_width
    else:
        marker_1 = 0.2 * track_width
        marker_2 = 0.5 * track_width
        marker_3 = 0.7 * track_width

    if distance_from_center <= marker_1:
        reward *= 1.5
    elif distance_from_center <= marker_2:
        reward *= 1.2
    elif distance_from_center <= marker_3:
        reward *= 0.8
    else:
        reward = 0 


    return float(reward)