import math

class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = None
        self.verbose = verbose

    def reward_function(self, params):
    
        import math

        ################## HELPER FUNCTIONS ###################

        def dist_2_points(x1, x2, y1, y2):
            return abs(abs(x1-x2)**2 + abs(y1-y2)**2)**0.5

        def closest_2_racing_points_index(racing_coords, car_coords):

            # Calculate all distances to racing points
            distances = []
            for i in range(len(racing_coords)):
                distance = dist_2_points(x1=racing_coords[i][0], x2=car_coords[0],
                                         y1=racing_coords[i][1], y2=car_coords[1])
                distances.append(distance)

            # Get index of the closest racing point
            closest_index = distances.index(min(distances))

            # Get index of the second closest racing point
            distances_no_closest = distances.copy()
            distances_no_closest[closest_index] = 999
            second_closest_index = distances_no_closest.index(
                min(distances_no_closest))

            return [closest_index, second_closest_index]

        def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):
            
            # Calculate the distances between 2 closest racing points
            a = abs(dist_2_points(x1=closest_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=closest_coords[1],
                                  y2=second_closest_coords[1]))

            # Distances between car and closest and second closest racing point
            b = abs(dist_2_points(x1=car_coords[0],
                                  x2=closest_coords[0],
                                  y1=car_coords[1],
                                  y2=closest_coords[1]))
            c = abs(dist_2_points(x1=car_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=car_coords[1],
                                  y2=second_closest_coords[1]))

            # Calculate distance between car and racing line (goes through 2 closest racing points)
            try:
                distance = abs(-(a**4) + 2*(a**2)*(b**2) + 2*(a**2)*(c**2) -
                               (b**4) + 2*(b**2)*(c**2) - (c**4))**0.5 / (2*a)
            except:
                distance = b

            return distance

        # Calculate which one of the closest racing points is the next one and which one the previous one
        def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):

            # Virtually set the car more into the heading direction
            heading_vector = [math.cos(math.radians(
                heading)), math.sin(math.radians(heading))]
            new_car_coords = [car_coords[0]+heading_vector[0],
                              car_coords[1]+heading_vector[1]]

            # Calculate distance from new car coords to 2 closest racing points
            distance_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                        x2=closest_coords[0],
                                                        y1=new_car_coords[1],
                                                        y2=closest_coords[1])
            distance_second_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                               x2=second_closest_coords[0],
                                                               y1=new_car_coords[1],
                                                               y2=second_closest_coords[1])

            if distance_closest_coords_new <= distance_second_closest_coords_new:
                next_point_coords = closest_coords
                prev_point_coords = second_closest_coords
            else:
                next_point_coords = second_closest_coords
                prev_point_coords = closest_coords

            return [next_point_coords, prev_point_coords]

        def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):

            # Calculate the direction of the center line based on the closest waypoints
            next_point, prev_point = next_prev_racing_point(closest_coords,
                                                            second_closest_coords,
                                                            car_coords,
                                                            heading)

            # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
            track_direction = math.atan2(
                next_point[1] - prev_point[1], next_point[0] - prev_point[0])

            # Convert to degree
            track_direction = math.degrees(track_direction)

            # Calculate the difference between the track direction and the heading direction of the car
            direction_diff = abs(track_direction - heading)
            if direction_diff > 180:
                direction_diff = 360 - direction_diff

            return direction_diff

        # Gives back indexes that lie between start and end index of a cyclical list
        def indexes_cyclical(start, end, array_len):

            if end < start:
                end += array_len

            return [index % array_len for index in range(start, end)]

        # Calculate how long car would take for entire lap, if it continued like it did until now
        def projected_time(first_index, closest_index, step_count, times_list):

            # Calculate how much time has passed since start
            current_actual_time = (step_count-1) / 15

            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum([times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            try:
                projected_time = (current_actual_time/current_expected_time) * total_expected_time
            except:
                projected_time = 9999

            return projected_time

        #################### RACING LINE ######################

        # Optimal racing line for the Ace Speedway track
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = [[0.11704, -3.41436, 4.0, 0.04281],
                        [0.00309, -3.3518, 4.0, 0.0325],
                        [-0.11087, -3.28924, 4.0, 0.0325],
                        [-0.26098, -3.20682, 4.0, 0.04281],
                        [-0.52505, -3.06184, 4.0, 0.07531],
                        [-0.78912, -2.91687, 4.0, 0.07531],
                        [-1.05319, -2.77189, 4.0, 0.07531],
                        [-1.31726, -2.6269, 4.0, 0.07531],
                        [-1.58133, -2.48192, 4.0, 0.07531],
                        [-1.84539, -2.33694, 4.0, 0.07531],
                        [-2.1095, -2.19204, 4.0, 0.07531],
                        [-2.37374, -2.04739, 4.0, 0.07531],
                        [-2.63811, -1.90301, 4.0, 0.07531],
                        [-2.9026, -1.75887, 4.0, 0.0753],
                        [-3.16722, -1.61498, 4.0, 0.0753],
                        [-3.43195, -1.47131, 4.0, 0.0753],
                        [-3.6968, -1.32788, 4.0, 0.0753],
                        [-3.96177, -1.18467, 4.0, 0.0753],
                        [-4.22685, -1.0417, 4.0, 0.0753],
                        [-4.49205, -0.89895, 4.0, 0.07529],
                        [-4.75732, -0.75636, 4.0, 0.07529],
                        [-5.02271, -0.614, 4.0, 0.07529],
                        [-5.28823, -0.47188, 4.0, 0.07529],
                        [-5.55258, -0.32957, 4.0, 0.07506],
                        [-5.81529, -0.18644, 4.0, 0.07479],
                        [-6.0757, -0.04197, 4.0, 0.07445],
                        [-6.33314, 0.1044, 4.0, 0.07404],
                        [-6.58692, 0.25322, 4.0, 0.07355],
                        [-6.83634, 0.40504, 4.0, 0.073],
                        [-7.08039, 0.56059, 4.0, 0.07235],
                        [-7.31807, 0.72059, 3.72781, 0.07686],
                        [-7.54808, 0.88589, 3.31774, 0.08537],
                        [-7.76885, 1.05748, 2.94535, 0.09494],
                        [-7.97844, 1.23652, 2.58649, 0.10657],
                        [-8.17395, 1.4246, 2.16748, 0.12516],
                        [-8.35219, 1.62325, 2.16748, 0.12313],
                        [-8.50906, 1.83411, 2.16748, 0.12126],
                        [-8.6386, 2.05919, 2.16748, 0.11981],
                        [-8.72873, 2.30118, 2.39307, 0.10791],
                        [-8.78541, 2.55365, 2.5977, 0.09961],
                        [-8.81275, 2.81209, 2.81458, 0.09233],
                        [-8.81447, 3.07299, 3.02364, 0.08629],
                        [-8.79377, 3.33379, 3.22866, 0.08103],
                        [-8.7534, 3.59271, 3.39117, 0.07727],
                        [-8.69535, 3.84851, 3.52614, 0.07439],
                        [-8.62117, 4.10027, 3.60328, 0.07284],
                        [-8.53182, 4.34714, 3.4691, 0.07568],
                        [-8.42822, 4.58843, 3.21102, 0.08178],
                        [-8.31086, 4.82334, 2.95974, 0.08872],
                        [-8.17977, 5.05092, 2.69229, 0.09755],
                        [-8.03401, 5.26948, 2.42449, 0.10836],
                        [-7.87185, 5.47639, 2.42449, 0.10843],
                        [-7.69142, 5.66816, 2.42449, 0.1086],
                        [-7.4904, 5.83971, 2.42449, 0.109],
                        [-7.26631, 5.98362, 2.70886, 0.09831],
                        [-7.02546, 6.10332, 3.00205, 0.08959],
                        [-6.77204, 6.20193, 3.3353, 0.08153],
                        [-6.50952, 6.28237, 3.59234, 0.07643],
                        [-6.24036, 6.34629, 3.84404, 0.07197],
                        [-5.96667, 6.39519, 4.0, 0.06951],
                        [-5.69017, 6.43041, 4.0, 0.06968],
                        [-5.41216, 6.45293, 4.0, 0.06973],
                        [-5.13366, 6.46361, 4.0, 0.06968],
                        [-4.85548, 6.46322, 4.0, 0.06955],
                        [-4.57823, 6.4524, 4.0, 0.06936],
                        [-4.30251, 6.43141, 4.0, 0.06913],
                        [-4.02895, 6.4, 3.73673, 0.07369],
                        [-3.75825, 6.3578, 3.29097, 0.08325],
                        [-3.49122, 6.3042, 2.93736, 0.09272],
                        [-3.22907, 6.23778, 2.61128, 0.10357],
                        [-2.97328, 6.15694, 2.61128, 0.10273],
                        [-2.72656, 6.05835, 2.61128, 0.10175],
                        [-2.49233, 5.93872, 2.61128, 0.10072],
                        [-2.27529, 5.79416, 2.78445, 0.09365],
                        [-2.07442, 5.62887, 2.82323, 0.09214],
                        [-1.89016, 5.44411, 3.0022, 0.08692],
                        [-1.72138, 5.2424, 3.46791, 0.07584],
                        [-1.56479, 5.02797, 3.96985, 0.06688],
                        [-1.4179, 4.80371, 4.0, 0.06702],
                        [-1.27827, 4.57223, 4.0, 0.06758],
                        [-1.14354, 4.33595, 4.0, 0.068],
                        [-1.01123, 4.09732, 4.0, 0.06822],
                        [-0.87139, 3.85003, 4.0, 0.07102],
                        [-0.72851, 3.60377, 4.0, 0.07117],
                        [-0.58196, 3.35923, 4.0, 0.07127],
                        [-0.43117, 3.11713, 4.0, 0.07131],
                        [-0.27551, 2.87827, 4.0, 0.07127],
                        [-0.11435, 2.64354, 4.0, 0.07118],
                        [0.05279, 2.41369, 4.0, 0.07105],
                        [0.22665, 2.1897, 4.0, 0.07088],
                        [0.40772, 1.97241, 4.0, 0.07071],
                        [0.59637, 1.76254, 4.0, 0.07055],
                        [0.79283, 1.56071, 4.0, 0.07041],
                        [0.99721, 1.3675, 4.0, 0.07031],
                        [1.2095, 1.18337, 4.0, 0.07025],
                        [1.42958, 1.00874, 4.0, 0.07024],
                        [1.65722, 0.8439, 4.0, 0.07026],
                        [1.89212, 0.68908, 4.0, 0.07033],
                        [2.13388, 0.54439, 4.0, 0.07044],
                        [2.38207, 0.40985, 4.0, 0.07058],
                        [2.63622, 0.28535, 4.0, 0.07075],
                        [2.89581, 0.17067, 4.0, 0.07095],
                        [3.16031, 0.06547, 4.0, 0.07116],
                        [3.42916, -0.03075, 4.0, 0.07139],
                        [3.70181, -0.11861, 4.0, 0.07161],
                        [3.97767, -0.19885, 4.0, 0.07182],
                        [4.25617, -0.27237, 4.0, 0.07201],
                        [4.53674, -0.34017, 4.0, 0.07216],
                        [4.81886, -0.40332, 4.0, 0.07227],
                        [5.10208, -0.4626, 4.0, 0.07234],
                        [5.38582, -0.51873, 4.0, 0.07231],
                        [5.66903, -0.57213, 4.0, 0.07205],
                        [5.94931, -0.62279, 4.0, 0.07121],
                        [6.22625, -0.67467, 4.0, 0.07044],
                        [6.50092, -0.73035, 3.83388, 0.0731],
                        [6.77191, -0.79208, 3.33153, 0.08342],
                        [7.0378, -0.86198, 2.93821, 0.09357],
                        [7.29689, -0.94237, 2.58968, 0.10475],
                        [7.54735, -1.03547, 2.25854, 0.11831],
                        [7.78633, -1.14437, 2.0, 0.13131],
                        [8.0104, -1.27212, 2.0, 0.12896],
                        [8.21484, -1.42204, 2.0, 0.12676],
                        [8.39241, -1.59808, 2.0, 0.12502],
                        [8.53312, -1.80274, 2.14453, 0.11581],
                        [8.63908, -2.02794, 2.34273, 0.10624],
                        [8.71389, -2.26723, 2.53149, 0.09904],
                        [8.7604, -2.51589, 2.76223, 0.09158],
                        [8.78202, -2.76998, 2.9252, 0.08718],
                        [8.78075, -3.02663, 3.10169, 0.08275],
                        [8.7588, -3.28349, 3.24556, 0.07943],
                        [8.71791, -3.53874, 3.37299, 0.07664],
                        [8.65961, -3.79095, 3.49041, 0.07416],
                        [8.58532, -4.03898, 3.58576, 0.07221],
                        [8.49624, -4.28193, 3.62013, 0.07148],
                        [8.39338, -4.51903, 3.44691, 0.07498],
                        [8.27762, -4.7496, 3.25018, 0.07938],
                        [8.14937, -4.9728, 2.94953, 0.08728],
                        [8.00861, -5.18743, 2.67744, 0.09586],
                        [7.85485, -5.39173, 2.40818, 0.10618],
                        [7.68747, -5.58344, 2.1256, 0.11973],
                        [7.505, -5.75876, 2.09677, 0.12069],
                        [7.30628, -5.91289, 2.09677, 0.11994],
                        [7.09038, -6.0393, 2.09677, 0.11932],
                        [6.8567, -6.12814, 2.09677, 0.11923],
                        [6.6093, -6.17613, 2.48051, 0.1016],
                        [6.35451, -6.19341, 2.74269, 0.09311],
                        [6.09486, -6.18427, 3.06481, 0.08477],
                        [5.83208, -6.15283, 3.33794, 0.07929],
                        [5.56722, -6.1016, 3.6358, 0.0742],
                        [5.30108, -6.0328, 3.96379, 0.06935],
                        [5.03428, -5.94851, 4.0, 0.06995],
                        [4.76727, -5.85074, 4.0, 0.07109],
                        [4.5004, -5.74158, 4.0, 0.07208],
                        [4.23388, -5.62285, 4.0, 0.07294],
                        [3.96782, -5.49652, 4.0, 0.07363],
                        [3.70226, -5.36428, 4.0, 0.07417],
                        [3.43715, -5.22755, 4.0, 0.07457],
                        [3.17245, -5.08746, 4.0, 0.07487],
                        [2.90805, -4.94503, 4.0, 0.07508],
                        [2.64385, -4.80112, 4.0, 0.07521],
                        [2.37975, -4.65651, 4.0, 0.07527],
                        [2.11566, -4.51158, 4.0, 0.07531],
                        [1.85157, -4.36664, 4.0, 0.07531],
                        [1.58749, -4.22169, 4.0, 0.07531],
                        [1.32342, -4.07671, 4.0, 0.07531],
                        [1.05936, -3.93172, 4.0, 0.07531],
                        [0.79529, -3.78674, 4.0, 0.07531],
                        [0.53122, -3.64176, 4.0, 0.07531],
                        [0.26716, -3.49678, 4.0, 0.07531]]

        ################## INPUT PARAMETERS ###################

        # Read all input parameters
        all_wheels_on_track = params['all_wheels_on_track']
        x = params['x']
        y = params['y']
        distance_from_center = params['distance_from_center']
        is_left_of_center = params['is_left_of_center']
        heading = params['heading']
        progress = params['progress']
        steps = params['steps']
        speed = params['speed']
        steering_angle = params['steering_angle']
        track_width = params['track_width']
        waypoints = params['waypoints']
        closest_waypoints = params['closest_waypoints']
        is_offtrack = params['is_offtrack']

        ############### OPTIMAL X,Y,SPEED,TIME ################

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(
            racing_track, [x, y])

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        # Save first racingpoint of episode for later
        if self.verbose == True:
            self.first_racingpoint_index = 0 # this is just for testing purposes
        if steps == 1:
            self.first_racingpoint_index = closest_index

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        reward = 1

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 1
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist/(track_width*0.5)))
        reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 2
        speed_diff = abs(optimals[2]-speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff/(SPEED_DIFF_NO_REWARD))**2)**2
        else:
            speed_reward = 0
        reward += speed_reward * SPEED_MULTIPLE

        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 1 
        STANDARD_TIME = 22
        FASTEST_TIME = 18
        times_list = [row[3] for row in racing_track]
        projected_time = projected_time(self.first_racingpoint_index, closest_index, steps, times_list)
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME*(FASTEST_TIME) /
                                           (STANDARD_TIME-FASTEST_TIME))*(steps_prediction-(STANDARD_TIME*15+1)))
            steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
        except:
            steps_reward = 0
        reward += steps_reward

        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading)
        if direction_diff > 30:
            reward = 1e-3
            
        # Zero reward of obviously too slow
        speed_diff_zero = optimals[2]-speed
        if speed_diff_zero > 0.5:
            reward = 1e-3
            
        ## Incentive for finishing the lap in less steps ##
        REWARD_FOR_FASTEST_TIME = 1500 # should be adapted to track length and other rewards
        STANDARD_TIME = 22  # seconds (time that is easily done by model)
        FASTEST_TIME = 18  # seconds (best time of 1st place on the track)
        if progress == 100:
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                      (15*(STANDARD_TIME-FASTEST_TIME)))*(steps-STANDARD_TIME*15))
        else:
            finish_reward = 0
        reward += finish_reward
        
        ## Zero reward if off track ##
        if all_wheels_on_track == False:
            reward = 1e-3

        ####################### VERBOSE #######################
        
        if self.verbose == True:
            print("Closest index: %i" % closest_index)
            print("Distance to racing line: %f" % dist)
            print("=== Distance reward (w/out multiple): %f ===" % (distance_reward))
            print("Optimal speed: %f" % optimals[2])
            print("Speed difference: %f" % speed_diff)
            print("=== Speed reward (w/out multiple): %f ===" % speed_reward)
            print("Direction difference: %f" % direction_diff)
            print("Predicted time: %f" % projected_time)
            print("=== Steps reward: %f ===" % steps_reward)
            print("=== Finish reward: %f ===" % finish_reward)
            
        #################### RETURN REWARD ####################
        
        # Always return a float value
        return float(reward)


reward_object = Reward() # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)