import time
import math
import random
import ArdumowerClient
import airsim
import numpy as np
import cv2
from skimage.segmentation import find_boundaries
from skimage.morphology import remove_small_objects

client = ArdumowerClient.ArdumowerClient(0.36, 0.225, 4)

### virtul laser parameter
# img = np.zeros((240, 320), dtype=np.uint8)
start_point = np.array([160, 230])
no_ray = 50
min_angle = math.radians(0)
max_angle = math.radians(180)
angle_incr = (max_angle - min_angle) / (no_ray - 1)
max_range = 160
end_pts = []
for i in range(no_ray):
    end_pt = np.array([start_point[0] + max_range * math.cos(i*angle_incr), start_point[1] - max_range * math.sin(i*angle_incr)]).astype(np.int32)
    end_pts.append(end_pt)

    
### wall follower parameter

loop_index = 0              # Number of sampling cycles
loop_index_outer_corner = 0 # Loop index when the outer corner is detected
loop_index_inner_corner = 0 # Loop index when the inner corner is detected
inf = 158                   #      considered out of sensor range
wall_dist = 120             # Distance desired from the wall
max_speed = 0.3             # Maximum speed of the robot on meters/seconds
p = 0.01                      # Proportional constant for controller  
d = 0.005                     # Derivative constant for controller 
angle = 0.01                   # Proportional constant for angle controller (just simple P controller)
direction = -1              # 1 for wall on the left side of the robot (-1 for the right side)
e = 0                       # Diference between current wall measurements and previous one
angle_min = 0               # Angle, at which was measured the shortest distance between the robot and a wall
dist_front = 0              # Measured front distance
diff_e = 0                  # Difference between current error and previous one
dist_min = 0                # Minimum measured distance

# Time when the last outer corner; direction and inner corner were detected or changed.
last_outer_corner_detection_time = time.time()
last_change_direction_time = time.time()
last_inner_corner_detection_time = time.time()
rotating = 0 

# Sensor regions
regions_ = {
        'right': 0,
        'fright': 0,
        'front': 0,
        'fleft': 0,
        'left': 0,
}
last_kinds_of_wall=[0, 0, 0, 0, 0]
index = 0

state_outer_inner=[0, 0, 0, 0]
index_state_outer_inner = 0

bool_outer_corner = 0
bool_inner_corner =0

last_vel = [random.uniform(0.1,0.3),  random.uniform(-0.3,0.3)]
wall_found =0

#Robot state machines
state_ = 0
state_dict_ = {
    0: 'random wandering',
    1: 'following wall',
    2: 'rotating'
}


### functions for virtual laser

def is_collidable(array, point):
    if array[point[1]][point[0]] == 0:
        return True
    else:
        return False


def get_distance(pt1, pt2):
    x_comp, y_comp = pt2[0] - pt1[0], pt2[1] - pt1[1]
    return math.hypot(x_comp, y_comp)


def cast_ray(start_pt, end_pt, array, return_pt=False):
    x0, y0 = start_pt[0], start_pt[1]
    x1, y1 = end_pt[0], end_pt[1]
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    x, y = x0, y0
    sx = -1 if x0 > x1 else 1
    sy = -1 if y0 > y1 else 1
    pt = np.zeros(2, np.int32)
    if dx > dy:
        err = dx / 2.0
        while x != x1 and is_collidable(array, pt):
            pt = np.array([x, y])
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1 and is_collidable(array, pt):
            pt = np.array([x, y])
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    pt = np.array([x, y])
    if return_pt:
        return get_distance(start_pt, pt), pt
    else:
        return get_distance(start_pt, pt)


def process_rays(image, visualize=False):
    global no_ray, start_point, end_pts
    ranges = []
    if not visualize:
        for i in range(no_ray):
            distance = cast_ray(start_point, end_pts[i], image)
            ranges.append(distance)
        return ranges, None
    else:
        visual_img = image.copy()
        for i in range(no_ray):
            distance, pt = cast_ray(start_point, end_pts[i], image, return_pt=True)
            cv2.line(visual_img, tuple(start_point), tuple(pt), 200)
            cv2.circle(visual_img, tuple(pt), 2, 200)
            ranges.append(distance)
        return ranges, visual_img


def process_contour(image):
    cnt_image = image.copy()
    contours, hierarchy = cv2.findContours(image, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    cnts = sorted(contours, key=cv2.contourArea, reverse=True)[:2]  # Sort contours by area
    for i,cnt in enumerate(cnts):# Remove contoure that has area less than thresh
        area = cv2.contourArea(cnt)
        if area < 5000:
            del(cnts[i])
    for k in range(len(cnts)):
        cv2.drawContours(cnt_image, cnts, contourIdx=k, color=125, thickness=-1)
    cnt_image = np.where(cnt_image == 125, 1, 0).astype(np.uint8)
    boundary = find_boundaries(cnt_image, mode='thick').astype(np.uint8)
    boundary = np.where(boundary == 0, 0, 255).astype(np.uint8)
    return boundary


### wall follower function
def change_state(state):
    """
    Update machine state
    """
    global state_, state_dict_
    if state is not state_:
        #print 'Wall follower - [%s] - %s' % (state, state_dict_[state])
        state_ = state

        
def take_action():
    """
    Change state for the machine states in accordance with the active and inactive regions of the sensor.
            State 0 No wall found - all regions infinite - Random Wandering
            State 1 Wall found - Following Wall
            State 2 Pattern sequence reached - Rotating
    """
    global regions_, index, last_kinds_of_wall, index_state_outer_inner, state_outer_inner, loop_index, loop_index_outer_corner
    
    global wall_dist, max_speed, direction, p, d, angle, dist_min, wall_found, rotating, bool_outer_corner, bool_inner_corner

    regions = regions_
    linear_x = 0
    angular_z = 0
    state_description = ''
    d_front = 100
    # Patterns for rotating
    rotate_sequence_V1 = ['I', 'C', 'C', 'C']
    rotate_sequence_V2 = [0, 'C', 'C', 'C']
    rotate_sequence_W = ['I', 'C', 'I', 'C']

    if rotating == 1:
        state_description = 'case 2 - rotating'
        change_state(2)
        if(regions['left'] < wall_dist or regions['right'] < wall_dist):
            rotating = 0
    elif regions['fright'] >= inf and regions['front'] >= inf and regions['right'] >= inf and regions['fleft'] >= inf and regions['left'] >= inf:
        state_description = 'case 0 - random wandering'
        change_state(0)
    elif (loop_index == loop_index_outer_corner) and (rotate_sequence_V1 == state_outer_inner or rotate_sequence_V2 == state_outer_inner or rotate_sequence_W == state_outer_inner):
        state_description = 'case 2 - rotating'
        change_direction()
        state_outer_inner = [ 0, 0,  0, 'C']
        change_state(2)
    else:
        state_description = 'case 1 - following wall'
        change_state(1)
    print(state_description)

def following_wall():
    """
    PD control for the wall following state. 
    Returns:
            Twist(): msg with angular and linear velocities to be published
                    msg.linear.x -> 0; 0.5max_speed; 0.4max_speed
                    msg.angular.z -> PD controller response
    """
    global wall_dist, max_speed, direction, p, d, angle, dist_min, dist_front, e, diff_e, angle_min
    if dist_front < wall_dist:
        linear_x = 0
    elif dist_front < wall_dist*2:
        linear_x = 0.5*max_speed
    elif abs(angle_min) > 1.75:
        linear_x = 0.4*max_speed
    else:
        linear_x = max_speed
    angular_z = max(min(direction*(p*e+d*diff_e) + angle*(angle_min-((math.pi)/2)*direction), 1), -1)
    #print 'Turn Left angular z, linear x %f - %f' % (msg.angular.z, msg.linear.x)
    print(linear_x, angular_z)
    return (linear_x, angular_z)


def random_wandering():
    """
    This function defines the linear.x and angular.z velocities for the random wandering of the robot.
    Returns:
            Twist(): msg with angular and linear velocities to be published
                    msg.linear.x -> [0.1, 0.3]
                    msg.angular.z -> [-1, 1]
    """
    linear_x = 0.2
    angular_z = -0.2
    return (linear_x, angular_z)


def change_direction():
    """
    Toggle direction in which the robot will follow the wall
        1 for wall on the left side of the robot and -1 for the right side
    """
    global direction, last_change_direction, rotating
    print('Change direction!')
    elapsed_time = time.time() - last_change_direction_time # Elapsed time since last change direction
    if elapsed_time >= 20:
        last_change_direction = time.time()
        direction = -direction # Wall in the other side now
        rotating = 1


def rotate():
    """
    Rotation movement of the robot. 
    Returns:
            Twist(): msg with angular and linear velocities to be published
                    msg.linear.x -> 0m/s
                    msg.angular.z -> -2 or +2 rad/s
    """
    linear_x = 0
    angular_z = direction*0.3
    return (linear_x, angular_z)


def is_outer_corner():
    """
    Assessment of outer corner in the wall. 
    If all the regions except for one of the back regions are infinite then we are in the presence of a possible corner.
    If all the elements in last_kinds_of_wall are 'C' and the last time a real corner was detected is superior or equal to 30 seconds:
        To state_outer_inner a 'C' is appended and 
        The time is restart.
    Returns:
            bool_outer_corner: 0 if it is not a outer corner; 1 if it is a outer corner
    """
    global regions_, last_kinds_of_wall, last_outer_corner_detection_time, index, state_outer_inner, index_state_outer_inner, loop_index, loop_index_outer_corner
    regions = regions_
    bool_outer_corner = 0
    if (regions['fright'] >= inf and regions['front'] >= inf and regions['right'] >= inf  and regions['left'] >= inf  and regions['fleft'] >= inf) or (regions['fleft'] >= inf and regions['front'] >= inf and regions['left'] >= inf and regions['right'] >= inf and regions['fright'] >= inf):
        bool_outer_corner = 1 # It is a corner
        last_kinds_of_wall[index]='C'
        elapsed_time = time.time() - last_outer_corner_detection_time # Elapsed time since last corner detection
        if last_kinds_of_wall.count('C') == len(last_kinds_of_wall) and elapsed_time >= 30:
            last_outer_corner_detection_time = time.time()
            loop_index_outer_corner = loop_index
            state_outer_inner = state_outer_inner[1:]
            state_outer_inner.append('C')
            print('It is a outer corner')
    return bool_outer_corner


def is_inner_corner():
    """
    Assessment of inner corner in the wall. 
    If the three front regions are inferior than the wall_dist.
    If all the elements in last_kinds_of_wall are 'I' and the last time a real corner was detected is superior or equal to 20 seconds:
        To state_outer_inner a 'I' is appended and 
        The time is restart.
    Returns:
            bool_inner_corner: 0 if it is not a inner corner; 1 if it is a inner corner
    """
    global regions_, wall_dist, last_kinds_of_wall, last_inner_corner_detection_time, index, state_outer_inner, index_state_outer_inner, loop_index_inner_corner, loop_index
    regions = regions_
    bool_inner_corner = 0
    if regions['fright'] < wall_dist and regions['front'] < wall_dist and regions['fleft'] < wall_dist:
        bool_inner_corner = 1
        last_kinds_of_wall[index]='I'
        elapsed_time = time.time() - last_inner_corner_detection_time # Elapsed time since last corner detection
        if last_kinds_of_wall.count('I') == len(last_kinds_of_wall) and elapsed_time >= 20:
            last_inner_corner_detection_time = time.time()
            loop_index_inner_corner = loop_index
            state_outer_inner = state_outer_inner[1:]
            state_outer_inner.append('I')
            print('It is a inner corner')
    return bool_inner_corner

def clbk_laser(laser_data):
    """
    Read sensor messagens, and determine distance to each region. 
    Manipulates the values measured by the sensor.
    Callback function for the subscription to the published Laser Scan values.
    """
    global angle_incr, regions_, e, angle_min, dist_front, diff_e, direction, bool_outer_corner, bool_inner_corner, index, last_kinds_of_wall
    size = int(len(laser_data))
    min_index = int(size*(direction+1)/4)
    max_index = int(size*(direction+3)/4)
    # Determine values for PD control of distance and P control of angle
    for i in range(min_index, max_index):
        if laser_data[i] < laser_data[min_index] and laser_data[i] > 0.01 :
            min_index = i
    angle_min = (min_index-size/2)*angle_incr
    dist_min = laser_data[min_index]
    dist_front = laser_data[int(size/2)]
    diff_e = min((dist_min - wall_dist) - e, 100)
    e = min(dist_min - wall_dist, 100)

    # Determination of minimum distances in each region
    regions_ = {
        'right': min(min(laser_data[0:10]), inf),
        'fright':  min(min(laser_data[11:20]), inf),
        'front':  min(min(laser_data[21:30]), inf),
        'fleft':   min(min(laser_data[31:40]), inf),
        'left':   min(min(laser_data[41:50]), inf),
    }
    #rospy.loginfo(regions_)

    # Detection of Outer and Inner corner
    bool_outer_corner = is_outer_corner()
    bool_inner_corner = is_inner_corner()
    if bool_outer_corner == 0 and bool_inner_corner == 0:
        last_kinds_of_wall[index]=0
    
    # Indexing for last five pattern detection
    # This is latter used for low pass filtering of the patterns
    index = index + 1 #5 samples recorded to asses if we are at the corner or not
    if index == len(last_kinds_of_wall):
        index = 0
        
    take_action()

def main():
    while True:
        seg_mask = client.airsim_client.simGetImages([airsim.ImageRequest('Main', airsim.ImageType.Segmentation, False, False)])
        img1d = np.fromstring(seg_mask[0].image_data_uint8, dtype=np.uint8)
        img_rgb = img1d.reshape(seg_mask[0].height, seg_mask[0].width, 4)
        img_rgb = cv2.cvtColor(img_rgb, cv2.COLOR_RGBA2RGB)
        blur_img = cv2.medianBlur(img_rgb,15)
        bin_img = np.where(blur_img[:, :, 0] == 232, 255, 0).astype(np.uint8)
        cnt_img = process_contour(bin_img)
        ranges, visual_out = process_rays(cnt_img, visualize=True)
        img = np.hstack((bin_img, visual_out)).astype(np.uint8)
        cv2.imwrite(path, img)
        cv2.imshow('visualization', img)
        if cv2.waitKey(3) & 0xFF == ord('q'):
            break

        # Wall Following
        clbk_laser(ranges)
        if state_ == 0:
            msg = random_wandering()
        elif state_ == 1:
            msg = following_wall()
        elif state_ == 2:
            msg = rotate()
        else:
            rospy.logerr('Unknown state!')

        client.diff_drive(msg[0],msg[1])
        time.sleep(1/20)
        
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
