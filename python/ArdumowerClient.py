import setup_path
import airsim
import airsim.airsim_types as at
import math

class ArdumowerClient():
    def __init__(self, wheel_base, wheel_radius, max_speed):
        self.airsim_client = airsim.UrdfBotClient()
        self.airsim_client.confirmConnection()
        self.airsim_client.enableApiControl(True)
        
        self.wheel_base = wheel_base
        self.wheel_radius = wheel_radius
        self.max_speed = max_speed
        self.right_wheel_joint_name = 'base_rwheel'
        self.left_wheel_joint_name = 'base_lwheel'
        self.pi = 3.1415926535

    def drive(self, magnitude, theta):
        if (abs(magnitude) > 1):
            raise ValueError('Speed must be between 0 and 1, provided value {0}'.format(magnitude))

        left_multiplier = self.drive_angle_multiplier(theta)
        right_multiplier = self.drive_angle_multiplier(theta - (self.pi / 2))

        right_update_obj = at.UrdfBotControlledMotionComponentControlSignal(component_name = self.right_wheel_joint_name, control_signal_values={'Value': magnitude * right_multiplier})
        left_update_obj = at.UrdfBotControlledMotionComponentControlSignal(component_name = self.left_wheel_joint_name, control_signal_values={'Value': magnitude * left_multiplier})

        self.airsim_client.updateControlledMotionComponentControlSignal(right_update_obj)
        self.airsim_client.updateControlledMotionComponentControlSignal(left_update_obj)
        

    def drive_angle_multiplier(self, theta):
        while (theta > 2 * self.pi):
            theta -= (2 * self.pi)
        while (theta < 0):
            theta += 2 * self.pi

        if (theta < self.pi / 2):
            return math.cos(theta * 2)
        elif (theta < self.pi):
            return -1.0
        elif (theta < 3 * self.pi / 2):
            return -1.0 * math.cos( (theta - self.pi) * 2)
        else:
            return 1.0
     
    def range_map(self, value, x_min, x_max, y_min=-1, y_max=1):
        slope = (y_max - y_min) / (x_max - x_min)
        c = ((x_max * y_min) - (x_min * y_max)) / (x_max - x_min)
        return c + slope * value
        
            
    def diff_drive(self, linear, angular):
        left_velocity = ((2 * linear) - (angular * self.wheel_base)) / (2 * self.wheel_radius)
        right_velocity = ((2 * linear) + (angular * self.wheel_base)) / (2 * self.wheel_radius)
        
        left_multiplier = self.range_map(left_velocity, -self.max_speed, self.max_speed)
        right_multiplier = self.range_map(right_velocity, -self.max_speed, self.max_speed)
        right_update_obj = at.UrdfBotControlledMotionComponentControlSignal(component_name = self.right_wheel_joint_name, control_signal_values={'Value': right_multiplier})
        left_update_obj = at.UrdfBotControlledMotionComponentControlSignal(component_name = self.left_wheel_joint_name, control_signal_values={'Value': left_multiplier})
        
        self.airsim_client.updateControlledMotionComponentControlSignal(right_update_obj)
        self.airsim_client.updateControlledMotionComponentControlSignal(left_update_obj)
