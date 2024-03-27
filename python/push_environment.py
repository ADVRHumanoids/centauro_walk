import rospy
from gazebo_msgs.srv import ApplyBodyWrench, ApplyBodyWrenchRequest
from geometry_msgs.msg import Wrench, Vector3
from std_srvs.srv import SetBool, Empty
from xbot_msgs.srv import PluginStatus, PluginStatusResponse, PluginStatusRequest
from gazebo_msgs.srv import SetModelConfiguration

import xbot_interface.config_options as xbot_opt
from xbot_interface import xbot_interface as xbot

import time


class TestEnvironment:
    def __init__(self):

        rospy.init_node('push_environment', anonymous=True)

        self.__model = None
        self.__robot = None

        self.__ramp_duration = 0.5
        self.__dt = 0.01

        self.__init_xbot_robot()


    def respawn_robot(self):

        reset_world = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        homing_state = rospy.ServiceProxy('/xbotcore/homing/state', PluginStatus)
        ros_control_state = rospy.ServiceProxy('/xbotcore/ros_control/state', PluginStatus)

        ros_control_res = ros_control_state.call()

        while ros_control_res == "Running":
            self.__bool_request('/xbotcore/ros_control/switch', False)
            ros_control_res = ros_control_state.call()

        self.__bool_request('/xbotcore/homing/switch')
        res = homing_state.call()
        while res.status == 'Running':
            res = homing_state.call()

        while ros_control_res == "Stopped":
            self.__bool_request('/xbotcore/ros_control/switch')
            ros_control_res = ros_control_state.call()

        self.__init_stiffness_damping(self.__ramp_duration, self.__dt)
        
        reset_world.call()



    def __ramp_setter(self, current_value_dict, target_value_dict, progress, setter):


        for key in current_value_dict.keys():
            current_value = current_value_dict[key]
            target_value = target_value_dict[key]
            new_value = current_value + (target_value - current_value) * progress
            current_value_dict[key] = new_value

        setter(current_value_dict)

    def __init_xbot_robot(self):

        opt = xbot_opt.ConfigOptions()

        urdf = rospy.get_param(param_name='/robot_description', default='')
        if urdf == '':
            raise print('urdf not set')

        srdf = rospy.get_param(param_name='/robot_description_semantic', default='')
        if srdf == '':
            raise print('srdf not set')

        opt.set_urdf(urdf)
        opt.set_srdf(srdf)
        opt.generate_jidmap()
        opt.set_bool_parameter('is_model_floating_base', True)
        opt.set_string_parameter('model_type', 'RBDL')
        opt.set_string_parameter('framework', 'ROS')

        self.__model = xbot.ModelInterface(opt)
        self.__robot = xbot.RobotInterface(opt)


    def __init_stiffness_damping(self, ramp_duration, dt):

        stiffness_value = 1500.
        damping_value = 20.

        initial_stiffness_map = self.__robot.getStiffnessMap()
        initial_damping_map = self.__robot.getDampingMap()

        goal_stiffness_map = {'knee_pitch_1': stiffness_value, 'knee_pitch_2': stiffness_value, 'knee_pitch_3': stiffness_value, 'knee_pitch_4': stiffness_value,
                                 'hip_pitch_1': stiffness_value, 'hip_pitch_2': stiffness_value, 'hip_pitch_3': stiffness_value, 'hip_pitch_4': stiffness_value,
                                 'hip_roll_1': stiffness_value, 'hip_roll_2': stiffness_value, 'hip_roll_3': stiffness_value, 'hip_roll_4': stiffness_value}

        goal_damping_map = {'knee_pitch_1': damping_value, 'knee_pitch_2': damping_value, 'knee_pitch_3': damping_value, 'knee_pitch_4': damping_value,
                                 'hip_pitch_1': damping_value, 'hip_pitch_2': damping_value, 'hip_pitch_3': damping_value, 'hip_pitch_4': damping_value,
                                 'hip_roll_1': damping_value, 'hip_roll_2': damping_value, 'hip_roll_3': damping_value, 'hip_roll_4': damping_value}


        if ramp_duration <= 0:
            raise ValueError("Ramp duration must be positive")

        start_time = time.time()
        end_time = start_time + ramp_duration


        while time.time() < end_time:
            progress = (time.time() - start_time) / ramp_duration

            self.__ramp_setter(initial_stiffness_map, goal_stiffness_map, progress, self.__robot.setStiffness)
            self.__ramp_setter(initial_damping_map, goal_damping_map, progress, self.__robot.setDamping)

            self.__robot.move()
            time.sleep(dt)  # Adjust as needed for smoother or faster ramp


    def __bool_request(self, service_name, value=True):

        rospy.wait_for_service(service_name, timeout=1.)
        print(f'Service found: {service_name}')

        service = rospy.ServiceProxy(service_name, SetBool)
        response = service(value)

        if response.success:
            rospy.loginfo(f"Service '{service_name}' call successful.")
        else:
            rospy.logerr("Service call failed.")


    def __start_trotting(self):
        self.__bool_request('/horizon/trot/switch')
    def __homing(self):
        self.__bool_request('/xbotcore/homing/switch')



    def apply_impulsive_force(self, wrench):

        body_wrench_service = '/gazebo/apply_body_wrench'
        rospy.wait_for_service(body_wrench_service, timeout=1.)
        print(f'Service found: {body_wrench_service}')

        apply_body_wrench = rospy.ServiceProxy(body_wrench_service, ApplyBodyWrench)

        wrench_msg = Wrench(
            force=Vector3(x=wrench[0], y=wrench[1], z=wrench[2]),  # Adjust force as needed
            torque=Vector3(x=wrench[3], y=wrench[4], z=wrench[5])  # No torque in this example
        )

        # Fill in the necessary information in the request
        req = ApplyBodyWrenchRequest()
        req.body_name = "kyon::pelvis"  # Replace with your robot's link name
        req.reference_frame = "world"
        req.reference_point = Vector3(x=0, y=0, z=0)  # Center of the link
        req.wrench = wrench_msg
        req.start_time = rospy.Time(0)
        req.duration = rospy.Duration.from_sec(1)  # Duration for which the force should be applied

        try:
            # Call the service
            apply_body_wrench(req)
            rospy.loginfo("Impulsive force applied to the robot.")

        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

if __name__ == '__main__':

    te = TestEnvironment()

    te.respawn_robot()
    exit()
    wrench = [0.0, 150.0, 0.0, 0.0, 0.0, 0.0]
    te.apply_impulsive_force(wrench)
