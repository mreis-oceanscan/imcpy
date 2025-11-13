#!/usr/bin/env python3

# Standard imports
import numpy as np
import time, math
import signal as sg
from pyparsing import Callable

# imcpy imports
import imcpy
import imcpy.coordinates
from imcpy.actors.dynamic import DynamicActor
from imcpy.decorators import Periodic, Subscribe

# ROS2 imports
import rclpy, tf2_ros
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

# from utils.angles import quaternion_from_euler

def quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.

  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.

  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]


class VehicleWrapper(DynamicActor):
    '''
    Class to listen to IMC messages for a specific vehicle and store its state.
    '''
    def __init__(self, target_name, callback : Callable = None, frequency=1.0):
        """
        Initialize the actor
        :param target_name: The name of the target system
        :param callback: Callback function to call periodically
        :param frequency: Frequency in Hz for periodic callback
        """
        super().__init__()
        self.periodic_callback = callback
        self.set_frequency(frequency)

        # This list contains the target systems to maintain communications with
        self.heartbeat.append(target_name)

        self.vehicle = {
            'name': target_name,
            'imcaddress': None,
            'pose': {'x': 0.0, 'y': 0.0, 'z': 0.0, 
                     'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
            'fin_positions': {'top': 0.0, 'bottom': 0.0, 'port': 0.0, 'starboard': 0.0},
            'rpm': 0.0
        }

        # Initialize attributes that will be set by the parent node
        self.origin = {'x': 0.0, 'y': 0.0, 'z': 0.0}

    @Subscribe(imcpy.Announce)
    def onAnnounce(self, msg : imcpy.Announce):
        '''
        Callback when an Announce message is received.
        The vehicle name/IMC address is recovered.
        '''
        name = msg.sys_name
        # imc_address = msg._header.src

        if name != self.vehicle['name']:
            return

        self.vehicle['name'] = name
        print(f'VehicleWrapper: Announce received for vehicle {name}')

    @Subscribe(imcpy.EstimatedState)
    def onEstimatedState(self, msg : imcpy.EstimatedState):
        '''
        Callback when an EstimatedState message is received.
        The vehicle_name is passed directly from the connection setup.
        '''
        self.vehicle['pose']['x'] = msg.x
        self.vehicle['pose']['y'] = msg.y
        self.vehicle['pose']['z'] = msg.z

        self.vehicle['pose']['roll'] = msg.phi
        self.vehicle['pose']['pitch'] = msg.theta
        self.vehicle['pose']['yaw'] = msg.psi

    @Subscribe(imcpy.ServoPosition)
    def onServoPosition(self, msg : imcpy.ServoPosition):
        '''
        Callback when a ServoPosition message is received.
        The vehicle_name is passed directly from the connection setup.
        '''        
        match msg.id:
            case 0:
                self.vehicle['fin_positions']['bottom'] = msg.value
            case 1:
                self.vehicle['fin_positions']['port'] = msg.value
            case 2:
                self.vehicle['fin_positions']['starboard'] = msg.value
            case 3:
                self.vehicle['fin_positions']['top'] = msg.value

    @Subscribe(imcpy.Rpm)
    def onRpm(self, msg : imcpy.Rpm):
        '''
        Callback when a Rpm message is received.
        The vehicle_name is passed directly from the connection setup.
        '''        
        self.vehicle['rpm'] = msg.value

    @Periodic(dt=0.1)  # Default period, will be overridden
    def run_periodic(self):
        """
        Periodic callback function that calls the user-defined callback.
        The period is set dynamically after initialization.
        """
        if self.periodic_callback:
            self.periodic_callback()

    def set_frequency(self, frequency):
        """
        Set the frequency for the periodic callback.
        This needs to be called after initialization to update the period.
        """
        self.frequency = frequency
        self.period = 1.0 / frequency
        # Update the period for the existing periodic decorator
        if hasattr(self.run_periodic, '_decorators'):
            for decorator in self.run_periodic._decorators:
                if isinstance(decorator, Periodic):
                    decorator.dt = self.period
                    break


class VehicleUpdater(Node):
    '''
    ROS2 Node to update vehicle position, orientation and fin/propeller positions based on received IMC messages.
    '''
    def __init__(self):
        '''
        ROS2 related stuff.
        '''
        super().__init__('vehicle_updater')

        # Vehice transforms and joint states
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.transform = TransformStamped()
        self.transform.header.frame_id = 'world'  # Parent frame for the robot transform
        self.joint_state = JointState()

        # Declare ROS2 parameters
        self.declare_parameter('frequency', 100.0, 
                             ParameterDescriptor(description='Node update frequency in Hz'))
        self.declare_parameter('vehicle', 'lauv-simulator-1',
                             ParameterDescriptor(description='Vehicle name'))
        self.declare_parameter('origin', [0.0, 0.0, 0.0],
                             ParameterDescriptor(description='Origin for the vehicle'))

        self.add_on_set_parameters_callback(self.parameter_callback)

        freq = self.get_parameter('frequency').get_parameter_value().double_value
        vehicle_name = self.get_parameter('vehicle').get_parameter_value().string_value
        origin = self.get_parameter('origin').get_parameter_value().double_array_value

        self.set_frequency(freq)
        self.set_vehicle(vehicle_name)
        self.set_origin(origin)

        # Create the IMC wrapper with custom ros_loop callback and custom frequency
        self.wrapper = VehicleWrapper(self.vehicle_name, 
                                      callback=self.ros_loop,
                                      frequency=freq)
        
        self.start_time = time.time()

    def set_frequency(self, freq, verbose=False):
        '''
        Update the frequency of the node update timer.
        '''
        if hasattr(self, 'timer'):
            self.timer.cancel()
        self.timer = self.create_timer(1.0 / freq, self.update_robot)

        if hasattr(self, 'wrapper'):
            self.wrapper.set_frequency(freq)

        if verbose:
            self.get_logger().info(f'Update frequency set to: {freq} Hz')

    def set_vehicle(self, name, verbose=False):
        '''
        Update the vehicle name.
        '''
        self.vehicle_name = name
        namespace = name.replace('-', '_')
        self.joint_pub = self.create_publisher(JointState, f'/{namespace}/joint_states', 10)
        
        self.transform.child_frame_id = f'{namespace}/base_link'  # Child frame
        self.joint_state.name = [
            f'{namespace}/fin_top_joint',
            f'{namespace}/fin_bottom_joint',
            f'{namespace}/fin_port_joint',
            f'{namespace}/fin_starboard_joint',
            f'{namespace}/propeller_joint'
        ]

        if verbose:
            self.get_logger().info(f'Vehicle name set to: {self.vehicle_name}')

    def set_origin(self, origin, verbose=False):
        '''
        Sets the origin of the vehicles reference frame
        '''
        self.origin = {
            'x': origin[0], 
            'y': origin[1], 
            'z': origin[2]
            }
        
        if verbose:
            self.get_logger().info(f'Origin set to: {self.origin}')
        
    def parameter_callback(self, params):
        '''
        Callback for parameter changes.
        '''        
        for param in params:
            
            # Process frequency parameter
            if param.name == 'frequency':
                self.set_frequency(param.value)
                self.get_logger().info(f'Frequency parameter changed to {self.update_frequency} Hz')
                continue

            # Process vehicle name parameter
            if param.name == 'vehicle':
                self.set_vehicle(param.value)
                self.get_logger().info(f'Vehicle name updated: {self.vehicle_name}')
                continue

            # Process origin parameter
            if param.name == 'origin':
                self.set_origin(param.value)
                self.get_logger().info(f'Origin updated: {self.origin}')
                continue

        return SetParametersResult(successful=True)
        
    def update_robot(self, verbose=False):
        '''
        Update vehicle pose transform for a specific vehicle.
        '''
        self.get_logger().info('Updating vehicle transform and joint states.')

        # Set transform and jointstate timestamps   
        ros_time = self.get_clock().now().to_msg()
        self.transform.header.stamp = ros_time
        self.joint_state.header.stamp = ros_time

        # Retrieve data from vehicle wrapper
        name = self.wrapper.vehicle['name']
        pose = self.wrapper.vehicle['pose']
        fin_pos = self.wrapper.vehicle['fin_positions']
        rpm = self.wrapper.vehicle['rpm']

        if verbose:
            x = pose['x']
            y = pose['y']
            z = pose['z']
            self.get_logger().info(f'{name}: x = {x:.2f}, y = {y:.2f}, z = {z:.2f}')

            roll = pose['roll'] * (180.0 / math.pi)
            pitch = pose['pitch'] * (180.0 / math.pi)
            yaw = pose['yaw'] * (180.0 / math.pi)
            self.get_logger().info(f'{name}: roll = {roll:.2f}, pitch = {pitch:.2f}, yaw = {yaw:.2f}')

        # Set position
        self.transform.transform.translation.x = pose['x']
        self.transform.transform.translation.y = pose['y']
        self.transform.transform.translation.z = pose['z']

        # Set orientation
        q = quaternion_from_euler(pose['roll'], pose['pitch'], pose['yaw'])

        self.transform.transform.rotation.x = q[0]
        self.transform.transform.rotation.y = q[1]
        self.transform.transform.rotation.z = q[2]
        self.transform.transform.rotation.w = q[3]

        # Update joint positions
        current_time = time.time() - self.start_time
        propeller_speed = rpm / (2 * math.pi)  # Convert RPM to rad/s
        propeller_position = propeller_speed * current_time

        self.joint_state.position = [
            -fin_pos['top'],         # fin_top
            fin_pos['bottom'],       # fin_bottom
            fin_pos['port'],         # fin_port
            -fin_pos['starboard'],   # fin_starboard
            propeller_position             # propeller (continuous rotation)
        ]

        self.joint_state.velocity = []
        self.joint_state.effort = []

        if verbose:
            self.get_logger().info(f'{name} - RPM: {rpm}, fins:')
            self.get_logger().info(f'         {fin_pos["top"]}          ')
            self.get_logger().info(f'{fin_pos["port"]}                {fin_pos["port"]}')
            self.get_logger().info(f'         {fin_pos["bottom"]}          ')

        # Publish transform and joint state
        self.tf_broadcaster.sendTransform(self.transform)
        self.joint_pub.publish(self.joint_state)

    def ros_loop(self):
        rclpy.spin_once(self, timeout_sec=0)

    def run(self):
        self.wrapper.run()

if __name__ == '__main__':

    rclpy.init()
    mover_node = VehicleUpdater()

    def signal_handler(sig, frame):
        mover_node.get_logger().info('Vehicle stopped by user (Ctrl+C)')
        raise KeyboardInterrupt()
    
    # Register signal handler for Ctrl+C
    sg.signal(sg.SIGINT, signal_handler)

    try:
        mover_node.run()
    except KeyboardInterrupt:
        mover_node.get_logger().info('Vehicle has stopped.')
    finally:
        mover_node.destroy_node()
        rclpy.shutdown()