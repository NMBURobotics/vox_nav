#!/usr/bin/env python3

from nav_msgs.msg import Odometry
import math
from time import sleep
from geometry_msgs.msg import TransformStamped
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer

from geometry_msgs.msg import PoseStamped
import rclpy
import rclpy.node
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PoseStamped
import rclpy.qos
from rclpy.executors import MultiThreadedExecutor

import pydrake
from pydrake.all import (VectorSystem, DiagramBuilder, SymbolicVectorSystem, LogVectorOutput,
                         Variable, Simulator, cos, sin)
from pydrake.all import (
    Adder,
    ZeroOrderHold, FirstOrderLowPassFilter
)

import numpy

L = 0.6
K_us = 1.0
g = 9.81
# you will write the control law in it in the next cell
# by defining the function called "lyapunov_controller"


class PointTrackingController(VectorSystem):

    def __init__(self):
        # 4 inputs (robot state)
        # 2 outputs (robot inputs)
        VectorSystem.__init__(self, 3, 2)

    def euclidean_dist(self, x, y, x_obs, y_obs):
        # x and y are vehicle pose
        return numpy.sqrt((x-x_obs)**2) + ((y-y_obs)**2)

    def clip(self, val, min_, max_):
        return min_ if val < min_ else max_ if val > max_ else val

    def lyapunov_controller(self, f_x, f_y, theta, f_theta, theta_d):
        k1 = 1E-4
        k2 = 1E-1
        u1 = k1 * numpy.copysign(1, (f_x*cos(theta) + f_y *
                                     sin(theta))) * (f_x**2 + f_y ** 2 + f_theta**2)

        p = u1*(f_x * cos(theta) + f_y * sin(theta)) + \
            k2 * (theta_d - theta) * f_theta
        if p >= 0.0:
            u2 = k2 * (theta_d - theta) * ((L + K_us/g * u1**2)/u1)
        else:
            u2 = -u1 / f_theta * (f_x * cos(theta) + f_y *
                                  sin(theta)) * ((L + K_us/g * u1**2)/u1)

        return -self.clip(u1, -0.5, 0.5), -self.clip(u2, -0.6, 0.6)

    def lyapunov_controller_ddf(self, x1, x2, x3):

        u1 = - x1 * cos(x3)
        u2 = x3+(x3+x2)*cos(x3)*sin(x3) / x3
        if x1 < 0.1:
            u1, u2 = 0, 0

        return self.clip(u1, -0.4, 0.4), self.clip(u2, -0.3, 0.3)

    def DoCalcVectorOutput(
        self,
        context,           # not used
        cartesian_state,   # input of the controller
        controller_state,  # not used
        input              # output of the controller
    ):

        kLAMBDA = 0.0
        kK = 0.05
        kPULL = 10

        # upack state of the robot x,y,theta
        x, y, theta = cartesian_state

        target_x, target_y = 10, -10
        dist = self.euclidean_dist(x, y, target_x, target_y)
        beta = kPULL*dist

        if x >= 0.0:
            f_x = 3 * (x**2) * (beta**(1.0/kK)) + (y**2)*(beta**(1.0/kK)) + kLAMBDA * (theta**2) * (beta**(1.0/kK)) + \
                (x*(x - target_x)*(x**2 + y**2 + kLAMBDA*(theta**2)) * (beta **
                 ((-kK+1.0)/kK))) / kK * numpy.sqrt((x-target_x)**2 + (y - target_y)**2)

            f_y = x * (((y - target_y)*(x**2 + y**2 + kLAMBDA*(theta**2)) * (beta**((-kK+1.0)/kK))
                        ) / kK * numpy.sqrt((x-target_x)**2 + (y - target_y)**2) + 2 * y * (beta**(1.0/kK)))
            f_theta = 2 * kLAMBDA * theta * (beta**(1.0/kK))
        else:
            f_x = -3 * (x**2) * (beta**(1/kK)) - (y**2)*(beta**(1/kK)) - kLAMBDA * (theta**2) * (beta**(1.0/kK)) - \
                (x*(x - target_x)*(x**2 + y**2 + kLAMBDA*(theta**2)) * (beta **
                 ((-kK+1.0)/kK))) / kK * numpy.sqrt((x-target_x)**2 + (y - target_y)**2)

            f_y = -x * (((y - target_y)*(x**2 + y**2 + kLAMBDA*(theta**2)) * (beta**((-kK+1)/kK))) /
                        kK * numpy.sqrt((x-target_x)**2 + (y - target_y)**2) + 2 * y * (beta**(1.0/kK)))
            f_theta = - 2 * kLAMBDA * theta * (beta**(1.0/kK))

        theta_d = numpy.arctan2(-numpy.copysign(1, x)*f_y,
                                -numpy.copysign(1, x)*f_x)

        # evaluate the function below and return the robot's input
        # input[:] = self.lyapunov_controller(f_x, f_y, theta, f_theta, theta_d)
        # upack state of the robot
        z1, z2, z3 = cartesian_state

        # state in polar coordinates
        x1 = numpy.sqrt((z1-target_x)**2 + (z2-target_y)
                        ** 2)  # radial coordinate
        x2 = numpy.arctan2((z2-target_y), (z1-target_x)
                           )     # angular coordinate
        x3 = x2 - z3
        input[:] = self.lyapunov_controller_ddf(x1, x2, x3)


class RosNode(rclpy.node.Node, VectorSystem):
    def __init__(self, *args):

        super(RosNode, self).__init__("rosnode")
        # 2 inputs (robot state)
        # 3 outputs (robot inputs)
        VectorSystem.__init__(self, 2, 3)

        self.get_logger().info("Starting rosnode")
        self.create_subscription(
            Odometry, "/odometry/base_raw",
            self.robot_pose_callback, 1)

        self.twist_pub = self.create_publisher(
            Twist, "vox_nav/cmd_vel", rclpy.qos.qos_profile_sensor_data)

        self.latest_robot_states = Odometry()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def __del__(self):
        # body of destructor
        self.twist_pub.publish(Twist())

    def robot_pose_callback(self, msg):
        self.get_logger().info("recieved pose %s" % str(msg))
        self.latest_robot_states = msg

    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)

        return roll_x, pitch_y, yaw_z  # in radians

    def get_curr_robot_pose(self):

        now = rclpy.time.Time()
        curr_robot_pose = PoseStamped()
        curr_robot_pose.header.frame_id = "map"
        curr_robot_pose.header.stamp = now.to_msg()
        trans = TransformStamped()
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                "odom",
                "base_link",
                now)

            curr_robot_pose.pose.position.x = trans.transform.translation.x
            curr_robot_pose.pose.position.y = trans.transform.translation.y
            curr_robot_pose.pose.position.z = trans.transform.translation.z
            curr_robot_pose.pose.orientation = trans.transform.rotation

        except TransformException as ex:
            self.get_logger().info('"Failed to get current robot pose"')
            curr_robot_pose.pose.orientation.w = 1.0
            return None

        return curr_robot_pose

    def DoCalcVectorOutput(
        self,
        context,            # not used
        control_commands,   # input of the robot
        controller_state,   # not used
        states              # output of the robot
    ):
        #curr = self.get_curr_robot_pose()
        curr = self.latest_robot_states.pose
        theta = self.euler_from_quaternion(
            curr.pose.orientation.x, curr.pose.orientation.y, curr.pose.orientation.z, curr.pose.orientation.w)[2]

        states[:] = curr.pose.position.x, curr.pose.position.y, theta

        v, w = control_commands
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        self.twist_pub.publish(msg)


def main():

    print("Drake is installed under: ")
    print(pydrake.getDrakePath())

    rclpy.init()
    robot = RosNode()
    executor = MultiThreadedExecutor()
    executor.add_node(robot)

    # construction site for our closed-loop system
    builder = DiagramBuilder()

    zoh = builder.AddSystem(ZeroOrderHold(0.001, 3))
    zoh.set_name("zoh")

    # filter = builder.AddSystem(FirstOrderLowPassFilter(
    #    time_constant=100.0, size=3))

    # add the robot to the diagram
    # the method .AddSystem() simply returns a pointer to the system
    # we passed as input, so it's ok to give it the same name
    robot = builder.AddSystem(robot)

    # add the controller
    controller = builder.AddSystem(PointTrackingController())

    # wire the controller with the system
    builder.Connect(robot.get_output_port(0), zoh.get_input_port(0))
    builder.Connect(zoh.get_output_port(0), controller.get_input_port(0))
    builder.Connect(controller.get_output_port(0), robot.get_input_port(0))

    # complete the construction of the diagram
    diagram = builder.Build()

    # set up a simulation environment
    simulator = Simulator(diagram)
    '''
    integrator = simulator.get_mutable_integrator()
    target_accuracy = 1E-4
    integrator.set_target_accuracy(target_accuracy)
    maximum_step_size = 0.05
    integrator.set_maximum_step_size(maximum_step_size)
    minimum_step_size = 2E-5
    integrator.set_requested_minimum_step_size(minimum_step_size)
    integrator.set_throw_on_minimum_step_size_violation(True)
    integrator.set_fixed_step_mode(True)
    '''
    simulator.set_target_realtime_rate(1.0)
    simulator.Initialize()

    # set the initial cartesian state to a random initial position
    # try initial_state = np.random.randn(3) for a random initial state
    curr = robot.get_curr_robot_pose()
    while curr == None:
        executor.spin_once(0.05)
        robot.get_logger().info("waiting")
        sleep(0.2)
        curr = robot.get_curr_robot_pose()

    theta = robot.euler_from_quaternion(
        curr.pose.orientation.x,
        curr.pose.orientation.y,
        curr.pose.orientation.z,
        curr.pose.orientation.w)[2]

    initial_state = [curr.pose.position.x, curr.pose.position.y, theta]
    context = simulator.get_mutable_context()
    context.SetDiscreteState(initial_state)

    # simulate from zero to sim_time
    # the trajectory will be stored in the logger

    while True:
        executor.spin_once(0.)
        simulator.AdvanceTo(context.get_time() + 0.25)
        robot.get_logger().info("Another step, time is: %s" % str(context.get_time()))


if __name__ == '__main__':
    main()
