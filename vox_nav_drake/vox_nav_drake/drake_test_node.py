#!/usr/bin/env python3

import rclpy
import rclpy.node
from geometry_msgs.msg import PoseWithCovarianceStamped
import rclpy.qos
from rclpy.executors import MultiThreadedExecutor

import pydrake
from pydrake.all import (VectorSystem, DiagramBuilder, SymbolicVectorSystem, LogVectorOutput,
                         Variable, Simulator, cos, sin)
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

        return -self.clip(u1, -1.5, 1.5), -self.clip(u2, -0.6, 0.6)

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

        target_x, target_y = 0, 0
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
        input[:] = self.lyapunov_controller(f_x, f_y, theta, f_theta, theta_d)


class RosNode(rclpy.node.Node):
    def __init__(self, *args):
        super(RosNode, self).__init__("rosnode")
        self.get_logger().info("Starting rosnode")

        self.create_subscription(
            PoseWithCovarianceStamped, "/vox_nav/cupoch/icp_base_to_map_pose",
            self.robot_pose_callback, rclpy.qos.qos_profile_sensor_data)

    def robot_pose_callback(self, msg):
        self.get_logger().info("recieved pose %s" % str(msg.pose.pose))


def main():

    print("Drake is installed under: ")
    print(pydrake.getDrakePath())

    x = Variable("x")
    y = Variable("y")
    theta = Variable("theta")        # vehicle orienttaion
    cartesian_state = [x, y, theta]
    v = Variable("v")                # driving velocity input
    delta = Variable("delta")        # steering velocity input
    input = [v, delta]

    # nonlinear dynamics, the whole state is measured (output = state)
    dynamics = [v * cos(theta),
                v * sin(theta),
                v * numpy.tan(delta)/L]

    robot = SymbolicVectorSystem(
        state=cartesian_state,
        input=input,
        output=cartesian_state,
        dynamics=dynamics,
    )

    # construction site for our closed-loop system
    builder = DiagramBuilder()

    # add the robot to the diagram
    # the method .AddSystem() simply returns a pointer to the system
    # we passed as input, so it's ok to give it the same name
    robot = builder.AddSystem(robot)

    # add the controller
    controller = builder.AddSystem(PointTrackingController())

    # wire the controller with the system
    builder.Connect(robot.get_output_port(0), controller.get_input_port(0))
    builder.Connect(controller.get_output_port(0), robot.get_input_port(0))

    # add a logger to the diagram
    # this will store the state trajectory
    logger = LogVectorOutput(robot.get_output_port(0), builder)

    # complete the construction of the diagram
    diagram = builder.Build()

    # set up a simulation environment
    simulator = Simulator(diagram)

    # set the initial cartesian state to a random initial position
    # try initial_state = np.random.randn(3) for a random initial state
    initial_state = [10, 16, 1.57]
    context = simulator.get_mutable_context()
    context.SetContinuousState(initial_state)

    # simulate from zero to sim_time
    # the trajectory will be stored in the logger
    integrator = simulator.get_mutable_integrator()
    target_accuracy = 1E-4
    integrator.set_target_accuracy(target_accuracy)
    maximum_step_size = 0.1
    integrator.set_maximum_step_size(maximum_step_size)
    minimum_step_size = 2E-5
    integrator.set_requested_minimum_step_size(minimum_step_size)
    integrator.set_throw_on_minimum_step_size_violation(True)
    integrator.set_fixed_step_mode(True)
    simulator.set_target_realtime_rate(1.0)

    rclpy.init()
    node = RosNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    while True:
        executor.spin_once(0.05)
        simulator.AdvanceTo(context.get_time() + 0.1)
        node.get_logger().info("Another step, time is: %s" % str(context.get_time()))


if __name__ == '__main__':
    main()
