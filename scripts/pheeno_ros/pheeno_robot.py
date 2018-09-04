"""
Pheeno Robot Python Class files for coding

Notes
-----
Contains files directly related to Pheeno Robot movement, sensors, etc. This
object class can be used in other python scripts to give one direct control
of the Pheeno without needing to write extra attributes, callbacks, etc.

Written by: Zahi Kakish (zmk5)
License: BSD 3-Clause

"""
import random
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32
from std_msgs.msg import Int16


class PheenoRobot(object):
    """
    PheenoRobot Class

    Provides a easier way of generating all the necessary attributes, ROS
    publishers and subscribers, and useful methods for running a Pheeno robot.
    Just instantiate the class to a variable within a script and all the
    necessary ROS calls are generated automatically. Many of the attributes
    of the Pheeno are hidden from users (except velocity ones) and require
    getters to get them.

    Parameters
    ----------
    robot_id : str
        ID number of the pheeno.
    linear : int/float
        Initial linear velocity of the robot.
    angular : int/float
        Initial angular velocity of the robot.
    obstacle_velocity : int/float
        Velocity for obstacle avoidance. In terms of angular velocity, may be faster or
        slower than the current value of angular velocity.

    Attributes
    ----------
    pheeno_id : str
        The ID of the pheeno that is used as the namespace for pheeno ROS publishers and
        subscribers.
    linear_velocity : float (default 0)
        The linear velocity of the robot.
    angular_velocity : float (default 0)
        The angular velocity of the robot.
    obstacle_velocity : float (default 0.5)
        If a robot approaces an obstacle, this is the angular velocity used for obstacle
        avoidance.

    Methods
    -------
    get_ir_data
        Returns the IR data for a specific sensor. If the "all" option is
        used, then all the data is returned as a list.
        Options: "center", "back", "right", "left", "cr", "cl", "all"
    get_encoder_data
        Returns encoder data for a specific encoder. If the "all" option is
        used, then all the data is returned as a list.
        Options: "LL", "LR", "RL", "RR", "all"
    get_magnetometer_data
        Returns magnetometer data for a specific coordinate. If the "all" option
        is used, then all the data is returned as a list.
        Options: "x", "y", "z", "all"
    get_gyroscope_data
        Returns gyroscope data for a specific coordinate. If "all" option is
        used, then all the data is returned as a list.
        Options: "x", "y", "z", "all"
    get_accelerometer_data
        Returns accelerometer data for a specific coordinate. If "all" option
        is used, then all the data is returned as a list.
        Options: "x", "y", "z", "all"
    avoid_obstacle
        Obstacle avoidance algorithm based on IR sensors. Call when wanting to
        plan motion while still trying to avoid obstacles.
    is_obstacle_detected
        Bool return if any IR sensor has been tripped.
    is_ir_sensor_triggered
        Bool return if a SPECIFIC IR sensor has been tripped.

    Examples
    --------
    >>> # If you have more than 10 robots add a `0` to the first
    >>> # 9 numbers.
    >>> pheeno = PheenoRobot("01", linear_vel=0.8, angular_vel=0)

    """
    def __init__(self, robot_id, linear_velocity=0, angular_velocity=0, obstacle_velocity=0.5):
        if robot_id == "":
            self.pheeno_id = robot_id

        else:
            self.pheeno_id = "/pheeno_" + str(robot_id)

        self._velocity = {"linear": float(linear_velocity),
                          "angular": float(angular_velocity),
                          "obstacle": float(obstacle_velocity)}

        # Sensor Values
        self._sensor_data = {"ir": {"center": 0.0,
                                    "back": 0.0,
                                    "right": 0.0,
                                    "left": 0.0,
                                    "cr": 0.0,
                                    "cl": 0.0},
                             "odom": {"position": 0.0,
                                      "orient": 0.0,
                                      "linear": 0.0,
                                      "angular": 0.0},
                             "encoder": {"LL": 0,
                                         "LR": 0,
                                         "RL": 0,
                                         "RR": 0},
                             "magnetometer": {"x": 0.0,
                                              "y": 0.0,
                                              "z": 0.0},
                             "gyroscope": {"x": 0.0,
                                           "y": 0.0,
                                           "z": 0.0},
                             "accelerometer": {"x": 0.0,
                                               "y": 0.0,
                                               "z": 0.0}}

        # cmd_vel Publisher
        self._pub = rospy.Publisher(self.pheeno_id + "/cmd_vel",
                                    Twist,
                                    queue_size=100)

        # IR Sensor Subscribers
        rospy.Subscriber(self.pheeno_id + "/scan_center", Float32,
                         self.__callback_ir_sensor, callback_args="center")
        rospy.Subscriber(self.pheeno_id + "/scan_back", Float32,
                         self.__callback_ir_sensor, callback_args="back")
        rospy.Subscriber(self.pheeno_id + "/scan_right", Float32,
                         self.__callback_ir_sensor, callback_args="right")
        rospy.Subscriber(self.pheeno_id + "/scan_left", Float32,
                         self.__callback_ir_sensor, callback_args="left")
        rospy.Subscriber(self.pheeno_id + "/scan_cr", Float32,
                         self.__callback_ir_sensor, callback_args="cr")
        rospy.Subscriber(self.pheeno_id + "/scan_cl", Float32,
                         self.__callback_ir_sensor, callback_args="cl")

        # Encoder Subscribers
        rospy.Subscriber(self.pheeno_id + "/encoder_LL", Int16,
                         self.__callback_encoder, callback_args="LL")
        rospy.Subscriber(self.pheeno_id + "/encoder_LR", Int16,
                         self.__callback_encoder, callback_args="LR")
        rospy.Subscriber(self.pheeno_id + "/encoder_RL", Int16,
                         self.__callback_encoder, callback_args="RL")
        rospy.Subscriber(self.pheeno_id + "/encoder_RR", Int16,
                         self.__callback_encoder, callback_args="RR")

        # Magnetometer, Gyroscope, Accelerometer Subscriber
        rospy.Subscriber(self.pheeno_id + "/magnetometer", Vector3,
                         self.__callback_magnetometer)
        rospy.Subscriber(self.pheeno_id + "/gyroscope", Vector3,
                         self.__callback_gyroscope)
        rospy.Subscriber(self.pheeno_id + "/accelerometer", Vector3,
                         self.__callback_accelerometer)

    def __callback_ir_sensor(self, msg, location):
        """ Callback for IR sensors subscriber """
        self._sensor_data["ir"][location] = msg.data

    def __callback_encoder(self, msg, location):
        """ Callback for Encoder sensors subscriber """
        self._sensor_data["encoder"][location] = msg.data

    def __callback_magnetometer(self, msg):
        """ Callback for Magnetometer sensor subscriber """
        self._sensor_data["magnetometer"]["x"] = msg.x
        self._sensor_data["magnetometer"]["y"] = msg.y
        self._sensor_data["magnetometer"]["z"] = msg.z

    def __callback_gyroscope(self, msg):
        """ Callback for Gyroscope sensor subscriber """
        self._sensor_data["gyroscope"]["x"] = msg.x
        self._sensor_data["gyroscope"]["y"] = msg.y
        self._sensor_data["gyroscope"]["z"] = msg.z

    def __callback_accelerometer(self, msg):
        """ Callback for Accelerometer sensor subscriber """
        self._sensor_data["accelerometer"]["x"] = msg.x
        self._sensor_data["accelerometer"]["y"] = msg.y
        self._sensor_data["accelerometer"]["z"] = msg.z

    @property
    def linear_velocity(self):
        """ Linear velocity property """
        return self._velocity["linear"]

    @linear_velocity.setter
    def linear_velocity(self, value):
        """ Linear velocity setter property """
        if -1.0 < value < 1.0:
            self._velocity["linear"] = value

        else:
            raise ValueError("Motor values must be between -1 and 1.")

    @property
    def angular_velocity(self):
        """ Angular velocity property """
        return self._velocity["angular"]

    @angular_velocity.setter
    def angular_velocity(self, value):
        """ Angular velocity setter property """
        if -1.0 < value < 1.0:
            self._velocity["angular"] = value

        else:
            raise ValueError("Motor values must be between -1 and 1.")

    @property
    def obstacle_velocity(self):
        """ Obstacle velocity property """
        return self._velocity["obstacle"]

    @obstacle_velocity.setter
    def obstacle_velocity(self, value):
        """ Obstacle velocity setter property """
        if 0 < value < 1.0:
            self._velocity["obstacle"] = value

        else:
            raise ValueError("Obstacle velocity should be an positive and between (0 < v < 1)")

    def get_ir_data(self, key):
        """ Getter for IR Sensor data """
        if key == "all":
            values = []
            for sensor in ["center", "back", "right", "left", "cr", "cl"]:
                values.append(self._sensor_data["ir"][sensor])

            return values

        return self._sensor_data["ir"][key]

    def get_encoder_data(self, key):
        """ Getter for Encoder data """
        if key == "all":
            values = []
            for sensor in ["LL", "LR", "RL", "RR"]:
                values.append(self._sensor_data["encoder"][sensor])

            return values

        return self._sensor_data["encoder"][key]

    def get_magnetometer_data(self, key):
        """ Getter for Magnetometer data """
        if key == "all":
            values = []
            for sensor in ["x", "y", "z"]:
                values.append(self._sensor_data["magnetometer"][sensor])

            return values

        return self._sensor_data["magnetometer"][key]

    def get_gyroscope_data(self, key):
        """ Getter for Gyroscope data """
        if key == "all":
            values = []
            for sensor in ["x", "y", "z"]:
                values.append(self._sensor_data["gyroscope"][sensor])

            return values

        return self._sensor_data["gyroscope"][key]

    def get_accelerometer_data(self, key):
        """ Getter for Accelerometer data """
        if key == "all":
            values = []
            for sensor in ["x", "y", "z"]:
                values.append(self._sensor_data["accelerometer"][sensor])

            return values

        return self._sensor_data["accelerometer"][key]

    def publish_cmd_vel(self, twist_msg):
        """ Publish an movement  velocity on the `cmd_vel` topic """
        self._pub.publish(twist_msg)

    def avoid_obstacle(self, limit, linear_vel, angular_vel):
        """
        Obstacle avoidance algorithm

        Given a desired linear and angular velocity, the algorithm will set those
        values as the robots linear and angular velocity if no obstacle is in
        the way. If there is, the linear velocity of the robot is set to zero,
        and the angular velocity is set of a positive or negative version of the
        obstacle velocity robot attribute depending on certain conditions.

        Arguments
        ---------
        limit : int/float
            The ir sensor limit.
        linear_vel : float
            The desired linear velocity a user would like to set if there are
            no obstacles in the way.
        angular_vel : float
            The desired angular velocity a user would like to set if there are
            no obstacles in the way.

        Examples
        -------
        For a robot moving along a linear path, the method may be called as such:
        >>> ir_limit = 15  # in cm
        >>> desired_linear = 0.7
        >>> desired_angular = 0  # b/c we don't want it to turn
        >>> pheeno.avoid_obstacle(ir_limit, desired_linear, desired_angular)

        If no obstacle is in the path of the robot, the attributes `linear_velocity`
        and `angular_velocity` are set to 0.7 and 0, respectively. If not, the
        values are set based on the internal logic of the algorithm. Assign these
        values to the proper places in a `geometry_msg.Twist()` msg and call the
        `publish_cmd_vel(twist_msg)` method to enact the changes.


        """
        if self.is_obstacle_detected(limit):
            if self.is_ir_sensor_triggered("center", limit):
                if (self._sensor_data["ir"]["right"] < 10 and
                        self._sensor_data["ir"]["left"] < 10):
                    angular = self.random_turn() * self._velocity["obstacle"]

                if self._sensor_data["ir"]["right"] < self._sensor_data["ir"]["left"]:
                    angular = -1 * self._velocity["obstacle"]  # Turn Left

                else:
                    angular = self._velocity["obstacle"]  # Turn Right

            elif (self.is_ir_sensor_triggered("cl", limit) and
                  self.is_ir_sensor_triggered("cr", limit)):
                angular = self.random_turn() * self._velocity["obstacle"]

            # If individual IR sensors are triggered.
            else:
                angular = self._velocity["obstacle"]

            # Set linear velocity to 0, so only angular velocity will occur.
            linear = 0

        # If no obstacle detected, use the default terms.
        else:
            angular = angular_vel
            linear = linear_vel

        # Set new velocities
        self._velocity["linear"] = linear
        self._velocity["angular"] = angular

    def is_obstacle_detected(self, limit):
        """ Returns bool if an obstacle is in the way of any IR sensor """
        for key in self._sensor_data["ir"]:
            if self._sensor_data["ir"][key] <= limit:
                return True

        return False

    def is_ir_sensor_triggered(self, key, limit):
        """ Returns bool if the specific IR sensor is triggered """
        if self._sensor_data["ir"][key] <= limit:
            return True

        return False

    @staticmethod
    def random_turn():
        """ Given a velocity, generate a random turn direction """
        if random.random() < 0.5:
            rospy.loginfo("Turning Left")
            return -1.0

        rospy.loginfo("Turning Right")
        return 1.0
