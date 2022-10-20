#!/usr/bin/python3

# rospy - ROS Python API
import rospy
# intera_interface - Sawyer Python API
import intera_interface
import rospy
from intera_core_msgs.msg import IONodeConfiguration
import intera_dataflow
from intera_io import IODeviceInterface
import csv 

#Gripper
###########################################################################

class Gripper(object):
    """
    Interface class for a gripper on the Intera Research Robot.
    """
    MAX_POSITION = 0.041667
    MIN_POSITION = 0.0
    MAX_VELOCITY = 3.0
    MIN_VELOCITY = 0.15

    def __init__(self, ee_name="right_gripper", calibrate=True):
        """
        Constructor.

        @type name: str
        @param name: robot gripper name (default: "right_gripper")
        @type calibrate: bool
        @param calibrate: Attempts to calibrate the gripper when initializing class (defaults True)
        """
        self.devices = None
        self.name = ee_name
        if ee_name == 'right' or ee_name == 'left':
            rospy.logwarn(("Specifying gripper by 'side' is deprecated, please use full name"
                " (ex: 'right_gripper')."))
            self.name = '_'.join([ee_name, 'gripper'])
        self.ee_config_sub = rospy.Subscriber('/io/end_effector/config', IONodeConfiguration, self._config_callback)
        # Wait for the gripper device status to be true
        intera_dataflow.wait_for(
            lambda: not self.devices is None, timeout=5.0,
            timeout_msg=("Failed to get gripper. No gripper attached on the robot.")
        )

        self.gripper_io = IODeviceInterface("end_effector", self.name)
        if self.has_error():
            self.reboot()
            calibrate = True
        if calibrate and not self.is_calibrated():
            self.calibrate()

    def _config_callback(self, msg):
        """
        config topic callback
        """
        if msg.devices != []:
            if str(msg.devices[0].name) == self.name:
                self.devices = msg

    def reboot(self):
        """
        Power cycle the gripper, removing calibration information.
        """
        self.gripper_io.set_signal_value("reboot", True)

    def stop(self):
        """
        Set the gripper to stop executing command at the current
        position, apply holding force.
        """
        self.gripper_io.set_signal_value("go", False)

    def start(self):
        """
        Set the gripper to start executing command at the current
        position, apply holding force.
        """
        self.gripper_io.set_signal_value("go", True)

    def open(self, position=MAX_POSITION):
        """
        Set the gripper position to open by providing opening position.

        @type: float
        @param: the postion of gripper in meters
        """
        self.gripper_io.set_signal_value("position_m", position)

    def close(self, position=MIN_POSITION):
        """
        Set the gripper position to close by providing closing position.

        @type: float
        @param: the postion of gripper in meters
        """
        self.gripper_io.set_signal_value("position_m", position)

    def has_error(self):
        """
        Returns a bool describing whether the gripper is in an error state.
        (True: Error detected, False: No errors)

        @rtype: bool
        @return: True if in error state, False otherwise
        """
        return self.gripper_io.get_signal_value("has_error")

    def is_ready(self):
        """
        Returns bool describing if the gripper ready, i.e. is
        calibrated, not busy (as in resetting or rebooting), and
        not moving.

        @rtype: bool
        @return: True if in ready state, False otherwise
        """
        return (self.is_calibrated() and not self.has_error()
          and not self.is_moving())

    def is_moving(self):
        """
        Returns bool describing if the gripper is moving.

        @rtype: bool
        @return: True if in moving state, False otherwise
        """
        return self.gripper_io.get_signal_value("is_moving")

    def is_gripping(self):
        """
        Returns bool describing if the gripper is gripping.

        @rtype: bool
        @return: True if in gripping state, False otherwise
        """
        return self.gripper_io.get_signal_value("is_gripping")

    def is_calibrated(self):
        """
        Returns bool describing if the gripper is calibrated.

        @rtype: bool
        @return: True if successfully calibrated, False otherwise
        """
        return self.gripper_io.get_signal_value("is_calibrated")

    def calibrate(self):
        """
        Calibrate the gripper in order to set maximum and
        minimum travel distance.

        @rtype: bool
        @return: True if calibration was successful, False otherwise
        """
        self.gripper_io.set_signal_value("calibrate", True)
        success = intera_dataflow.wait_for(
            lambda: self.is_calibrated(),
            timeout=5.0,
            raise_on_error=False
        )
        if not success:
            rospy.logerr("({0}) calibration failed.".format(self.name))
        return success

    def get_position(self):
        """
        Returns float describing the gripper position in meters.

        @rtype: float
        @return: Current Position value in Meters (m)
        """
        return self.gripper_io.get_signal_value("position_response_m")

    def set_position(self, position):
        """
        Set the position of gripper.

        @type: float
        @param: the postion of gripper in meters (m)
        """
        self.gripper_io.set_signal_value("position_m", position)

    def set_velocity(self, speed):
        """
        DEPRECATED: Use set_cmd_velocity(speed)

        Set the velocity at which the gripper position movement will execute.

        @type: float
        @param: the velocity of gripper in meters per second (m/s)
        """
        rospy.logwarn("The set_velocity function is DEPRECATED. Please "
                      "update your code to use set_cmd_velocity() instead")
        self.set_cmd_velocity(speed)

    def set_cmd_velocity(self, speed):
        """
        Set the commanded velocity at which the gripper position
        movement will execute.

        @type: float
        @param: the velocity of gripper in meters per second (m/s)
        """
        self.gripper_io.set_signal_value("speed_mps", speed)

    def get_cmd_velocity(self):
        """
        Get the commanded velocity at which the gripper position
        movement executes.

        @rtype: float
        @return: the velocity of gripper in meters per second (m/s)
        """
        return self.gripper_io.get_signal_value("speed_mps")

    def get_force(self):
        """
        Returns the force sensed by the gripper in estimated Newtons.

        @rtype: float
        @return: Current Force value in Newton-Meters (N-m)
        """
        return self.gripper_io.get_signal_value("force_response_n")

    def set_object_weight(self, object_weight):
        """
        Set the weight of the object in kilograms.

        Object mass is set as a point mass at the location of the tool endpoint
        link in the URDF (e.g. 'right_gripper_tip'). The robot's URDF and
        internal robot model will be updated to compensate for the mass.

        @type: float
        @param: the object weight in kilograms (kg)
        """
        self.gripper_io.set_signal_value(self.name+"_tip_object_kg", object_weight)

    def get_object_weight(self):
        """
        Get the currently configured weight of the object in kilograms.

        Object mass is set as a point mass at the location of the tool endpoint
        link in the URDF (e.g. 'right_gripper_tip'). The robot's URDF and
        internal robot model will be updated to compensate for the mass.

        @rtype: float
        @return: the object weight in kilograms (kg)
        """
        return self.gripper_io.get_signal_value(self.name+"_tip_object_kg")

    def set_dead_zone(self, dead_zone):
        """
        Set the gripper dead zone describing the position error threshold
        where a move will be considered successful.

        @type: float
        @param: the dead zone of gripper in meters
        """
        self.gripper_io.set_signal_value("dead_zone_m", dead_zone)

    def get_dead_zone(self):
        """
        Get the gripper dead zone describing the position error threshold
        where a move will be considered successful.

        @rtype: float
        @return: the dead zone of gripper in meters
        """
        return self.gripper_io.get_signal_value("dead_zone_m")

    def set_holding_force(self, holding_force):
        """
        @deprecated: Function deprecated. Holding force is now a fixed value.
        """
        rospy.logerr("Removed variable holding force to improve gripper performance.")
        return False
###########################################################################

# Robot programming

rospy.init_node('Hello_Sawyer')
# create an instance of intera_interface's Limb class
limb = intera_interface.Limb('right')
# get the right limb's current joint angles
# angles = limb.joint_angles()
# print the current joint angles
# print (angles)
# move to neutral pose
#limb.move_to_neutral()
# get the right limb's current joint angles now that it is in neutral
#angles = limb.joint_angles()
# print the current joint angles again
#print (angles)
# reassign new joint angles (all zeros) which we will later command to the limb
# angles['right_j0']=0.0
# angles['right_j1']=0.0
# angles['right_j2']=0.0
# angles['right_j3']=0.0
# angles['right_j4']=0.0
# angles['right_j5']=0.0
# angles['right_j6']=0.0

# angles['right_j0']=1.5152685546875
# angles['right_j1']=-0.578564453125
# angles['right_j2']=-1.373470703125
# angles['right_j3']=1.256453125
# angles['right_j4']=0.0870078125
# angles['right_j5']=0.40004296875
# angles['right_j6']=-0.6204775390625

# print the joint angle command
# print (angles)

# move the right arm to those joint angles
# limb.move_to_joint_positions(angles)

# Pos 1
pos1 = {'right_j6': -0.6204775390625, 'right_j5': 0.40004296875, 'right_j4': 0.0870078125, 'right_j3': 1.256453125, 'right_j2': -1.373470703125, 'right_j1': -0.578564453125, 'right_j0': 1.5152685546875}

# Pos 2
pos2 = {'right_j6': -0.3439638671875, 'right_j5': -0.161685546875, 'right_j4': 1.7134892578125, 'right_j3': 1.2511650390625, 'right_j2': -2.17815234375, 'right_j1': 1.1135810546875, 'right_j0': 0.751255859375}

# Pos 3
pos3 = {'right_j6': -0.3439638671875, 'right_j5': -0.1278330078125, 'right_j4': 1.64938671875, 'right_j3': 1.062359375, 'right_j2': -2.3747998046875, 'right_j1': 1.019966796875, 'right_j0': 0.529359375}

# Pos 4
pos4 = {'right_j6': 0.09576953125, 'right_j5': -1.5409091796875, 'right_j4': 0.8081611328125, 'right_j3': 1.157330078125, 'right_j2': -1.9763974609375, 'right_j1': 0.7075166015625, 'right_j0': -0.5119140625}

# Pos 5
pos5 = {'right_j6': 0.0658154296875, 'right_j5': -1.28227734375, 'right_j4': 1.1207734375, 'right_j3': 1.1096953125, 'right_j2': -2.2556181640625, 'right_j1': 0.9975966796875, 'right_j0': -0.5409853515625}

# Pos 6
pos6 = {'right_j6': 0.2363818359375, 'right_j5': -1.430353515625, 'right_j4': 1.29594921875, 'right_j3': 1.572353515625, 'right_j2': -2.288607421875, 'right_j1': 1.2957939453125, 'right_j0': -0.5807529296875}

# Pos 7
limb.set_joint_position_speed(0.1)

GRP = Gripper()

LisT=[[0,0]]
while (True):
	with open("Coordinates.csv", newline='') as f:
		reader = csv.reader(f)
		for row in reader:
			L=row
	X=float(L[-2])
	Y=float(L[-1])
	if X<0.5 and X!=0:
		print("Move Left")
		limb.move_to_joint_positions(pos1)
		print("I'm at pos1")
		GRP.close()
		rospy.sleep(3.5)
	if X>0.5 and X<1:
		print("Move Right")
		limb.move_to_joint_positions(pos3)
		print("I'm at pos3")
		GRP.open()
		rospy.sleep(3.5)
client_socket.close()
