#!/usr/bin/env python3
"""
cyphal communicator
"""
import asyncio
import sys
import pathlib
import rospy
from std_msgs.msg import Bool, Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, Joy, MagneticField, NavSatFix


try:
    compiled_dsdl_dir = pathlib.Path(__file__).resolve().parent.parent / "compile_output"
    sys.path.insert(0, str(compiled_dsdl_dir))
    import pyuavcan.application
    import uavcan.node
    from reg.udral.service.actuator.common.sp import Vector4_0_1
    from reg.udral.service.common import Readiness_0_1
    from uavcan.primitive.scalar import Integer16_1_0 as Integer16
    import uavcan.si.sample.angular_velocity.Vector3_1_0
    import uavcan.si.sample.acceleration.Vector3_1_0
    import uavcan.si.sample.magnetic_field_strength.Vector3_1_0
    import uavcan.si.sample.temperature.Scalar_1_0
    import uavcan.si.sample.pressure.Scalar_1_0
    import uavcan.si.sample.angle.Scalar_1_0
    import reg.udral.physics.kinematics.geodetic.PointStateVarTs_0_1
except (ImportError, AttributeError):
    rospy.logerr(f"Cyphal communicator. Can't find compiled DSDL here {compiled_dsdl_dir}! Exit.")
    sys.exit()
REGISTER_FILE = "allocation_table.db"


class SetpointCyphalToRos:
    def __init__(self):
        self._ros_setpoint_pub = rospy.Publisher("/uav/actuators_raw", Joy, queue_size=10)
        self._actuators_msg = Joy()
        self._sp_sub = None

    def init(self, cyphal_node):
        self._sp_sub = cyphal_node.make_subscriber(Vector4_0_1, "setpoint")
        self._sp_sub.receive_in_background(self._sp_cb)

    async def _sp_cb(self, msg, _):
        self._actuators_msg.axes = msg.value
        self._actuators_msg.header.stamp = rospy.get_rostime()
        self._ros_setpoint_pub.publish(self._actuators_msg)


class ReadinessCyphalToRos:
    def __init__(self):
        self._ros_readiness_pub = rospy.Publisher("/uav/arm", Bool, queue_size=10)
        self._arm_msg = Bool()
        self._sp_readiness = None

    def init(self, cyphal_node):
        self._sp_readiness = cyphal_node.make_subscriber(Readiness_0_1, "readiness")
        self._sp_readiness.receive_in_background(self._readiness_cb)

    async def _readiness_cb(self, msg, _):
        ENGAGED = 3
        self._arm_msg.data = msg.value == ENGAGED
        self._ros_readiness_pub.publish(self._arm_msg)

class ImuRosToCyphal:
    def __init__(self):
        rospy.Subscriber("/uav/imu", Imu, self._ros_imu_cb)
        self._cyphal_imu_gyro_msg = uavcan.si.sample.angular_velocity.Vector3_1_0()
        self._cyphal_imu_accel_msg = uavcan.si.sample.acceleration.Vector3_1_0()
        self._loop = None

    def init(self, cyphal_node, loop):
        self._loop = loop

        cyphal_data_type = uavcan.si.sample.angular_velocity.Vector3_1_0
        reg_name = "gyro"
        self._imu_gyro_pub = cyphal_node.make_publisher(cyphal_data_type, reg_name)

        cyphal_data_type = uavcan.si.sample.acceleration.Vector3_1_0
        reg_name = "accel"
        self._imu_accel_pub = cyphal_node.make_publisher(cyphal_data_type, reg_name)

    def _ros_imu_cb(self, msg):
        self._cyphal_imu_gyro_msg.radian_per_second = [
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ]
        self._cyphal_imu_accel_msg.meter_per_second_per_second = [
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ]
        if self._loop is not None:
            self._loop.create_task(self.pub_imu())

    async def pub_imu(self):
        await self._imu_accel_pub.publish(self._cyphal_imu_accel_msg)
        await self._imu_gyro_pub.publish(self._cyphal_imu_gyro_msg)


class MagRosToCyphal:
    def __init__(self):
        rospy.Subscriber("/uav/mag", MagneticField, self._ros_mag_cb)
        self._cyphal_mag_msg = uavcan.si.sample.magnetic_field_strength.Vector3_1_0()
        self._loop = None

    def init(self, cyphal_node, loop):
        self._loop = loop

        cyphal_data_type = uavcan.si.sample.magnetic_field_strength.Vector3_1_0
        reg_name = "magnetometer"
        self._mag_pub = cyphal_node.make_publisher(cyphal_data_type, reg_name)

    def _ros_mag_cb(self, msg):
        self._cyphal_mag_msg = uavcan.si.sample.magnetic_field_strength.Vector3_1_0()
        self._cyphal_mag_msg.tesla = [
            msg.magnetic_field.x / 10000,
            msg.magnetic_field.y / 10000,
            msg.magnetic_field.z / 10000
        ]

        if self._loop is not None:
            self._loop.create_task(self.pub_mag())

    async def pub_mag(self):
        await self._mag_pub.publish(self._cyphal_mag_msg)

class BaroRosToCyphal:
    def __init__(self):
        rospy.Subscriber("/uav/baro_temperature", Float32, self._ros_baro_temperature_cb)
        rospy.Subscriber("/uav/baro_pressure", Float32, self._ros_baro_pressure_cb)
        self._cyphal_baro_temperature_msg = uavcan.si.sample.temperature.Scalar_1_0()
        self._cyphal_baro_pressure_msg = uavcan.si.sample.pressure.Scalar_1_0()
        self._loop = None

    def init(self, cyphal_node, loop):
        self._loop = loop

        cyphal_data_type = uavcan.si.sample.temperature.Scalar_1_0
        reg_name = "baro_temperature"
        self._baro_temperature_pub = cyphal_node.make_publisher(cyphal_data_type, reg_name)

        cyphal_data_type = uavcan.si.sample.pressure.Scalar_1_0
        reg_name = "baro_pressure"
        self._baro_pressure_pub = cyphal_node.make_publisher(cyphal_data_type, reg_name)

    def _ros_baro_temperature_cb(self, msg):
        self._cyphal_baro_temperature_msg = uavcan.si.sample.temperature.Scalar_1_0()
        self._cyphal_baro_temperature_msg.kelvin = msg.data

        if self._loop is not None:
            self._loop.create_task(self.pub_baro_temperature())

    def _ros_baro_pressure_cb(self, msg):
        self._cyphal_baro_pressure_msg = uavcan.si.sample.pressure.Scalar_1_0()
        self._cyphal_baro_pressure_msg.pascal = msg.data
        if self._loop is not None:
            self._loop.create_task(self.pub_baro_pressure())

    async def pub_baro_temperature(self):
        await self._baro_temperature_pub.publish(self._cyphal_baro_temperature_msg)

    async def pub_baro_pressure(self):
        await self._baro_pressure_pub.publish(self._cyphal_baro_pressure_msg)

class GpsRosToCyphal:
    def __init__(self):
        rospy.Subscriber("/uav/gps_point", NavSatFix, self._ros_gps_point_cb)

        rospy.Subscriber("/uav/velocity", Twist, self._ros_velocity_cb)
        self._ros_velocity_msg = Twist()

        self._gps_yaw_pub = None
        self._yaw_msg = uavcan.si.sample.angle.Scalar_1_0()

        self._gps_point_pub = None
        self._cyphal_point_msg = reg.udral.physics.kinematics.geodetic.PointStateVarTs_0_1()

        self._sats_pub = None
        self._cyphal_gps_sats_msg = Integer16()
        self._cyphal_gps_sats_msg.value = 20

        self._status_pub = None
        self._cyphal_gps_status_msg = Integer16()
        self._cyphal_gps_status_msg.value = 3  # STATUS_3D_FIX

        self._pdop_pub = None
        self._cyphal_gps_pdop_msg = Integer16()
        self._cyphal_gps_pdop_msg.value = 1.00

        self._loop = None

    def init(self, cyphal_node, loop):
        self._loop = loop

        cyphal_data_type = uavcan.si.sample.angle.Scalar_1_0
        reg_name = "gps_yaw"
        self._gps_yaw_pub = cyphal_node.make_publisher(cyphal_data_type, reg_name)

        cyphal_data_type = reg.udral.physics.kinematics.geodetic.PointStateVarTs_0_1
        reg_name = "gps_point"
        self._gps_point_pub = cyphal_node.make_publisher(cyphal_data_type, reg_name)

        self._sats_pub = cyphal_node.make_publisher(Integer16, "gps_sats")
        self._status_pub = cyphal_node.make_publisher(Integer16, "gps_status")
        self._pdop_pub = cyphal_node.make_publisher(Integer16, "gps_pdop")

    def _ros_gps_point_cb(self, ros_point_msg):
        if self._loop is None:
            return

        self._cyphal_point_msg.value.position.value.latitude = float(ros_point_msg.latitude) / 57.29577951308232
        self._cyphal_point_msg.value.position.value.longitude = float(ros_point_msg.longitude) / 57.29577951308232
        self._cyphal_point_msg.value.position.value.altitude.meter = float(ros_point_msg.altitude)
        self._cyphal_point_msg.value.velocity.value.meter_per_second = [
            self._ros_velocity_msg.linear.x,
            self._ros_velocity_msg.linear.y,
            self._ros_velocity_msg.linear.z,
        ]
        self._loop.create_task(self.pub_gps_point())

    def _ros_velocity_cb(self, msg):
        self._ros_velocity_msg = msg

    async def pub_gps_point(self):
        await self._gps_point_pub.publish(self._cyphal_point_msg)
        await self._sats_pub.publish(self._cyphal_gps_sats_msg)
        await self._status_pub.publish(self._cyphal_gps_status_msg)
        await self._pdop_pub.publish(self._cyphal_gps_pdop_msg)

class CyphalCommunicator:
    def __init__(self):
        self._node = None
        self._loop = None

        self.setpoint = SetpointCyphalToRos()
        self.readiness = ReadinessCyphalToRos()
        self.imu = ImuRosToCyphal()
        self.mag = MagRosToCyphal()
        self.baro = BaroRosToCyphal()
        self.gps = GpsRosToCyphal()

        self._log_ts_ms = rospy.get_time()

    async def log(self):
        if self._log_ts_ms + 1.0 < rospy.get_time():
            self._log_ts_ms = rospy.get_time()

    async def main(self):
        node_info = uavcan.node.GetInfo_1_0.Response(
                    software_version=uavcan.node.Version_1_0(major=1, minor=0),
                    name="vehicle_mock",
                )
        try:
            self._node = pyuavcan.application.make_node(node_info)
        except OSError as err:
            rospy.logerr("Cyphal communicator. SLCAN not found. Exit.")
            sys.exit()
        self._node.heartbeat_publisher.mode = uavcan.node.Mode_1_0.OPERATIONAL
        self._node.heartbeat_publisher.vendor_specific_status_code = 50
        self._node.start()

        self._loop = asyncio.get_event_loop()

        self.setpoint.init(self._node)
        self.readiness.init(self._node)
        self.imu.init(self._node, self._loop)
        self.mag.init(self._node, self._loop)
        self.baro.init(self._node, self._loop)
        self.gps.init(self._node, self._loop)

        while not rospy.is_shutdown():
            await asyncio.sleep(1)
            await self.log()


if __name__ == "__main__":
    rospy.init_node('vehicle_ros', anonymous=True)
    communicator = CyphalCommunicator()
    try:
        asyncio.run(communicator.main())
    except pyuavcan.application._node_factory.MissingTransportConfigurationError as err:
        rospy.logerr("Cyphal communicator. Registers not found. Did you source config.sh file? Exit.")
