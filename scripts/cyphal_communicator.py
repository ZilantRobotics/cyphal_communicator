#!/usr/bin/env python3
"""
cyphal communicator
"""
import asyncio
import sys
import pathlib
import rospy
from std_msgs.msg import Bool, Float32
from sensor_msgs.msg import Imu, Joy, MagneticField, NavSatFix


try:
    compiled_dsdl_dir = pathlib.Path(__file__).resolve().parent / "compile_output"
    sys.path.insert(0, str(compiled_dsdl_dir))
    import pyuavcan.application
    import uavcan.node
    from reg.udral.service.actuator.common.sp import Vector4_0_1
    from reg.udral.service.common import Readiness_0_1
except (ImportError, AttributeError):
    sys.exit()
REGISTER_FILE = "allocation_table.db"


class CyphalCommunicator:
    def __init__(self):
        self._node = None
        self._ros_setpoint_pub = rospy.Publisher("/uav/actuators", Joy, queue_size=10)
        self._ros_readiness_pub = rospy.Publisher("/uav/arm", Bool, queue_size=10)
        self._actuators_msg = Joy()
        self._arm_msg = Bool()
        self._sp_sub = None

    async def sp_cb(self, msg, _):
        self._actuators_msg.axes = msg.value
        self._ros_setpoint_pub.publish(self._actuators_msg)

    async def readiness_cb(self, msg, _):
        ENGAGED = 3
        self._arm_msg.data = msg.value == ENGAGED
        self._ros_readiness_pub.publish(self._arm_msg)

    async def main(self):
        rospy.init_node('vehicle_ros', anonymous=True)

        node_info = uavcan.node.GetInfo_1_0.Response(
                    software_version=uavcan.node.Version_1_0(major=1, minor=0),
                    name="vehicle_mock",
                )
        self._node = pyuavcan.application.make_node(node_info, REGISTER_FILE)
        self._node.heartbeat_publisher.mode = uavcan.node.Mode_1_0.OPERATIONAL
        self._node.heartbeat_publisher.vendor_specific_status_code = 50
        self._node.start()

        self._sp_sub = self._node.make_subscriber(Vector4_0_1, "setpoint")
        self._sp_sub.receive_in_background(self.sp_cb)

        self._sp_readiness = self._node.make_subscriber(Readiness_0_1, "readiness")
        self._sp_readiness.receive_in_background(self.readiness_cb)

        while not rospy.is_shutdown():
            rospy.sleep(0.001)
            await asyncio.sleep(0.001)


if __name__ == "__main__":
    communicator = CyphalCommunicator()
    asyncio.run(communicator.main())
