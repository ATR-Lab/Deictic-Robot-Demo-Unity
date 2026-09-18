# Local compatibility patch

The embedded package is ROS-TCP-Connector 0.7.0-preview with one local ROS 2
compatibility fix: `Runtime/Messages/Std/msg/EmptyMsg.cs` reads and writes the
single dummy byte required by the ROS 2 IDL representation of an empty message.
The change is guarded by `ROS2`; ROS 1 serialization is unchanged.

Without the patch, Unity sends only the four-byte CDR header (`00 01 00 00`).
ROS Jazzy's Fast CDR deserializer rejects that payload. A header followed by one
zero byte is accepted; native rclpy serialization also adds trailing padding,
yielding eight bytes. This was verified against the supplied remote ROS Jazzy
installation. The bug prevented `std_msgs/Empty` Execute and Cancel commands
from being delivered correctly.

`FrameAndIntentTests.EmptyRos2MessageIncludesRequiredCdrDummyByte` checks the
TCP length prefix, outgoing bytes, incoming native padded data, and consumption
of the dummy byte before subsequent fields. Preserve this patch when replacing
or regenerating the embedded message classes until the replacement passes that
regression test and a real ROS 2 execute/cancel round trip.
