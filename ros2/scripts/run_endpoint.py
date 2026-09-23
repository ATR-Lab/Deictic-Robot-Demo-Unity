#!/usr/bin/env python3
"""Run the pinned ROS2 endpoint with Connector unsubscribe compatibility.

The embedded Unity Connector sends __remove_subscriber when robot video is
hidden. ROS-TCP-Endpoint ROS2v0.7.0 lacks that command; an AttributeError closes
the shared connection, also dropping control/status traffic. Keep the pinned
dependency unchanged. Camera frames use a bounded latest-image queue so a slow
viewer cannot accumulate old video ahead of status and clock replies.
"""
from queue import Queue
import struct


DISPLAY_TOPICS = frozenset((b'/deictic/camera_view/stereo/image_raw',
                           b'/deictic/camera_view/stereo/image_raw/compressed'))


def display_topic(packet):
    """Recognize only complete standalone image packets, never service bundles."""
    if not isinstance(packet, bytes) or len(packet) < 8:
        return None
    length = struct.unpack_from('<I', packet)[0]
    if not 0 < length <= 128 or len(packet) < length + 8:
        return None
    topic = packet[4:4 + length]
    payload_length = struct.unpack_from('<I', packet, 4 + length)[0]
    return topic if topic in DISPLAY_TOPICS and len(packet) == length + 8 + payload_length else None


class _ImageSlot:
    def __init__(self, topic):
        self.topic = topic


class CameraLatestQueue(Queue):
    """FIFO control traffic plus at most one unsent image per display topic.

    Queue's mutex protects _put/_get, including concurrent ROS subscriber
    callbacks. A replacement does not add unfinished work or a second slot.
    Bytes already handed to TCP cannot be withdrawn.
    """
    def __init__(self):
        super().__init__()
        self.images = {}

    def _put(self, packet):
        topic = display_topic(packet)
        if topic is None:
            super()._put(packet)
        elif topic in self.images:
            self.images[topic] = packet
            # Queue.put increments this after _put; replacing retains the
            # existing unit of work, including normal task_done/join semantics.
            self.unfinished_tasks -= 1
        else:
            self.images[topic] = packet
            super()._put(_ImageSlot(topic))

    def _get(self):
        value = super()._get()
        return self.images.pop(value.topic) if isinstance(value, _ImageSlot) else value


def remove_subscriber(self, topic):
    server = self.tcp_server
    if not isinstance(topic, str) or not topic:
        server.send_unity_error('remove_subscriber requires a nonempty topic name')
        return
    subscriber = server.subscribers_table.pop(topic, None)
    if subscriber is not None:
        # Upstream unregister_node destroys the ROS subscription/node and
        # removes it from the executor. Popping also makes duplicate removes
        # harmless and prevents destroy_nodes from revisiting the old node.
        server.unregister_node(subscriber)
        server.loginfo('RemoveSubscriber({}) OK'.format(topic))


def install_compatibility(commands_class):
    """Preserve a native implementation if the dependency gains one later."""
    if hasattr(commands_class, 'remove_subscriber'):
        return False
    commands_class.remove_subscriber = remove_subscriber
    return True


def main(args=None):
    from ros_tcp_endpoint import tcp_sender
    from ros_tcp_endpoint.server import SysCommands
    from ros_tcp_endpoint.default_server_endpoint import main as endpoint_main

    install_compatibility(SysCommands)
    # The pinned sender constructs one Queue per TCP connection. Scope this
    # adapter to that module; other ROS/service queues retain their semantics.
    tcp_sender.Queue = CameraLatestQueue
    endpoint_main(args=args)


if __name__ == '__main__':
    main()
