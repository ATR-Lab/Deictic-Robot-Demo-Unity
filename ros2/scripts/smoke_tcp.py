#!/usr/bin/env python3
"""Exercise the real Connector 0.7 ROS2 TCP contract, with optional sim execution.

Uses ROS serialization to emulate a Unity client. It does not require a ROS
daemon or DDS on the client. Run in a sourced ROS Python environment.
"""
import argparse
import json
from pathlib import Path
import socket
import struct
import threading
import time
import numpy as np
from rclpy.serialization import serialize_message, deserialize_message
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Empty, Float64, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory
from deictic_control.kinematics import ArmModel


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=10000)
    parser.add_argument('--execute', action='store_true', help='Execute one reaching trajectory in simulation')
    parser.add_argument('--timeout', type=float, default=30.)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    model = ArmModel(root/'models/K1/K1_22dof.urdf')
    sock = socket.create_connection((args.host,args.port), timeout=5)
    sock.settimeout(None)
    incoming, errors, stop = {}, [], threading.Event()
    types = {'/deictic/status': String, '/deictic/preview': JointTrajectory,
             '/joint_states': JointState, '/k1/end_effector_pose': PoseStamped,
             '/deictic/time_sync/reply': Float64MultiArray}
    clock_samples = []

    def send(topic, payload):
        name = topic.encode()
        sock.sendall(struct.pack('<I',len(name))+name+struct.pack('<I',len(payload))+payload)

    def read_exact(n):
        data = bytearray()
        while len(data)<n:
            chunk = sock.recv(n-len(data))
            if not chunk:
                raise EOFError('Endpoint disconnected')
            data.extend(chunk)
        return bytes(data)

    def read():
        try:
            while not stop.is_set():
                name = read_exact(struct.unpack('<I',read_exact(4))[0]).decode()
                payload = read_exact(struct.unpack('<I',read_exact(4))[0])
                if name in types:
                    incoming[name] = deserialize_message(payload, types[name])
                    if name == '/deictic/time_sync/reply':
                        t3 = time.time()
                        t0,t1,t2 = incoming[name].data
                        rtt = (t3-t0)-(t2-t1)
                        if 0 <= rtt <= .25:
                            clock_samples.append((rtt,((t1-t0)+(t2-t3))/2))
                elif name == '__handshake':
                    incoming[name] = json.loads(payload.rstrip(b'\0'))
                elif name == '__error':
                    errors.append(payload.decode())
        except (OSError, EOFError) as error:
            if not stop.is_set():
                errors.append(str(error))

    def wait_until(predicate, description, timeout=None):
        deadline = time.monotonic()+(args.timeout if timeout is None else timeout)
        while time.monotonic()<deadline:
            if errors:
                raise RuntimeError(errors)
            if predicate():
                return
            time.sleep(.02)
        raise TimeoutError(description+'; last status='+str(status()))

    def status():
        message = incoming.get('/deictic/status')
        return {} if message is None else json.loads(message.data)

    def current_position():
        message = incoming.get('/joint_states')
        if message is None:
            return None
        lookup = dict(zip(message.name,message.position))
        if not all(name in lookup for name in model.names):
            return None
        return model.fk([lookup[name] for name in model.names])[:3,3]

    reader = threading.Thread(target=read,daemon=True)
    reader.start()
    try:
        for topic, message_name in [('/deictic/status','std_msgs/String'),('/deictic/preview','trajectory_msgs/JointTrajectory'),
                                    ('/joint_states','sensor_msgs/JointState'),('/k1/end_effector_pose','geometry_msgs/PoseStamped'),
                                    ('/deictic/time_sync/reply','std_msgs/Float64MultiArray')]:
            send('__subscribe',json.dumps(dict(topic=topic,message_name=message_name)).encode()+b'\0')
        for topic, message_name in [('/deictic/goal','geometry_msgs/PoseStamped'),('/deictic/execute_request','std_msgs/String'),('/deictic/cancel','std_msgs/Empty'),
                                    ('/deictic/time_sync/request','std_msgs/Float64')]:
            send('__publish',json.dumps(dict(topic=topic,message_name=message_name)).encode()+b'\0')
        time.sleep(.5)
        for _ in range(5):
            send('/deictic/time_sync/request',serialize_message(Float64(data=time.time())))
            time.sleep(.2)
        wait_until(lambda:bool(clock_samples),'Clock synchronization')
        clock_rtt,clock_offset=min(clock_samples)
        wait_until(lambda: status().get('can_commit') and current_position() is not None,'Waiting for aligned robot feedback')
        initial = current_position().copy()
        # The stale-command gate is part of the external protocol contract.
        stale = PoseStamped()
        stale.header.frame_id='base_link'
        stale.pose.orientation.w=1.
        send('/deictic/goal',serialize_message(stale))
        wait_until(lambda:'goal_timestamp_out_of_range' in status().get('operation',''),'Stale goal rejection')
        goal = PoseStamped()
        now = int((time.time()+clock_offset)*1e9)
        goal.header.stamp.sec,goal.header.stamp.nanosec=divmod(now,1_000_000_000)
        goal.header.frame_id='base_link'
        goal.pose.position.x,goal.pose.position.y,goal.pose.position.z=.1,-.25,.02
        goal.pose.orientation.w=1.
        send('/deictic/goal',serialize_message(goal))
        wait_until(lambda:'/deictic/preview' in incoming and status().get('preview_ready'),'Validated preview')
        preview=incoming['/deictic/preview']
        assert list(preview.joint_names)==model.names
        assert len(preview.points)>2
        assert preview.header.stamp == goal.header.stamp, 'Preview must echo the exact goal timestamp'
        assert status()['mode']=='synthetic_test', 'This smoke script only executes synthetic simulation mode'
        target=np.array([.1,-.25,.02])
        result=dict(handshake=incoming.get('__handshake'),backend=status()['backend'],mode=status()['mode'],
                    stale_goal_rejected=True,preview_points=len(preview.points),initial_position=initial.tolist(),executed=False,
                    server_clock_offset_seconds=clock_offset,time_sync_rtt_seconds=clock_rtt)
        if args.execute:
            send('/deictic/execute_request',serialize_message(String(data=json.dumps(dict(
                schema_version=1, goal_stamp_sec=preview.header.stamp.sec,
                goal_stamp_nanosec=preview.header.stamp.nanosec)))))
            wait_until(lambda:status().get('operation')=='trajectory_sent_simulation','Execution dispatch')
            wait_until(lambda:current_position() is not None and np.linalg.norm(current_position()-target)<.008,'Measured reaching completion')
            wait_until(lambda:status().get('operation')=='execution_succeeded','Feedback-confirmed execution result')
            result.update(executed=True,final_position=current_position().tolist(),position_error_m=float(np.linalg.norm(current_position()-target)))
            send('/deictic/cancel',serialize_message(Empty()))
            wait_until(lambda:status().get('operation')=='cancel_hold_sent','Cancellation hold')
            result['cancel_hold_sent']=True
        print(json.dumps(result,indent=2))
    finally:
        stop.set()
        sock.shutdown(socket.SHUT_RDWR)
        sock.close()
        reader.join(timeout=1)


if __name__=='__main__':
    main()
