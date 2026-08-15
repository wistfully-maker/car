#!/usr/bin/env python3
"""TCP server joining the vehicle ROS domain to a remote Gazebo bridge."""

import json
import socket
import threading

import rospy
from std_msgs.msg import String

from task_orchestrator.gazebo_tcp_bridge import (
    BridgeProtocolError,
    GazeboBridgeSession,
    JsonLineDecoder,
    encode_line,
)


class GazeboTcpBridgeServer:
    def __init__(self):
        self.bind_host = rospy.get_param("~bind_host", "0.0.0.0")
        self.port = int(rospy.get_param("~port", 1525))
        self.allowed_client_ip = rospy.get_param("~allowed_client_ip", "")
        self.max_frame_bytes = int(rospy.get_param("~max_frame_bytes", 8192))
        self._lock = threading.RLock()
        self._session = GazeboBridgeSession(self.allowed_client_ip)
        self._server = None
        self._client = None
        self._stopping = False
        self._complete_pub = rospy.Publisher(
            "/task/gazebo/complete", String, queue_size=10, latch=False
        )
        self._start_sub = rospy.Subscriber(
            "/task/gazebo/start", String, self._on_start, queue_size=10
        )
        rospy.on_shutdown(self.shutdown)

    def start(self):
        thread = threading.Thread(target=self._serve, name="gazebo-tcp-server")
        thread.daemon = True
        thread.start()

    def _publish_complete(self, message):
        self._complete_pub.publish(String(
            data=json.dumps(message, ensure_ascii=False, separators=(",", ":"))
        ))

    def _apply_event(self, event):
        if not event:
            return
        complete = event.get("complete")
        if complete is not None:
            self._publish_complete(complete)

    def _on_start(self, ros_message):
        try:
            raw = json.loads(ros_message.data)
            with self._lock:
                event = self._session.start(raw)
                if event and "send" in event:
                    if self._client is None:
                        event = self._session.disconnect()
                    else:
                        try:
                            self._client.sendall(encode_line(event["send"]))
                            event = None
                        except OSError:
                            event = self._disconnect_locked()
            self._apply_event(event)
        except (TypeError, ValueError, BridgeProtocolError) as exc:
            rospy.logwarn("ignored invalid gazebo start: %s", exc)

    def _serve(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.bind_host, self.port))
        server.listen(1)
        server.settimeout(1.0)
        with self._lock:
            self._server = server
        rospy.loginfo("gazebo TCP bridge listening on %s:%d", self.bind_host, self.port)
        while not rospy.is_shutdown() and not self._stopping:
            try:
                client, peer = server.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            with self._lock:
                accepted = self._session.connect(peer[0])
                if accepted:
                    self._client = client
            if not accepted:
                rospy.logwarn("rejected gazebo bridge client %s", peer[0])
                client.close()
                continue
            rospy.loginfo("gazebo bridge client connected: %s", peer[0])
            self._receive_client(client)

    def _receive_client(self, client):
        decoder = JsonLineDecoder(self.max_frame_bytes)
        client.settimeout(1.0)
        try:
            while not rospy.is_shutdown() and not self._stopping:
                try:
                    data = client.recv(4096)
                except socket.timeout:
                    continue
                if not data:
                    break
                for message in decoder.feed(data):
                    with self._lock:
                        event = self._session.receive(message)
                    self._apply_event(event)
        except (OSError, BridgeProtocolError) as exc:
            rospy.logwarn("gazebo bridge client ended: %s", exc)
        finally:
            with self._lock:
                event = self._disconnect_locked()
            self._apply_event(event)

    def _disconnect_locked(self):
        client = self._client
        self._client = None
        if client is not None:
            try:
                client.close()
            except OSError:
                pass
        return self._session.disconnect()

    def shutdown(self):
        with self._lock:
            self._stopping = True
            self._disconnect_locked()
            server = self._server
            self._server = None
            if server is not None:
                try:
                    server.close()
                except OSError:
                    pass


def main():
    rospy.init_node("gazebo_tcp_bridge_server")
    server = GazeboTcpBridgeServer()
    server.start()
    rospy.spin()


if __name__ == "__main__":
    main()
