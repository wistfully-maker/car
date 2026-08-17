#!/usr/bin/env python3
import socket
import rospy
from geometry_msgs.msg import Twist

LISTEN_PORT = 8080
BASE_SPEED = 0.09
STEER_SCALE = 0.03

rospy.init_node("tcp_receive_node")
cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=5)

def tcp_server():
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(("0.0.0.0", LISTEN_PORT))
    server_socket.listen(1)
    print(f"TCP服务启动，端口 {LISTEN_PORT}")

    while not rospy.is_shutdown():
        try:
            client, address = server_socket.accept()
            print(f"PC接入 {address}")
            buffer = ""
            while True:
                data = client.recv(256)
                if not data:
                    break
                buffer += data.decode()
                while "\n" in buffer:
                    line, buffer = buffer.split("\n", 1)
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        err_part, steer_part = line.split(" ")
                        steer_val = float(steer_part.split(":")[1])
                        twist = Twist()
                        twist.linear.x = BASE_SPEED
                        twist.angular.z = steer_val * STEER_SCALE
                        cmd_pub.publish(twist)
                    except Exception:
                        continue
        except Exception as e:
            pass
        finally:
            try:
                client.close()
            except:
                pass

if __name__ == "__main__":
    try:
        tcp_server()
    except rospy.ROSInterruptException:
        pass

