
import time, logging
import numpy as np
import rospy
from sensor_msgs.msg import Image
logging._nameToLevel.update({"CRITICAL":50,"FATAL":50,"ERROR":40,"WARN":30,"WARNING":30,"INFO":20,"DEBUG":10,"NOTSET":0})
ts=[]
def cb(m):
    ts.append(time.time())
rospy.init_node("framerate", anonymous=True)
rospy.Subscriber("/usb_cam/image_raw", Image, cb, queue_size=1)
rospy.sleep(6)
ts=ts[1:]
if len(ts)<2:
    print("没收到帧（摄像头/roscore 可能没起）")
else:
    d=np.diff(ts)
    print("帧数=%d  平均间隔=%.1fms  中位=%.1fms  min=%.1fms  max=%.1fms  平均=%.1f FPS"%(len(ts), d.mean()*1000, np.median(d)*1000, d.min()*1000, d.max()*1000, 1.0/d.mean()))
