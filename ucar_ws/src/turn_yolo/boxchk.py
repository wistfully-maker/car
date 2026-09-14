import sys, logging
import numpy as np, cv2, rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rknnlite.api import RKNNLite
logging._nameToLevel.update({"CRITICAL":50,"FATAL":50,"ERROR":40,"WARN":30,"WARNING":30,"INFO":20,"DEBUG":10,"NOTSET":0})
rknn=RKNNLite(); rknn.load_rknn("/home/ucar/ucar_ws/src/turn_yolo/turn_yolo_480.rknn"); rknn.init_runtime(core_mask=RKNNLite.NPU_CORE_0_1_2)
br=CvBridge(); f=[None]
def cb(m):
    try: f[0]=br.imgmsg_to_cv2(m,"bgr8")
    except: pass
rospy.init_node("boxchk",anonymous=True)
rospy.Subscriber("/usb_cam/image_raw",Image,cb,queue_size=1)
rospy.sleep(2)
img=f[0]
def lb(im,new=(480,480),color=(114,114,114)):
    sh=im.shape[:2]; r=min(new[0]/sh[0],new[1]/sh[1]); nu=int(round(sh[1]*r)),int(round(sh[0]*r))
    dw,dh=(new[1]-nu[0])/2,(new[0]-nu[1])/2
    if sh[::-1]!=nu: im=cv2.resize(im,nu,interpolation=cv2.INTER_LINEAR)
    return cv2.copyMakeBorder(im,int(round(dh-0.1)),int(round(dh+0.1)),int(round(dw-0.1)),int(round(dw+0.1)),cv2.BORDER_CONSTANT,value=color)
x=cv2.cvtColor(lb(img),cv2.COLOR_BGR2RGB)[None]
out=rknn.inference(inputs=[x],data_type="uint8",data_format="nhwc")[0]
preds=out[0].T
print("box 通道(前4) 范围: min=%.3f max=%.3f"%(preds[:,:4].min(),preds[:,:4].max()))
print("class 通道(后4) 范围: min=%.3f max=%.3f"%(preds[:,4:].min(),preds[:,4:].max()))
print("全部8通道 范围: min=%.3f max=%.3f"%(preds.min(),preds.max()))
for i,nm in enumerate(["cx","cy","w","h"]):
    print("  %s: min=%.3f max=%.3f"%(nm,preds[:,i].min(),preds[:,i].max()))
