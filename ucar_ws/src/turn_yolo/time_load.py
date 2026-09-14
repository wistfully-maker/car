
import time
from rknnlite.api import RKNNLite
M = "/home/ucar/ucar_ws/src/turn_yolo/turn_yolo_480_cls.rknn"

t0=time.time(); rk=RKNNLite(); rk.load_rknn(M); t1=time.time()
rk.init_runtime(core_mask=RKNNLite.NPU_CORE_0_1_2); t2=time.time()
print("load_rknn           : %.3fs" % (t1-t0))
print("init_runtime(3核)   : %.3fs" % (t2-t1))
rk.release()

t0=time.time(); rk=RKNNLite(); rk.load_rknn(M); t1=time.time()
rk.init_runtime(core_mask=RKNNLite.NPU_CORE_0); t2=time.time()
print("load_rknn(再测)     : %.3fs" % (t1-t0))
print("init_runtime(单核)  : %.3fs" % (t2-t1))
rk.release()
