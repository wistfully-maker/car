import cv2
import numpy as np
import time
# PID参数及初始变量
Kp = 0.015
Ki = 0.000025
Kd = 0.0000000
sum_pid = 0
d_pid = 0
latter_pid = 0
flag = 0

# 定义历史数据长度，用于平滑
history_len = 5
mid_history = []
high_pixel=400
low_pixel=300
#####显示二值化区域########
biaoding_line=200
biaoding_line_2=440
show_high=400
show_low=300
image_path = r"F:\ucarws\ucar_ws\src\ucar_camera\line\108.jpg"
#取中点的行数
deal_low=320   #360
deal_high=400  #380
stop_line_low=400
stop_line_high=470
camera_matrix = np.array([
    [417.02331176, 0, 317.6164318],
    [0, 417.36101837, 222.31700641],
    [0, 0, 1]
], dtype=np.float32)
lines_num=0
dist_coeffs = np.array([-0.3183328, 0.09406683, 0.00304064, -0.00085934, 0], dtype=np.float32)
def check_is_black(image, check_low,check_high):
    """
    检测图像中指定行数内是否存在白线。
    
    参数：
    image: 输入图像
    check_low：行数1，偏小的行
    check_high:行数2，偏大的行
    比如，check_low=300,check_high=330
    返回：
    如果指定存在白线，返回1；
    如果没有检测到白线，返回0。
    """

    
    # 从指定行数以下开始扫描图像
    region = image[check_low:check_high, :]  # 获取从指定行到图像底部的区域
    
    # 判断该区域是否包含白线（即是否存在非零像素）
    if np.any(region == 255):  # 如果有非零像素（白线）
        return 1  # 返回白线所在的区域
    else:
        return 0  # 没有白线，返回None


def check_white_lines(image, check_low, check_high):
    """
    检测图像中指定行数之间各列是否存在白线，并统计有白线的列数。
    
    参数：
    image: 输入的二值化图像（假设白线为255，背景为0）
    check_low：起始行（包含）
    check_high: 结束行（不包含）
    
    返回：
    存在白线的列的数量。
    """
    # 提取指定行之间的区域
    region = image[check_low:check_high, :]
    
    # 检查每列是否存在至少一个白线（255）
    # 沿垂直方向（行方向）检查，若某列存在至少一个白线则标记为True
    white_cols = np.any(region == 255, axis=0)
    # 统计有白线的列数
    return np.sum(white_cols)

# 用来控制打印频率的时间
last_print_time = time.time()
def detect_and_binarize_white_line(image):
    global stop_line_high,stop_line_low,lines_num
    # 调整图像大小和翻转
    image = cv2.resize(image, (640, 480))
    # # 图像增强
    # image = cv2.convertScaleAbs(image, alpha=1, beta=80)
    # cv2.imshow("lightend image", image)
    
    # 应用高斯滤波进行平滑处理
    blurred = cv2.GaussianBlur(image, (5, 5), 0)

    # 转换为HSV颜色空间
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # 定义白色的HSV范围
    lower_white = np.array([0, 21, 203])#50
    upper_white = np.array([180, 63, 255])



    # 创建白色掩码
    mask_white = cv2.inRange(hsv, lower_white, upper_white)

    # # 创建绿色掩码
    # mask_green = cv2.inRange(hsv, lower_green, upper_green)

    # # 移除绿色区域
    # mask_result = cv2.bitwise_and(mask_white, cv2.bitwise_not(mask_green))

    # 将白色部分变为白色，二值化处理
    binary_result = cv2.threshold(mask_white, 1, 255, cv2.THRESH_BINARY)[1]
    lines_num=check_white_lines(binary_result,stop_line_low,stop_line_high)
    print(f"---------白线有{lines_num}列-----------")
    # cv2.imshow("er zhi hua ", binary_result)
    return binary_result


def mid(follow, mask):
    global high_pixel_mid,low_pixel_mid,deal_low,deal_high

    halfWidth = follow.shape[1] // 2
    half = halfWidth  # 从下往上扫描赛道,最下端取图片中线为分割线
    mid_count=0 #最后输出的中点值
    range_scan=deal_high-deal_low
    for y in range(deal_high, deal_low, -1):
        if (mask[y][max(0, half - halfWidth):half] == np.zeros_like(mask[y][max(0, half - halfWidth):half])).all():
            left = max(0, half - halfWidth)  # 取图片左边界
        else:
            left = np.average(np.where(mask[y][0:half] == 255))  # 计算左端平均位置
        if (mask[y][half:min(follow.shape[1], half + halfWidth)] == np.zeros_like(mask[y][half:min(follow.shape[1], half + halfWidth)])).all():
            right = min(follow.shape[1], half + halfWidth)  # 取图片右边界
        else:
            right = np.average(np.where(mask[y][half:follow.shape[1]] == 255)) + half  # 计算右端平均位置

        mid = (left + right) // 2  # 计算拟合中点
        mid = int(mid)
        half = mid  # 递归,从下往上确定分割线
        follow[y, mid] = 255  # 画出拟合中线
        mid_count=mid_count+mid#中点叠加
        # if y==30:
        #     mid1=mid
    mid_count=mid_count/range_scan #扫描范围内的中点均值，除数为y循环的范围
    print(f"中点为{mid_count}")
    return mid_count
def check_white_lines(image, check_low, check_high):
    """
    检测图像中指定行数之间各列是否存在白线，并统计有白线的列数。
    
    参数：
    image: 输入的二值化图像（假设白线为255，背景为0）
    check_low：起始行（包含）
    check_high: 结束行（不包含）
    
    返回：
    存在白线的列的数量。
    """
    # 提取指定行之间的区域
    region = image[check_low:check_high, :]
    
    # 检查每列是否存在至少一个白线（255）
    # 沿垂直方向（行方向）检查，若某列存在至少一个白线则标记为True
    white_cols = np.any(region == 255, axis=0)
    # 统计有白线的列数
    return np.sum(white_cols)
def test_line(lines,w):
    if lines is not None:
        # fn=0
        # n=0
        T=0
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if x1!=x2:
                tan=(y1-y2)/(x1-x2)
                if abs(tan)<0.01 and abs(x1-x2)>0.2*w:
                    T+=1
            else:
                tan=1                                       #通过检测直线斜率检测是否遇到横线
        return T
    else:
        # delta=0
        T = 0 
        return  T
def xunxian(image):
    global Kp, Ki, Kd, latter_pid, d_pid, sum_pid, mid_history,last_print_time

    mask = detect_and_binarize_white_line(image)
    # result=check_is_black(mask,stop_line_low,stop_line_high)
    # if result==0:
    #     print("--------------停车---------------------")
    follow = mask.copy()
    midoutput = mid(follow, mask)
    h, w = 480, 640

    cx = midoutput  # 中点
    # print(f"调整过后的中点坐标为{cx}")
    # print(f"中点为:{cx}")
    search_top = int(3.5*h/5)
    search_bot = int(search_top + 20)
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    undistorted_frame = cv2.undistort(
        mask, 
        camera_matrix,
        dist_coeffs
    )

    # 再进行边缘检测和霍夫变换
    # gray = cv2.cvtColor(undistorted_frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(undistorted_frame, 50, 150)
    lines = cv2.HoughLinesP(edges, 0.5, np.pi/180, 100, minLineLength=100, maxLineGap=60)
    t=test_line(lines,w)                                            
    if t>=1:
        # if  stop_flag==True:
        print ("检测到终点直线")
    vtherror = w/2 - cx  # 偏差
    # print('-----------------vtherror:', vtherror)

    # if abs(vtherror) >= 100:
    #     if vtherror < 0:
    #         vtherror = -100
    #     else:
    #         vtherror = 100

    # vtherror = vtherror / 1.8

    # 连续转弯，过转弯——————限制
    if abs(vtherror) >= 125:
        if vtherror < 0:
            vtherror = -125
        else:
            vtherror = 125
    # vtherror = vtherror / 1.8
    # 获取当前时间
    current_time = time.time()

    # 每5秒打印一次中点
    if current_time - last_print_time >= 5:
        print('-----------------vtherror:', vtherror)
        last_print_time = current_time  # 更新最后一次打印的时间

#-------------------

    sum_pid = sum_pid + float(vtherror)
    d_pid = float(vtherror) - latter_pid
    angular_z = float(vtherror) * Kp + sum_pid * Ki + d_pid * Kd
    latter_pid = float(vtherror)
    # print(f"z的值为:{angular_z}, 偏差为{vtherror}")
    # if detect_white_line_below_row(mask,360)==0:
    #     print("未检测到白线,停车")
    return cx,mask

#-----------------
def detect_white_line_below_row(image, check_row):
    """
    检测图像中指定行数以下是否存在白线。
    
    参数：
    image: 输入图像
    check_row: 指定的行数，函数将从该行以下进行白线检测。
    
    返回：
    如果在指定行以下找到了白线，返回检测到的白线区域；
    如果没有检测到白线，返回None。
    """

    
    # 从指定行数以下开始扫描图像
    region = image[check_row:, :]  # 获取从指定行到图像底部的区域
    
    # 判断该区域是否包含白线（即是否存在非零像素）
    if np.any(region == 255):  # 如果有非零像素（白线）
        return 1  # 返回白线所在的区域
    else:
        return 0  # 没有白线，返回None





def process_frame(frame):
    global last_print_time,deal_high,deal_low,biaoding_line,biaoding_line_2

    cx, mask = xunxian(frame)  # 调用xunxian函数
    
    # 获取当前时间
    current_time = time.time()

    # 每5秒打印一次中点
    if current_time - last_print_time >= 5:
        print(f"中点为:{cx}")
        last_print_time = current_time  # 更新最后一次打印的时间

    frame = cv2.resize(frame, (640, 480))
    # result=check_is_black(mask,stop_line_low,stop_line_high)
    # lines_num=check_white_lines(mask,stop_line_low,stop_line_high)
    # print(f"---------白线有{lines_num}列-----------")
    # if result==0:
    #     print("--------------白线数目多，停车---------------------")

    # 绘制中线
    # cv2.line(frame, (int(cx), 0), (int(cx), frame.shape[0]), (0, 0, 255), 2)  # 在原图像上绘制红色中线
    # cv2.line(frame, (0, deal_low), (frame.shape[1], deal_low), (0, 0, 255), 2)  # 在原图像上绘制红色中线
    cv2.line(frame, (0, biaoding_line), (frame.shape[1], biaoding_line), (0, 0, 255), 2)  # 在原图像上绘制红色中线
    cv2.line(frame, (0, biaoding_line_2), (frame.shape[1], biaoding_line_2), (0, 0, 255), 2)  # 在原图像上绘制红色中线
    # cv2.line(frame, (0, stop_line_high), (frame.shape[1], stop_line_high), (255, 0, 0), 4)  # 在原图像上绘制绿色中线

    # 显示二值化处理后的图像
    cv2.imshow("Binarized Image", mask)

    # 显示二值化后的图像的指定区域（height在350到488之间的部分）
    mask_region = mask[show_low:show_high, :]
    # cv2.imshow("Region of Interest", mask_region)

    # 在原图像上绘制中线
    cv2.imshow("Original Image with Mid Line", frame)



def main():
    # 初始化摄像头（0表示默认摄像头）
    cap = cv2.VideoCapture(0)
    
    # 检查摄像头是否成功打开
    if not cap.isOpened():
        print("错误：无法打开摄像头")
        return

    try:
        while True:
            # 读取摄像头帧
            ret, frame = cap.read()
            
            # 检查是否成功读取帧
            if not ret:
                print("错误：无法获取摄像头画面")
                break
            frame=cv2.resize(frame,(640,480))
            frame = cv2.flip(frame,1)  #镜像一下
            # 处理帧
            process_frame(frame)

            # 按q键退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        # 释放资源
        cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
