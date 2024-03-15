import cv2 as cv
import time
import numpy as np

# Ref
# 1 https://zhuanlan.zhihu.com/p/71840019
# 2 https://blog.csdn.net/great_yzl/article/details/119645423
# 3 https://blog.csdn.net/drippingstone/article/details/116081434
# 4 https://blog.csdn.net/lly1122334/article/details/113358019
# 最后发现减帧法效率最高也最好调


#
# 'C:\\Users\\lan48\\Videos\\IPC_113_20240303163024.mp4'
# 'C:\\Users\\lan48\\Videos\\Drone\\20240313\\DJI_20240313233617_0078_D.MP4'
# 'D:\\User\\Videos\\2024.3.Drone\\Trim\\DJI_20240313233617_0078_D - Trim.MP4'
#
video_name = 'D:\\User\\Videos\\2024.3.Drone\\Trim\\DJI_20240315170326_0082_D.MP4'
trace_out_name = 'trace_out'
video_out_name = 'video_out'
t_takeoff  = 5                  # 给一个大概的起飞时间即可
#
trace_length = 300              # unit: frame
#
is_debug = True
debug_size_wh = (1440, 1080)

def calc_sobel(frame):
    x = cv.Sobel(frame, cv.CV_16S, 1, 0)
    y = cv.Sobel(frame, cv.CV_16S, 0, 1)

    # 转换数据 并 合成
    Scale_absX = cv.convertScaleAbs(x)  # 格式转换函数
    Scale_absY = cv.convertScaleAbs(y)
    result = cv.addWeighted(Scale_absX, 0.5, Scale_absY, 0.5, 0)  # 图像混合

    return result

def sliding_window_storage(img_t, img_buffer):
    for i in range(0, img_buffer.__len__() - 1):
        img_buffer[i + 1] = img_buffer[i]
    img_buffer[0] = img_t.copy()

    return img_buffer

def main():
    global video_name, t_takeoff, trace_length
    global is_debug, debug_size_wh

    cap = cv.VideoCapture(video_name)

    # 读取第一帧
    ret, frame = cap.read()
    if is_debug:
        frame = cv.resize(frame, debug_size_wh)
    frame_last_bf = list()                                 # 从左到右, 新到旧, 新数据要从左边进

    cv.bilateralFilter(frame, 9, 75, 75)        # cv.GaussianBlur(frame,(5,5),0)  cv.medianBlur(frame, 5)        cv.bilateralFilter(frame, 9, 75, 75)
    for i in range(0, trace_length):
        frame_last_bf.append(frame.copy())

    # 初始化
    frame_sobel      = frame.copy()
    frame_sobel_last = frame.copy()

    #
    t_index = 0

    #
    fourcc = cv.VideoWriter_fourcc('m', 'p', '4', 'v')  # 视频编解码器
    fps = cap.get(cv.CAP_PROP_FPS)  # 帧数
    (height_t, width_t, channel) = frame.shape  # 宽高
    #
    t_str = str(time.strftime('%Y-%m-%d_%H_%M_%S', time.localtime()))
    #
    trace_out_name_full = trace_out_name + t_str + '.avi'
    video_out_name_full = video_out_name + t_str + '.avi'
    trace_out_mp4 = cv.VideoWriter(trace_out_name_full, fourcc, fps, (width_t, height_t))  # 写入视频
    video_out_mp4 = cv.VideoWriter(video_out_name_full, fourcc, fps, (width_t, height_t))
    print("output video width_t / height: %d / %d" % (width_t, height_t))
    print("trace out is saved to: " + trace_out_name_full)
    print("video out is saved to: " + video_out_name_full)

    # TODO
    # 只对视频图像的一个部分做处理而非整个图像

    while True:
        # 读取视频流
        grabbed, frame = cap.read()
        if grabbed != True:
            break
        if is_debug:
            frame = cv.resize(frame, debug_size_wh)

        # step 1 滤波
        frame = cv.bilateralFilter(frame, 9, 75, 75)

        # step 2 计算sobel算子
        frame_sobel_last = frame_sobel
        frame_sobel = calc_sobel(frame)

        # step 3 sobel算子结果求差值获得运动物体
        diff_sobel = cv.absdiff(frame_sobel, frame_sobel_last)

        # step 4 滑动存储运算结果
        sliding_window_storage(diff_sobel, frame_last_bf)

        # step 5 获得结果
        #        过滤初始帧
        if t_index <= trace_length:
            t_index += 1
            print("waiting for initial data, index: %d" % (t_index))
            continue
        #        求尾迹
        trace_out = diff_sobel.copy()
        for i in range((int)(trace_length * 0.2), trace_length):
            trace_out = cv.addWeighted(trace_out, 0.75, frame_last_bf[i], 0.75, 0)

        frame_out = cv.addWeighted(frame, 0.75, trace_out, 0.5, 0)

        #
        cv.imshow("trace", trace_out)
        cv.imshow('Sobel', frame_sobel)
        cv.imshow('Out',   frame_out)

        trace_out_mp4.write(trace_out)
        video_out_mp4.write(frame_out)

        key = cv.waitKey(1) & 0xFF
        # 按'q'健退出循环
        if key == ord('q'):
            break

        t_index += 1

    # When everything done, release the capture
    cap.release()
    trace_out_mp4.release()
    video_out_mp4.release()
    print("Video Saved ...")
    cv.destroyAllWindows()



if __name__ == '__main__':
    main()
