# This is a sample Python script.

# Press Shift+F10 to execute it or replace it with your code.
# Press Double Shift to search everywhere for classes, files, tool windows, actions, and settings.
import cv2
import numpy as np
import scipy.io as io

traj_color_learning = (88, 18, 175)                 # https://www.icoa.cn/a/512.html
traj_color_curr = (0, 255, 0)
traj_color_old  = (225, 169, 36)

def set_color_4_trajectory(is_learning_complete):
    if not is_learning_complete:
        return traj_color_learning                  # BGR
    else:
        return traj_color_curr

def adjust_color_in_traj_array(traj_array, f, decay_t=3):
    if traj_array.__len__() >= 2:
        index_learning_end = -1

        for i in range(1, traj_array.__len__()):
            color_last = traj_array[i - 1][1]
            color_curr = traj_array[i][1]
            if color_curr != color_last:
                index_learning_end = i
                break

        # if the learning process is NOT complete
        if index_learning_end == -1:
            return traj_array

        decay_duration = int(decay_t / (1. / f))
        index_traj_old = index_learning_end
        if decay_duration < traj_array.__len__() - index_learning_end:
            index_traj_old = traj_array.__len__() - decay_duration + 1
        else:
            index_traj_old = index_learning_end + 1

        for i in range(index_learning_end, index_traj_old):
            traj_array[i][1] = traj_color_old

        k_b = (traj_color_curr[0] - traj_color_old[0]) / (traj_array.__len__() - index_traj_old + 1)
        k_g = (traj_color_curr[1] - traj_color_old[1]) / (traj_array.__len__() - index_traj_old + 1)
        k_r = (traj_color_curr[2] - traj_color_old[2]) / (traj_array.__len__() - index_traj_old + 1)
        for i in range(index_traj_old, traj_array.__len__()):
            b = k_b * i + traj_color_old[0]
            g = k_g * i + traj_color_old[1]
            r = k_r * i + traj_color_old[2]
            traj_array[i][1] = (b, g, r)

    return traj_array

def main(name):

    cap = cv2.VideoCapture("./experiment.mp4")  #读取视频文件
    # obtain imformation of the video
    frame_width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    frame_fps    = cap.get(cv2.CAP_PROP_FPS)


    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # MP4格式
    #out = cv2.VideoWriter('./processed.mp4', fourcc, frame_fps, (frame_width, frame_height), True)      # https://blog.csdn.net/mao_hui_fei/article/details/107573021

    cv2.namedWindow("frame", cv2.WINDOW_GUI_NORMAL)     # for display: cv2.WINDOW_GUI_NORMAL, for recording: cv2.WINDOW_AUTOSIZE  https://blog.csdn.net/XYKenny/article/details/90513480

    # parameter of test fields
    field_x = 3.2
    field_y = 3

    o_x_real = field_x / 2 + 0.2
    o_y_real = field_y / 2 + 0.275

    # obtain data from .mat file
    data = io.loadmat('droneyee_20220924T164752.mat')
    t = data['droneyee_timestamp']
    f = 30
    x_real  = data['droneyee2_states'][:, 0]
    y_real  = data['droneyee2_states'][:, 1]
    z_real  = data['droneyee2_states'][:, 2]
    vx_real = data['droneyee2_states'][:, 3]
    vy_real = data['droneyee2_states'][:, 4]
    yaw     = data['droneyee2_states'][:, 5]
    k_x = data['K_x']
    k_y = data['K_y']
    t_video_alignment = 9.05                           # alternative: 9.15

    o_x_pixel = frame_width  / 2
    o_y_pixel = frame_height / 2

    traj_arr = []

    t_video = 0
    j = int(0 - t_video_alignment / (1. / f))
    while True:
        ret, frame = cap.read()
        if ret:

            if j >= 0 and j < t.__len__():
                if abs(t_video - t[j] - t_video_alignment) <=  1. / f * 0.5:
                    # synthesize x-y coordinate in pixels
                    x_pixel = int((x_real[j] + o_x_real) * (frame_width  / field_y))
                    y_pixel = int((y_real[j] + o_y_real) * (frame_height / field_x))

                    # adjust the orientation for the trajectory
                    #x_pixel = int(frame_width  - x_pixel)
                    y_pixel = int(frame_height - y_pixel)


                    color = set_color_4_trajectory(k_x[j][0] != 0. and k_y[j][0] != 0.)
                    traj_arr.append([(y_pixel, x_pixel), color])
                    traj_arr = adjust_color_in_traj_array(traj_arr, f, decay_t=3)

                    traj_channel = np.ones(frame.shape, dtype=frame.dtype) * 0
                    for [pt_t, color_t] in traj_arr:
                        cv2.circle(traj_channel, pt_t, 25, color_t, -1)

                    frame = cv2.addWeighted(frame, 1, traj_channel, 0.85, -1)       # dst = src1 [I]*alpha+ src2[I]*beta + gamma;

                    #print((y_pixel, x_pixel))
                else:
                    if t_video > t[j]:
                        j += 1
                    else:
                        j -= 1

            cv2.imshow("frame", frame)
            #out.write(frame)

            t_video += 1 / frame_fps
            j += 1
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break



        else:
            break

    cap.release()
    cv2.destoryAllWindows()

    # Use a breakpoint in the code line below to debug your script.
    print(f'Hi, {name}')  # Press Ctrl+F8 to toggle the breakpoint.


# Press the green button in the gutter to run the script.
if __name__ == '__main__':
    main('PyCharm')

# See PyCharm help at https://www.jetbrains.com/help/pycharm/
