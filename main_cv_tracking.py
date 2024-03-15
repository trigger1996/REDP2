import cv2
import functools
import scipy.io as io
import numpy as np

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

def sort_contours(x, y):
    if cv2.contourArea(x) >= cv2.contourArea(y):
        return 1
    else:
        return -1

if __name__ == '__main__':
    OPENCV_MAJOR_VERSION = int(cv2.__version__.split('.')[0])

    bg_subtractor = cv2.createBackgroundSubtractorMOG2(detectShadows=True)

    erode_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    dilate_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))

    cap = cv2.VideoCapture("./experiment.mp4")
    success, frame = cap.read()

    cv2.namedWindow("mog",       cv2.WINDOW_AUTOSIZE)     # for display: cv2.WINDOW_GUI_NORMAL, for recording: cv2.WINDOW_AUTOSIZE
    cv2.namedWindow("thresh",    cv2.WINDOW_AUTOSIZE)     # for display: cv2.WINDOW_GUI_NORMAL, for recording: cv2.WINDOW_AUTOSIZE
    cv2.namedWindow("detection", cv2.WINDOW_AUTOSIZE)     # for display: cv2.WINDOW_GUI_NORMAL, for recording: cv2.WINDOW_AUTOSIZE

    # obtain imformation of the video
    frame_width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    frame_fps    = cap.get(cv2.CAP_PROP_FPS)

    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # MP4格式
    out = cv2.VideoWriter('./processed.mp4', fourcc, frame_fps, (frame_width, frame_height), True)      # https://blog.csdn.net/mao_hui_fei/article/details/107573021

    # parameter of test fields
    field_x = 3.2
    field_y = 3

    o_x_real = field_x / 2 + 0.2
    o_y_real = field_y / 2 + 0.295

    # obtain data from .mat file
    data = io.loadmat('droneyee_20220924T164752.mat')
    t = data['droneyee_timestamp']
    f = 30
    x_real = data['droneyee2_states'][:, 0]
    y_real = data['droneyee2_states'][:, 1]
    z_real = data['droneyee2_states'][:, 2]
    vx_real = data['droneyee2_states'][:, 3]
    vy_real = data['droneyee2_states'][:, 4]
    yaw = data['droneyee2_states'][:, 5]
    k_x = data['K_x']
    k_y = data['K_y']
    t_video_alignment = 9.05  # alternative: 9.15

    o_x_pixel = frame_width / 2
    o_y_pixel = frame_height / 2

    traj_arr = []

    t_video = 0
    j = int(0 - t_video_alignment / (1. / f))

    while success:

        fg_mask = bg_subtractor.apply(frame)

        _, thresh = cv2.threshold(fg_mask, 244, 255, cv2.THRESH_BINARY)
        cv2.erode(thresh, erode_kernel, thresh, iterations=2)
        cv2.dilate(thresh, dilate_kernel, thresh, iterations=2)

        if OPENCV_MAJOR_VERSION >= 4:
            # OpenCV 4 or a later version is being used.
            contours, hier = cv2.findContours(thresh, cv2.RETR_EXTERNAL,
                                              cv2.CHAIN_APPROX_SIMPLE)
        else:
            # OpenCV 3 or an earlier version is being used.
            # cv2.findContours has an extra return value.
            # The extra return value is the thresholded image, which is
            # unchanged, so we can ignore it.
            _, contours, hier = cv2.findContours(thresh, cv2.RETR_EXTERNAL,
                                                 cv2.CHAIN_APPROX_SIMPLE)

        if j >= 0 and j < t.__len__():
            if abs(t_video - t[j] - t_video_alignment) <=  1. / f * 0.5:
                # synthesize x-y coordinate in pixels
                x_pixel = int((x_real[j] + o_x_real) * (frame_width  / field_y))
                y_pixel = int((y_real[j] + o_y_real) * (frame_height / field_x))

                # adjust the orientation for the trajectory
                #x_pixel = int(frame_width  - x_pixel)
                y_pixel = int(frame_height - y_pixel)

                contours_to_display = []
                for c in contours:
                    if cv2.contourArea(c) > 5000:  # 1000
                        contours_to_display.append(c)

                if contours_to_display.__len__():
                    contours_to_display.sort(key=functools.cmp_to_key(sort_contours))

                    c = contours_to_display[0]
                    x, y, w, h = cv2.boundingRect(c)
                    # cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 255, 0), 2)
                    pt_cv = (int(x + w / 2), int(y + h / 2))

                    if t_video < 20.5:
                        alpha = 0.775
                    else:
                        alpha = 0.15
                    pt_f = [0., 0.]
                    y_pixel = int(y_pixel * alpha + pt_cv[0] * (1. - alpha))
                    x_pixel = int(x_pixel * alpha + pt_cv[1] * (1. - alpha))


                color = set_color_4_trajectory(k_x[j][0] != 0. and k_y[j][0] != 0.)
                traj_arr.append([(y_pixel, x_pixel), color])
                traj_arr = adjust_color_in_traj_array(traj_arr, f, decay_t=3)

                traj_channel = np.ones(frame.shape, dtype=frame.dtype) * 0
                for k in range(1, traj_arr.__len__()):
                    pt_t    = traj_arr[k][0]
                    pt_last = traj_arr[k - 1][0]
                    color_t = traj_arr[k][1]
                    cv2.line(traj_channel, pt_last, pt_t, color_t, 25)
                    #cv2.circle(traj_channel, pt_t, 25, color_t, -1)

                frame = cv2.addWeighted(frame, 1, traj_channel, 0.85, -1)       # dst = src1 [I]*alpha+ src2[I]*beta + gamma;

                #print((y_pixel, x_pixel))
            else:
                if t_video > t[j]:
                    j += 1
                else:
                    j -= 1



        cv2.imshow('mog', fg_mask)
        cv2.imshow('thresh', thresh)
        cv2.imshow('detection', frame)
        out.write(frame)

        k = cv2.waitKey(30)
        if k == 27:  # Escape
            break

        success, frame = cap.read()
        t_video += 1 / frame_fps
        j += 1

        print(f'T: {t_video}')

    cap.release()
    #cv2.destoryAllWindows()
