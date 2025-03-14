########################################################################
#
# Copyright (c) 2022, STEREOLABS.
#
# All rights reserved.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
########################################################################

"""
    Read a stream and display the left images using OpenCV
"""
import sys

import numpy as np
import pyzed.sl as sl
import cv2
import argparse
import socket
import time
from threading import Thread
from threading import Event
import os

exit_event = Event()  # 全局退出标志
camera_settings = sl.VIDEO_SETTINGS.BRIGHTNESS
str_camera_settings = "BRIGHTNESS"
step_camera_settings = 1
led_on = True
selection_rect = sl.Rect()
select_in_progress = False
origin_rect = (-1, -1)


def on_mouse(event, x, y, flags, param):
    global select_in_progress, selection_rect, origin_rect
    if event == cv2.EVENT_LBUTTONDOWN:
        origin_rect = (x, y)
        select_in_progress = True
    elif event == cv2.EVENT_LBUTTONUP:
        select_in_progress = False
    elif event == cv2.EVENT_RBUTTONDOWN:
        select_in_progress = False
        selection_rect = sl.Rect(0, 0, 0, 0)

    if select_in_progress:
        selection_rect.x = min(x, origin_rect[0])
        selection_rect.y = min(y, origin_rect[1])
        selection_rect.width = abs(x - origin_rect[0]) + 1
        selection_rect.height = abs(y - origin_rect[1]) + 1


def main(opt):
    print("hello world")
    print(opt.ip_address.split(':')[0])
    print(opt.ip_address.split(':')[1])
    ip = opt.ip_address.split(':')[0]
    port = int(opt.ip_address.split(':')[1])
    record_cam(opt.ip_address.split(':')[0], int(opt.ip_address.split(':')[1]))

    # threads = []
    # for i in range(opt.cam_num):
    #     thread = Thread(target=record_cam,
    #                     args=(opt.ip_address.split(':')[0], int(opt.ip_address.split(':')[1]) + i * 2))
    #     thread.start()
    #     threads.append(thread)
    #     print(i)
    # for t in threads:
    #     t.join()
    #     print("线程挂起")
    # while not exit_event.is_set():
    #     if cv2.waitKey(1) == 113:  # 主窗口监听Q键
    #         print("开始退出")
    #         exit_event.set()


def print_camera_information(cam):
    cam_info = cam.get_camera_information()
    print("ZED Model                 : {0}".format(cam_info.camera_model))
    print("ZED Serial Number         : {0}".format(cam_info.serial_number))
    print("ZED Camera Firmware       : {0}/{1}".format(cam_info.camera_configuration.firmware_version,
                                                       cam_info.sensors_configuration.firmware_version))
    print("ZED Camera Resolution     : {0}x{1}".format(round(cam_info.camera_configuration.resolution.width, 2),
                                                       cam.get_camera_information().camera_configuration.resolution.height))
    print("ZED Camera FPS            : {0}".format(int(cam_info.camera_configuration.fps)))


def print_help():
    print("\n\nCamera controls hotkeys:")
    print("* Increase camera settings value:  '+'")
    print("* Decrease camera settings value:  '-'")
    print("* Toggle camera settings:          's'")
    print("* Toggle camera LED:               'l' (lower L)")
    print("* Reset all parameters:            'r'")
    print("* Reset exposure ROI to full image 'f'")
    print("* Use mouse to select an image area to apply exposure (press 'a')")
    print("* Exit :                           'q'\n")


# Update camera setting on key press
def update_camera_settings(key, cam, runtime, mat):
    global led_on
    if key == 115:  # for 's' key
        # Switch camera settings
        switch_camera_settings()
    elif key == 43:  # for '+' key
        # Increase camera settings value.
        current_value = cam.get_camera_settings(camera_settings)[1]
        cam.set_camera_settings(camera_settings, current_value + step_camera_settings)
        print(str_camera_settings + ": " + str(current_value + step_camera_settings))
    elif key == 45:  # for '-' key
        # Decrease camera settings value.
        current_value = cam.get_camera_settings(camera_settings)[1]
        if current_value >= 1:
            cam.set_camera_settings(camera_settings, current_value - step_camera_settings)
            print(str_camera_settings + ": " + str(current_value - step_camera_settings))
    elif key == 114:  # for 'r' key
        # Reset all camera settings to default.
        cam.set_camera_settings(sl.VIDEO_SETTINGS.BRIGHTNESS, -1)
        cam.set_camera_settings(sl.VIDEO_SETTINGS.CONTRAST, -1)
        cam.set_camera_settings(sl.VIDEO_SETTINGS.HUE, -1)
        cam.set_camera_settings(sl.VIDEO_SETTINGS.SATURATION, -1)
        cam.set_camera_settings(sl.VIDEO_SETTINGS.SHARPNESS, -1)
        cam.set_camera_settings(sl.VIDEO_SETTINGS.GAIN, -1)
        cam.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, -1)
        cam.set_camera_settings(sl.VIDEO_SETTINGS.WHITEBALANCE_TEMPERATURE, -1)
        print("[Sample] Reset all settings to default")
    elif key == 108:  # for 'l' key
        # Turn on or off camera LED.
        led_on = not led_on
        cam.set_camera_settings(sl.VIDEO_SETTINGS.LED_STATUS, led_on)
    elif key == 97:  # for 'a' key
        # Set exposure region of interest (ROI) on a target area.
        print("[Sample] set AEC_AGC_ROI on target [", selection_rect.x, ",", selection_rect.y, ",",
              selection_rect.width, ",", selection_rect.height, "]")
        cam.set_camera_settings_roi(sl.VIDEO_SETTINGS.AEC_AGC_ROI, selection_rect, sl.SIDE.BOTH)
    elif key == 102:  # for 'f' key
        # Reset exposure ROI to full resolution.
        print("[Sample] reset AEC_AGC_ROI to full res")
        cam.set_camera_settings_roi(sl.VIDEO_SETTINGS.AEC_AGC_ROI, selection_rect, sl.SIDE.BOTH, True)


# Function to switch between different camera settings (brightness, contrast, etc.).
def switch_camera_settings():
    global camera_settings
    global str_camera_settings
    if camera_settings == sl.VIDEO_SETTINGS.BRIGHTNESS:
        camera_settings = sl.VIDEO_SETTINGS.CONTRAST
        str_camera_settings = "Contrast"
        print("[Sample] Switch to camera settings: CONTRAST")
    elif camera_settings == sl.VIDEO_SETTINGS.CONTRAST:
        camera_settings = sl.VIDEO_SETTINGS.HUE
        str_camera_settings = "Hue"
        print("[Sample] Switch to camera settings: HUE")
    elif camera_settings == sl.VIDEO_SETTINGS.HUE:
        camera_settings = sl.VIDEO_SETTINGS.SATURATION
        str_camera_settings = "Saturation"
        print("[Sample] Switch to camera settings: SATURATION")
    elif camera_settings == sl.VIDEO_SETTINGS.SATURATION:
        camera_settings = sl.VIDEO_SETTINGS.SHARPNESS
        str_camera_settings = "Sharpness"
        print("[Sample] Switch to camera settings: Sharpness")
    elif camera_settings == sl.VIDEO_SETTINGS.SHARPNESS:
        camera_settings = sl.VIDEO_SETTINGS.GAIN
        str_camera_settings = "Gain"
        print("[Sample] Switch to camera settings: GAIN")
    elif camera_settings == sl.VIDEO_SETTINGS.GAIN:
        camera_settings = sl.VIDEO_SETTINGS.EXPOSURE
        str_camera_settings = "Exposure"
        print("[Sample] Switch to camera settings: EXPOSURE")
    elif camera_settings == sl.VIDEO_SETTINGS.EXPOSURE:
        camera_settings = sl.VIDEO_SETTINGS.WHITEBALANCE_TEMPERATURE
        str_camera_settings = "White Balance"
        print("[Sample] Switch to camera settings: WHITEBALANCE")
    elif camera_settings == sl.VIDEO_SETTINGS.WHITEBALANCE_TEMPERATURE:
        camera_settings = sl.VIDEO_SETTINGS.BRIGHTNESS
        str_camera_settings = "Brightness"
        print("[Sample] Switch to camera settings: BRIGHTNESS")


def valid_ip_or_hostname(ip_or_hostname):
    try:
        host, port = ip_or_hostname.split(':')
        socket.inet_aton(host)  # Vérifier si c'est une adresse IP valide
        port = int(port)
        return f"{host}:{port}"
    except (socket.error, ValueError):
        raise argparse.ArgumentTypeError("Invalid IP address or hostname format. Use format a.b.c.d:p or hostname:p")


def save_video():
    pass


def get_cam_img(ip,port):

    pass


def record_cam(ip, port):
    cam_list = []
    cam_info_list = []
    mat_list = []
    video_writer_list = []
    init_parameters = sl.InitParameters()
    init_parameters.depth_mode = sl.DEPTH_MODE.NONE
    init_parameters.sdk_verbose = 1
    init_parameters.set_from_stream(ip, port)
    left_cam = sl.Camera()
    right_cam = sl.Camera()

    status = left_cam.open(init_parameters)
    status = right_cam.open(init_parameters)
    if status != sl.ERROR_CODE.SUCCESS:
        print("Camera Open : " + repr(status) + ". Exit program.")
        exit()
    runtime = sl.RuntimeParameters()

    left_mat = sl.Mat()
    right_mat = sl.Mat()
    # cv2.namedWindow(left_win_name)
    # cv2.setMouseCallback(left_win_name, on_mouse)
    print_camera_information(left_cam)
    print_camera_information(right_cam)

    print_help()
    switch_camera_settings()

    # 计算帧数，分辨率
    start_time = time.time()
    left_cam.retrieve_image(left_mat, sl.VIEW.LEFT)  # Retrieve left image
    right_cam.retrieve_image(right_mat, sl.VIEW.LEFT)  # Retrieve right image
    left_cam_info = left_cam.get_camera_information()
    right_cam_info = right_cam.get_camera_information()

    left_win_name = "Camera {} Left Lens".format(left_cam_info.serial_number)
    right_win_name = "Camera {} Right Lens".format(right_cam_info.serial_number)
    cv2.namedWindow(left_win_name)
    cv2.namedWindow(right_win_name)
    left_cvImage = left_mat.get_data()
    right_cvImage = right_mat.get_data()
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # 视频编解码器
    # print(cvImage.shape)
    frame_width = left_cam_info.camera_configuration.resolution.width
    frame_height = left_cam_info.camera_configuration.resolution.height
    # print(dir(cam_info))
    print('相机序列号：%s' % left_cam_info.serial_number)
    print('相机型号：%s' % left_cam_info.camera_model)
    print('相机分辨率: width:%s, height:%s' % (
        left_cam_info.camera_configuration.resolution.width, left_cam_info.camera_configuration.resolution.height))
    # print('相机FPS：%s' % left_cam_info.camera_configuration.fps)
    # print('相机外部参数：')
    # print('相机旋转矩阵R：%s' % cam_info.calibration_parameters.R)
    # print('相机变换矩阵T：%s' % cam_info.calibration_parameters.T)
    # print('相机基距：%s' % cam_info.calibration_parameters.get_camera_baseline())

    fps = float('%.2f' % (1 / (time.time() - start_time)))
    left_out = cv2.VideoWriter('C:/video_save/{}_left.mp4v'.format(left_cam_info.serial_number), fourcc,
                               left_cam_info.camera_configuration.fps,
                               frameSize=(frame_width, frame_height))  # 写入视频
    right_out = cv2.VideoWriter('C:/video_save/{}_right.mp4v'.format(left_cam_info.serial_number), fourcc,
                                right_cam_info.camera_configuration.fps,
                                frameSize=(frame_width, frame_height))  # 写入视频
    key = ''
    start_time = time.time()
    left_count = 0
    right_count = 0
    left_fps = 0
    right_fps = 0
    while key != 113:  # for 'q' key
        # left camera
        err = left_cam.grab(runtime)  # Check that a new image is successfully acquired
        if err == sl.ERROR_CODE.SUCCESS:
            left_cam.retrieve_image(left_mat, sl.VIEW.LEFT)  # Retrieve left image
            left_cvImage = left_mat.get_data()
            if (not selection_rect.is_empty() and selection_rect.is_contained(
                    sl.Rect(0, 0, left_cvImage.shape[1], left_cvImage.shape[0]))):
                cv2.rectangle(left_cvImage, (selection_rect.x, selection_rect.y),
                              (selection_rect.width + selection_rect.x, selection_rect.height + selection_rect.y),
                              (220, 180, 20), 2)
            cv2.putText(left_cvImage, "Resolution:{}x{}".format(frame_width, frame_height), (0, 1 * 24),
                        cv2.FONT_HERSHEY_PLAIN, 2.0, (0, 0, 255), 2)
            cv2.putText(left_cvImage, "FPS:{}".format(left_fps), (0, 2 * 24),
                        cv2.FONT_HERSHEY_PLAIN, 2.0, (0, 0, 255), 2)
            cv2.imshow(left_win_name, left_cvImage)
            left_out.write(left_cvImage)  # 写入帧
            del left_cvImage
            left_count += 1
        else:
            print("Error during capture : ", err)
            break
        key = cv2.waitKey(5)
        update_camera_settings(key, left_cam, runtime, left_mat)
        # right camera
        err = right_cam.grab(runtime)  # Check that a new image is successfully acquired
        if err == sl.ERROR_CODE.SUCCESS:
            right_cam.retrieve_image(right_mat, sl.VIEW.RIGHT)  # Retrieve left image
            right_cvImage = right_mat.get_data()
            if (not selection_rect.is_empty() and selection_rect.is_contained(
                    sl.Rect(0, 0, right_cvImage.shape[1], right_cvImage.shape[0]))):
                cv2.rectangle(right_cvImage, (selection_rect.x, selection_rect.y),
                              (selection_rect.width + selection_rect.x, selection_rect.height + selection_rect.y),
                              (220, 180, 20), 2)
            cv2.putText(right_cvImage, "Resolution:{}x{}".format(frame_width, frame_height), (0, 1 * 24),
                        cv2.FONT_HERSHEY_PLAIN, 2.0, (0, 0, 255), 2)
            cv2.putText(right_cvImage, "FPS:{}".format(right_fps), (0, 2 * 24),
                        cv2.FONT_HERSHEY_PLAIN, 2.0, (0, 0, 255), 2)
            cv2.imshow(right_win_name, right_cvImage)
            right_out.write(right_cvImage)  # 写入帧
            del right_cvImage
            right_count += 1

        else:
            print("Error during capture : ", err)
            break
        key = cv2.waitKey(5)
        if key == 113:  # 主窗口监听Q键
            print("开始退出")
            exit_event.set()
        update_camera_settings(key, right_cam, runtime, right_mat)

        if (time.time() - start_time) > 1:  # 每秒计算并刷新一次帧率
            left_fps = '%.2f' % (left_count / (time.time() - start_time))
            right_fps = '%.2f' % (right_count / (time.time() - start_time))
            start_time = time.time()
            left_count = 0
            right_count = 0
    # out.release()
    cv2.destroyAllWindows()
    left_out.release()
    right_out.release()
    print("视频保存成功")
    left_cam.close()
    right_cam.close()
    pass


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    # parser.add_argument('--ip_address', type=valid_ip_or_hostname,
    #                     help='IP address or hostname of the sender. Should be in format a.b.c.d:p or hostname:p',
    #                     required=True)
    parser.add_argument('--ip_address', type=valid_ip_or_hostname,
                        help='IP address or hostname of the sender. Should be in format a.b.c.d:p or hostname:p',
                        default="192.168.101.35:30000")
    parser.add_argument("--video_save_dir", type=str, default="C:/video_save")
    parser.add_argument("--cam_num", type=int, default=4)
    opt = parser.parse_args()
    main(opt)
