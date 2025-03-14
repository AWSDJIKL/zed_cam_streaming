from typing import Final

import numpy as np
import pyzed.sl as sl
import cv2
import argparse
import socket
import time
from threading import Thread
from threading import Event
import os
import threading
import keyboard

camera_settings = sl.VIDEO_SETTINGS.BRIGHTNESS
str_camera_settings = "BRIGHTNESS"
step_camera_settings = 1
led_on = True
selection_rect = sl.Rect()
select_in_progress = False
origin_rect = (-1, -1)


class Camera(threading.Thread):
    def __init__(self, ip, port, save_dir=""):
        super().__init__()
        # 根据ip和port获取摄像头
        self.ip = ip
        self.port = port
        self.save_dir = save_dir
        init_parameters = sl.InitParameters()
        init_parameters.depth_mode = sl.DEPTH_MODE.NONE
        init_parameters.sdk_verbose = 1
        init_parameters.set_from_stream(ip, port)
        self.left_cam = sl.Camera()
        self.right_cam = sl.Camera()

        self.left_img_stack = []
        self.right_img_stack = []

        self.flag = True

        status = self.left_cam.open(init_parameters)
        if status != sl.ERROR_CODE.SUCCESS:
            print("Camera Open : " + repr(status) + ". Exit program.")
            exit()
        status = self.right_cam.open(init_parameters)
        if status != sl.ERROR_CODE.SUCCESS:
            print("Camera Open : " + repr(status) + ". Exit program.")
            exit()
        self.runtime = sl.RuntimeParameters()

        self.left_mat = sl.Mat()
        self.right_mat = sl.Mat()
        # cv2.namedWindow(left_win_name)
        # cv2.setMouseCallback(left_win_name, on_mouse)
        self.print_camera_information(self.left_cam)
        self.print_camera_information(self.right_cam)

        self.print_help()
        self.switch_camera_settings()

        # 计算帧数，分辨率
        start_time = time.time()
        self.left_cam.retrieve_image(self.left_mat, sl.VIEW.LEFT)  # Retrieve left image
        self.right_cam.retrieve_image(self.right_mat, sl.VIEW.LEFT)  # Retrieve right image
        self.left_cam_info = self.left_cam.get_camera_information()
        self.right_cam_info = self.right_cam.get_camera_information()

        self.left_win_name = "Camera {} Left Lens".format(self.left_cam_info.serial_number)
        self.right_win_name = "Camera {} Right Lens".format(self.right_cam_info.serial_number)
        cv2.namedWindow(self.left_win_name)
        cv2.namedWindow(self.right_win_name)
        left_cvImage = self.left_mat.get_data()
        right_cvImage = self.right_mat.get_data()
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # 视频编解码器
        # print(cvImage.shape)
        frame_width = self.left_cam_info.camera_configuration.resolution.width
        frame_height = self.left_cam_info.camera_configuration.resolution.height
        # print(dir(cam_info))
        print('相机序列号：%s' % self.left_cam_info.serial_number)
        print('相机型号：%s' % self.left_cam_info.camera_model)
        print('相机分辨率: width:%s, height:%s' % (
            self.left_cam_info.camera_configuration.resolution.width,
            self.left_cam_info.camera_configuration.resolution.height))

        # fps = float('%.2f' % (1 / (time.time() - start_time)))
        self.left_out = cv2.VideoWriter('C:/video_save/{}_left.mp4v'.format(self.left_cam_info.serial_number), fourcc,
                                        self.left_cam_info.camera_configuration.fps,
                                        frameSize=(frame_width, frame_height))  # 写入视频
        self.right_out = cv2.VideoWriter('C:/video_save/{}_right.mp4v'.format(self.left_cam_info.serial_number), fourcc,
                                         self.right_cam_info.camera_configuration.fps,
                                         frameSize=(frame_width, frame_height))  # 写入视频
        self.left_count = 0
        self.right_count = 0
        self.record_thread = RecordThread(self)
        self.get_cam_img_thread = GetCamImgThread(self)
        self.flag = True

    def run(self):
        # 开启2个子线程获取图像和录制视频
        print("------------------run-------------")
        self.get_cam_img_thread.start()
        print("------------------ get cam img run---------------")
        self.record_thread.start()
        print("------------------recording run---------------")
        while self.flag:
            pass

    def __del__(self):
        pass

    def record_video(self):
        if len(self.left_img_stack) > 0:
            self.left_out.write(self.left_img_stack[0])
            self.left_img_stack.pop(0)
        if len(self.right_img_stack) > 0:
            self.right_out.write(self.right_img_stack[0])
            self.right_img_stack.pop(0)

    def end_record(self):
        self.record_thread.flag = False
        self.get_cam_img_thread.flag = False
        self.record_thread.join()
        self.get_cam_img_thread.join()
        self.flag = False

    def get_cam_img(self):
        # left camera
        print("start get cam img")
        err = self.left_cam.grab(self.runtime)  # Check that a new image is successfully acquired
        if err == sl.ERROR_CODE.SUCCESS:
            self.left_cam.retrieve_image(self.left_mat, sl.VIEW.LEFT)  # Retrieve left image
            left_cvImage = self.left_mat.get_data()
            if (not selection_rect.is_empty() and selection_rect.is_contained(
                    sl.Rect(0, 0, left_cvImage.shape[1], left_cvImage.shape[0]))):
                cv2.rectangle(left_cvImage, (selection_rect.x, selection_rect.y),
                              (selection_rect.width + selection_rect.x, selection_rect.height + selection_rect.y),
                              (220, 180, 20), 2)
            cv2.putText(left_cvImage,
                        "Resolution:{}x{}".format(self.left_cam_info.camera_configuration.resolution.width,
                                                  self.left_cam_info.camera_configuration.resolution.height),
                        (0, 1 * 24),
                        cv2.FONT_HERSHEY_PLAIN, 2.0, (0, 0, 255), 2)
            # cv2.putText(left_cvImage, "FPS:{}".format(left_fps), (0, 2 * 24),
            #             cv2.FONT_HERSHEY_PLAIN, 2.0, (0, 0, 255), 2)
            self.left_img_stack.append(left_cvImage)
            cv2.imshow(self.left_win_name, left_cvImage)
            self.left_out.write(left_cvImage)  # 写入帧
            self.left_count += 1
        key = cv2.waitKey(5)
        self.update_camera_settings(key, self.left_cam, self.runtime, self.left_mat)
        # right camera
        err = self.right_cam.grab(self.runtime)  # Check that a new image is successfully acquired
        if err == sl.ERROR_CODE.SUCCESS:
            self.right_cam.retrieve_image(self.right_mat, sl.VIEW.RIGHT)  # Retrieve left image
            right_cvImage = self.right_mat.get_data()
            if (not selection_rect.is_empty() and selection_rect.is_contained(
                    sl.Rect(0, 0, right_cvImage.shape[1], right_cvImage.shape[0]))):
                cv2.rectangle(right_cvImage, (selection_rect.x, selection_rect.y),
                              (selection_rect.width + selection_rect.x, selection_rect.height + selection_rect.y),
                              (220, 180, 20), 2)
            cv2.putText(right_cvImage,
                        "Resolution:{}x{}".format(self.right_cam_info.camera_configuration.resolution.width,
                                                  self.right_cam_info.camera_configuration.resolution.height),
                        (0, 1 * 24),
                        cv2.FONT_HERSHEY_PLAIN, 2.0, (0, 0, 255), 2)
            # cv2.putText(right_cvImage, "FPS:{}".format(right_fps), (0, 2 * 24),
            #             cv2.FONT_HERSHEY_PLAIN, 2.0, (0, 0, 255), 2)
            self.right_img_stack.append(right_cvImage)
            cv2.imshow(self.right_win_name, right_cvImage)
            # right_count += 1
        key = cv2.waitKey(5)
        self.update_camera_settings(key, self.right_cam, self.runtime, self.right_mat)
        print("get a frame image")

    def print_camera_information(self, cam):
        cam_info = cam.get_camera_information()
        print("ZED Model                 : {0}".format(cam_info.camera_model))
        print("ZED Serial Number         : {0}".format(cam_info.serial_number))
        print("ZED Camera Firmware       : {0}/{1}".format(cam_info.camera_configuration.firmware_version,
                                                           cam_info.sensors_configuration.firmware_version))
        print("ZED Camera Resolution     : {0}x{1}".format(round(cam_info.camera_configuration.resolution.width, 2),
                                                           cam.get_camera_information().camera_configuration.resolution.height))
        print("ZED Camera FPS            : {0}".format(int(cam_info.camera_configuration.fps)))

    def print_help(self):
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
    def update_camera_settings(self, key, cam, runtime, mat):
        global led_on
        if key == 115:  # for 's' key
            # Switch camera settings
            self.switch_camera_settings()
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
    def switch_camera_settings(self):
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

    def valid_ip_or_hostname(self, ip_or_hostname):
        try:
            host, port = ip_or_hostname.split(':')
            socket.inet_aton(host)  # Vérifier si c'est une adresse IP valide
            port = int(port)
            return f"{host}:{port}"
        except (socket.error, ValueError):
            raise argparse.ArgumentTypeError(
                "Invalid IP address or hostname format. Use format a.b.c.d:p or hostname:p")


class RecordThread(threading.Thread):
    def __init__(self, cam: Camera):
        super().__init__()
        self.cam = cam
        self.flag = True

    def run(self):
        while self.flag:
            self.cam.record_video()
        while len(self.cam.left_img_stack) > 0:

            self.cam.left_out.write(self.cam.left_img_stack[0])
            self.cam.left_img_stack.pop(0)
        while len(self.cam.right_img_stack) > 0:
            self.cam.right_out.write(self.cam.right_img_stack[0])
            self.cam.right_img_stack.pop(0)

        self.cam.left_out.release()
        self.cam.right_out.release()

    def __del__(self):
        pass


class GetCamImgThread(threading.Thread):
    def __init__(self, cam: Camera):
        super().__init__()
        self.cam = cam
        self.flag = True

    def run(self):
        while self.flag:
            self.cam.get_cam_img()

    def __del__(self):
        pass
