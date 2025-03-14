import camera
import keyboard

if __name__ == '__main__':
    camera_list = []
    for i in range(1):
        camera_list.append(camera.Camera("192.168.101.35", 30000 + i * 2))

    for c in camera_list:
        c.start()

    # 监听键盘，按下Q键结束
    print("start listening")
    while True:
        print(1)
        if keyboard.is_pressed('q'):
            print('quit')
            for c in camera_list:
                c.end_record()
            for c in camera_list:
                c.join()
            break
