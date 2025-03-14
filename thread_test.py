import threading
import time
from time import sleep
import sys
import msvcrt
import keyboard


# myThread继承父类，并进行重写
class myThread(threading.Thread):
    # 重写父类的构造函数
    def __init__(self, number, letter):
        threading.Thread.__init__(self)
        self.number = number  # 添加number变量
        self.letter = letter  # 添加letter变量
        self.sub_thread = SubThread(self)

    # 重写父类中的run函数
    def run(self):
        self.sub_thread.start()
        print("start listening")
        while True:
            if keyboard.is_pressed('q'):
                print('quit')
                self.sub_thread.flag = False
                break

        self.sub_thread.join()

    def task1(self):
        current_time = time.strftime('%H:%M:%S', time.localtime())
        print("线程{} 正在运行".format(self.number))

    # 重写父类析构函数
    def __del__(self):
        print("【父线程销毁释放内存】")


class SubThread(threading.Thread):
    def __init__(self, father):
        threading.Thread.__init__(self)
        self.father = father
        self.flag = True

    def run(self):
        while self.flag:
            sleep(1)
            self.father.task1()

    def __del__(self):
        print("【子线程销毁释放内存】")


if __name__ == '__main__':
    t_list = []
    for i in range(4):
        t_list.append(myThread(i, ""))
    for t in t_list:
        t.start()
    for t in t_list:
        t.join()
    print("end")
