'''
树莓派引脚图（BCM编号）：
     3V3  (1)  (2)   5V
   GPIO2  (3)  (4)   5V
   GPIO3  (5)  (6)   GND
   GPIO4  (7)  (8)   GPIO14 (TXD)
      GND (9)  (10)  GPIO15 (RXD)
  GPIO17 (11)  (12)  GPIO18 (PWM0)
  GPIO27 (13)  (14)  GND
  GPIO22 (15)  (16)  GPIO23
      3V3 (17)  (18) GPIO24
  GPIO10 (19)  (20)  GND
   GPIO9 (21)  (22) GPIO25
  GPIO11 (23)  (24) GPIO8
      GND (25)  (26) GPIO7
   GPIO0 (27)  (28) GPIO1
   GPIO5 (29)  (30) GND
   GPIO6 (31)  (32) GPIO12 (PWM0)
  GPIO13 (33)  (34) GND
  GPIO19 (35)  (36) GPIO16
  GPIO26 (37)  (38) GPIO20
      GND (39)  (40) GPIO21
'''

import RPi.GPIO as GPIO
from remote_listener import Listener
import threading
import time

listener = Listener()
threading.Thread(target=listener.listen, daemon=True).start()


class DCMotorController(object):
    def __init__(self, vel_a=12, vel_b=13, vel_c=18, dir_a=23, dir_b=24, pwm_freq=10000):
        """
        初始化直流电机控制器
        
        参数:
        vel_a: 电机A PWM速度控制引脚 (BCM编号)
        vel_b: 电机B PWM速度控制引脚 (BCM编号)  
        dir_a: 电机A 方向控制引脚 (BCM编号)
        dir_b: 电机B 方向控制引脚 (BCM编号)
        pwm_freq: PWM频率(Hz)，默认10kHz
        """
        # 引脚定义
        self.DIR_A = dir_a    # 电机A方向控制
        self.DIR_B = dir_b    # 电机B方向控制
        self.VEL_A = vel_a    # 电机A速度控制 (PWM)
        self.VEL_B = vel_b    # 电机B速度控制 (PWM)
        self.VEL_C = vel_c    # chuxiu dian ji
        # 状态变量
        self.current_speed_a = 0    # 当前速度A (0-100)
        self.current_speed_b = 0    # 当前速度B (0-100)
        self.current_speed_c = 0
        self.direction_a = True     # True: 正向, False: 反向
        self.direction_b = True     # True: 正向, False: 反向
        self.max_speed = 100        # 最大速度
        self.min_speed = 0          # 最小速度
        self.speed_step = 10        # 每次加速/减速的步长
        
        # 初始化GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.VEL_A, GPIO.OUT)
        GPIO.setup(self.VEL_B, GPIO.OUT)
        GPIO.setup(self.VEL_C, GPIO.OUT)
        GPIO.setup(self.DIR_A, GPIO.OUT)
        GPIO.setup(self.DIR_B, GPIO.OUT)
        

        # 初始化PWM
        self.PWM_A = GPIO.PWM(self.VEL_A, pwm_freq)
        self.PWM_B = GPIO.PWM(self.VEL_B, pwm_freq)
        self.PWM_C = GPIO.PWM(self.VEL_C, pwm_freq)
        self.PWM_A.start(0)  # 初始占空比为0
        self.PWM_B.start(0)  # 初始占空比为0
        self.PWM_C.start(0)
        
        print("直流电机控制器初始化完成")
        print(f"引脚配置: DIR_A:GPIO{dir_a}, DIR_B:GPIO{dir_b}, VEL_A:GPIO{vel_a}, VEL_B:GPIO{vel_b}, VEL_C:GPIO{vel_c}")
        print(f"PWM频率: {pwm_freq}Hz")
        
    def set_motor_a(self, speed, direction=True):
        """设置电机A的速度和方向"""
        speed = max(self.min_speed, min(self.max_speed, speed))  # 限制速度范围
        self.current_speed_a = speed
        self.direction_a = direction
        
        # 设置方向
        GPIO.output(self.DIR_A, GPIO.HIGH if direction else GPIO.LOW)
        
        # 设置PWM占空比
        self.PWM_A.ChangeDutyCycle(speed)
        
        dir_text = "正向" if direction else "反向"
        print(f"电机A: 速度{speed}%, 方向{dir_text}")
        
    def set_motor_b(self, speed, direction=True):
        """设置电机B的速度和方向"""
        speed = max(self.min_speed, min(self.max_speed, speed))  # 限制速度范围
        self.current_speed_b = speed
        self.direction_b = direction
        
        # 设置方向
        GPIO.output(self.DIR_B, GPIO.HIGH if direction else GPIO.LOW)
        
        # 设置PWM占空比
        self.PWM_B.ChangeDutyCycle(speed)
        
        dir_text = "正向" if direction else "反向"
        print(f"电机B: 速度{speed}%, 方向{dir_text}")
    
    def set_motor_c(self, speed):
        """设置电机B的速度和方向"""
        speed = max(self.min_speed, min(self.max_speed, speed))  # 限制速度范围
        self.current_speed_c = speed
        
        # 设置PWM占空比
        self.PWM_C.ChangeDutyCycle(speed)
    
        print(f"除锈电机C: 速度{speed}%")
      


    def set_both_motors(self, speed, direction=True):
        """同时设置两个电机的速度和方向"""
        self.set_motor_a(speed, direction)
        self.set_motor_b(speed, direction)
        
    def forward(self, speed=None):
        """前进"""
        if speed is None:
            speed = max(self.current_speed_a, self.current_speed_b, 30)
        self.set_both_motors(speed, True)
        print(f"前进: 速度{speed}%")
        
    def backward(self, speed=None):
        """后退"""
        if speed is None:
            speed = max(self.current_speed_a, self.current_speed_b, 30)
        self.set_both_motors(speed, False)
        print(f"后退: 速度{speed}%")

    def derusting(self, speed=None):
        if speed is None:
            speed = max(self.current_speed_c, 30)
        self.set_motor_c(speed)
        print(f"除锈: 速度{speed}%")



    def turn_left(self, speed=None, turn_type="pivot"):
        """
        左转
        
        参数:
        speed: 转弯速度
        turn_type: 转弯类型
            "pivot" - 原地旋转 (A正转, B反转)
            "differential" - 差速转弯 (A慢, B快)
        """
        if speed is None:
            speed = max(self.current_speed_a, self.current_speed_b, 30)
            
        if turn_type == "pivot":
            # 原地旋转：A正转，B反转
            self.set_motor_a(speed, True)
            self.set_motor_b(speed, False)
            print(f"原地左转: 速度{speed}%")
            
        elif turn_type == "differential":
            # 差速转弯：A慢，B快
            speed_a = max(0, speed - 30)
            speed_b = min(100, speed + 30)
            self.set_motor_a(speed_a, True)
            self.set_motor_b(speed_b, True)
            print(f"差速左转: A{speed_a}%, B{speed_b}%")
            
    def turn_right(self, speed=None, turn_type="pivot"):
        """
        右转
        
        参数:
        speed: 转弯速度
        turn_type: 转弯类型
            "pivot" - 原地旋转 (A反转, B正转)
            "differential" - 差速转弯 (A快, B慢)
        """
        if speed is None:
            speed = max(self.current_speed_a, self.current_speed_b, 30)
            
        if turn_type == "pivot":
            # 原地旋转：A反转，B正转
            self.set_motor_a(speed, False)
            self.set_motor_b(speed, True)
            print(f"原地右转: 速度{speed}%")
            
        elif turn_type == "differential":
            # 差速转弯：A快，B慢
            speed_a = min(100, speed + 30)
            speed_b = max(0, speed - 30)
            self.set_motor_a(speed_a, True)
            self.set_motor_b(speed_b, True)
            print(f"差速右转: A{speed_a}%, B{speed_b}%")
            
    def accelerate(self, increment=None):
        """加速"""
        if increment is None:
            increment = self.speed_step
            
        new_speed_a = min(self.max_speed, self.current_speed_a + increment)
        new_speed_b = min(self.max_speed, self.current_speed_b + increment)
        
        # 保持当前方向
        self.set_motor_a(new_speed_a, self.direction_a)
        self.set_motor_b(new_speed_b, self.direction_b)
        
        print(f"加速: A{self.current_speed_a}% → {new_speed_a}%, B{self.current_speed_b}% → {new_speed_b}%")
        
    def decelerate(self, decrement=None):
        """减速"""
        if decrement is None:
            decrement = self.speed_step
            
        new_speed_a = max(self.min_speed, self.current_speed_a - decrement)
        new_speed_b = max(self.min_speed, self.current_speed_b - decrement)
        
        # 保持当前方向
        self.set_motor_a(new_speed_a, self.direction_a)
        self.set_motor_b(new_speed_b, self.direction_b)
        
        print(f"减速: A{self.current_speed_a}% → {new_speed_a}%, B{self.current_speed_b}% → {new_speed_b}%")
        

    def stop(self):
        """停止"""
        self.set_both_motors(0, True)
        print("停止")
        
    def get_status(self):
        """获取当前状态"""
        status = {
            "电机A": {
                "速度": f"{self.current_speed_a}%",
                "方向": "正向" if self.direction_a else "反向"
            },
            "电机B": {
                "速度": f"{self.current_speed_b}%",
                "方向": "正向" if self.direction_b else "反向"
            }
        }
        return status
        
    def cleanup(self):
        """清理GPIO资源"""
        self.stop()
        self.PWM_A.stop()
        self.PWM_B.stop()
        GPIO.cleanup()
        print("GPIO资源已清理")


class KeyboardController:
    """键盘控制类"""
    
    def __init__(self, motor_controller):
        self.mc = motor_controller
        self.running = True
        self.turn_type = "pivot"  # 默认转弯类型
        self.speed_step = 10      # 默认速度步长
        
        # 控制按键映射
        self.key_actions = {
            'w': ('前进', lambda: self.mc.forward(listener.vel)),
            's': ('后退', lambda: self.mc.backward(listener.vel)),
            'a': ('左转', lambda: self.mc.turn_left(speed=listener.vel, turn_type=self.turn_type)),
            'd': ('右转', lambda: self.mc.turn_right(speed=listener.vel, turn_type=self.turn_type)),
            'q': ('加速', lambda: self.mc.accelerate()),
            'e': ('减速', lambda: self.mc.decelerate()),
            'h': ('停止', lambda: self.mc.stop()),
            'g': ('除锈', lambda: self.mc.derusting(100)),
            'j': ('停止除锈', lambda: self.mc.derusting(0)),
            '1': ('设置差速转弯', lambda: self.set_turn_type("differential")),
            '2': ('设置原地转弯', lambda: self.set_turn_type("pivot")),
            '+': ('增加速度步长', lambda: self.change_speed_step(5)),
            '-': ('减少速度步长', lambda: self.change_speed_step(-5)),
            'i': ('显示状态', self.show_status),
            'x': ('退出', self.exit_program)
        }
        
    def set_turn_type(self, turn_type):
        """设置转弯类型"""
        self.turn_type = turn_type
        print(f"转弯类型已设置为: {turn_type}")
        
    def change_speed_step(self, delta):
        """改变速度步长"""
        new_step = max(1, min(50, self.speed_step + delta))
        self.speed_step = new_step
        self.mc.speed_step = new_step
        print(f"速度步长已设置为: {new_step}")
        
    def show_status(self):
        """显示状态"""
        print("\n" + "="*50)
        print("当前状态:")
        status = self.mc.get_status()
        for motor, info in status.items():
            print(f"  {motor}: 速度{info['速度']}, 方向{info['方向']}")
        print(f"转弯类型: {self.turn_type}")
        print(f"速度步长: {self.speed_step}")
        print("="*50)
        
    def exit_program(self):
        """退出程序"""
        self.running = False
        print("正在退出...")
        
    def print_menu(self):
        """打印控制菜单"""
        print("\n" + "="*50)
        print("直流电机键盘控制器")
        print("="*50)
        print("按键控制:")
        for key, (description, _) in self.key_actions.items():
            print(f"  [{key}] - {description}")
        print("\n" + "="*50)
        
    def start(self):
        """开始键盘控制"""
        self.print_menu()
        
        try:
            # 注意：在树莓派上可能需要使用不同的方法获取键盘输入
            # 这里使用简单的input()，但可能需要异步处理
            print("\n按对应按键控制，然后按Enter键确认")
            print("或按 'x' 退出")
            current_key = None
            current_vel = None
            while self.running:
                # 获取用户输入
                if listener.keys is None:
                    time.sleep(0.1)
                    continue
                if len(listener.keys) == 0:
                    time.sleep(0.1)
                    continue
                key_input = listener.keys[-1]
                if key_input in ['K5', 'K1', 'K2'] and key_input == current_key:
                    time.sleep(0.1)
                    continue
                if key_input in ['l_left', 'l_right', 'l_up', 'l_down'] and current_vel == listener.vel:
                    time.sleep(0.1)
                    continue
                    
                if not key_input:
                    continue
                current_key = key_input
                current_vel = listener.vel

                if key_input == 'l_left':
                    key = 'a'
                elif key_input == 'l_right':
                    key = 'd'
                elif key_input == 'l_up':
                    key = 'w'
                elif key_input == 'l_down':
                    key = 's'
                elif key_input == 'K5':
                    key = 'h'
                elif key_input == 'K1':
                    key = 'g'
                elif key_input == 'K2':
                    key = 'j'
                else:
                    print(' '*20)

                
                if key in self.key_actions:
                    description, action = self.key_actions[key]
                    print(f"执行: {description}")
                    action()
                else:
                    print(f"未知按键: {key}")
                    self.print_menu()
                    
        except KeyboardInterrupt:
            print("\n检测到中断信号")
        finally:
            self.mc.cleanup()



if __name__ == "__main__":
    try:
        # 创建电机控制器实例
        print("初始化电机控制器...")
        motor_ctrl = DCMotorController(
            vel_a=12,    # GPIO12 - 电机A速度
            vel_b=13,    # GPIO13 - 电机B速度
            vel_c=18,
            dir_a=23,    # GPIO23 - 电机A方向
            dir_b=24,    # GPIO24 - 电机B方向
            pwm_freq=10000  # PWM频率10kHz
        )
        # 键盘控制模式
        kb_ctrl = KeyboardController(motor_ctrl)
        kb_ctrl.start()
           
            
            
    except Exception as e:
        print(f"发生错误: {e}")
        # 确保清理GPIO
        try:
            GPIO.cleanup()
        except:
            pass