# -*- coding: utf-8 -*-
#
# Description: Python controller for an e-puck robot implementing a robust
#              wall-following behavior using a P-controller.
#
# Original Author: Cyberbotics Ltd.
# Python Conversion & Enhancement: Gemini
#

from controller import Robot

# --- Global Defines ---

# Time step in milliseconds
TIME_STEP = 32

# Constants for sides
NO_SIDE = -1
LEFT = 0
RIGHT = 1

# --- Proximity Sensors (IR) ---
NB_DIST_SENS = 8
PS_RIGHT_00 = 0
PS_RIGHT_45 = 1
PS_RIGHT_90 = 2
PS_RIGHT_REAR = 3
PS_LEFT_REAR = 4
PS_LEFT_90 = 5
PS_LEFT_45 = 6
PS_LEFT_00 = 7

# --- Behavioral Module Constants ---

# ค่าที่ใช้ในการตัดสินใจเริ่มและหยุดพฤติกรรมเดินตามกำแพง
OBSTACLE_THRESHOLD = 100.0      # ค่า sensor ขั้นต่ำที่ด้านหน้าเพื่อเริ่มพฤติกรรม
WALL_LOST_THRESHOLD = 60.0      # ค่า sensor ขั้นต่ำที่ด้านข้างเพื่อตัดสินว่ากำแพงหายไป

# ค่าคงที่สำหรับ P-controller ในการเดินตามกำแพง
TARGET_DISTANCE = 200.0         # (เป้าหมาย) ค่า sensor ที่ต้องการเพื่อรักษาระยะห่างจากกำแพง
BASE_SPEED = 200                # ความเร็วพื้นฐานขณะเดินตามกำแพง
WALL_FOLLOW_KP = 0.4            # (Gain) ค่าคงที่สำหรับปรับความแรงในการเลี้ยว (ยิ่งมากยิ่งเลี้ยวแรง)
SHARP_TURN_SPEED = 300          # ความเร็วในการเลี้ยวหักศอกเมื่อเจอทางตัน

# --- Main Controller Class ---

class WallFollower(Robot):
    """
    A controller class for the e-puck robot that implements
    a robust wall-following behavior.
    """
    def __init__(self):
        super(WallFollower, self).__init__()
        
        # --- State Variables ---
        self.following_wall = False # สถานะว่ากำลังเดินตามกำแพงอยู่หรือไม่
        self.wall_side = NO_SIDE    # ด้านของกำแพงที่กำลังตามอยู่ (ซ้าย/ขวา)
        self.final_speed = [0, 0]   # ความเร็วสุดท้ายที่จะส่งให้มอเตอร์
        
        # --- Initialize Devices ---
        
        # Proximity Sensors
        self.ps = []
        ps_names = [f'ps{i}' for i in range(NB_DIST_SENS)]
        for name in ps_names:
            sensor = self.getDevice(name)
            sensor.enable(TIME_STEP)
            self.ps.append(sensor)
            
        # Motors
        self.left_motor = self.getDevice('left wheel motor')
        self.right_motor = self.getDevice('right wheel motor')
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

    def run(self):
        """
        Main control loop.
        """
        while self.step(TIME_STEP) != -1:
            # 1. อ่านค่าจากเซ็นเซอร์
            ps_values = [sensor.getValue() for sensor in self.ps]

            # 2. เรียกใช้ Controller หลักเพื่อคำนวณความเร็ว
            self.wall_following_controller(ps_values)
            
            # 3. ดึงค่าความเร็วสุดท้าย
            final_speed_left = self.final_speed[LEFT]
            final_speed_right = self.final_speed[RIGHT]

            # Debug print
            mode = "Following" if self.following_wall else "Searching"
            side_str = "Left" if self.wall_side == LEFT else "Right" if self.wall_side == RIGHT else "None"
            print(f"Mode: {mode}, Side: {side_str}, "
                  f"L_Speed: {final_speed_left:.2f}, R_Speed: {final_speed_right:.2f}")

            # 4. ตั้งค่าความเร็วมอเตอร์
            # ค่า 0.00628 ใช้สำหรับแปลงหน่วยความเร็วให้เป็น rad/s ที่มอเตอร์ต้องการ
            self.left_motor.setVelocity(final_speed_left * 0.00628)
            self.right_motor.setVelocity(final_speed_right * 0.00628)

    def wall_following_controller(self, ps_values):
        """
        Controller หลักที่รวมพฤติกรรมการค้นหา, การเริ่มตาม, และการรักษาระยะห่างจากกำแพง
        """
        # --- ตรวจจับสถานะ ---
        front_obstacle_detected = (ps_values[PS_LEFT_00] > OBSTACLE_THRESHOLD or
                                   ps_values[PS_RIGHT_00] > OBSTACLE_THRESHOLD)

        # เงื่อนไข: เริ่มเดินตามกำแพงเมื่อเจอสิ่งกีดขวางด้านหน้าครั้งแรก
        if not self.following_wall and front_obstacle_detected:
            self.following_wall = True
            # ตัดสินใจว่าจะตามกำแพงด้านไหน โดยดูว่าฝั่งไหนมีค่า sensor รวมสูงกว่า
            activation_left = ps_values[PS_LEFT_45] + ps_values[PS_LEFT_00]
            activation_right = ps_values[PS_RIGHT_45] + ps_values[PS_RIGHT_00]
            self.wall_side = LEFT if activation_left > activation_right else RIGHT
        
        # เงื่อนไข: หยุดเดินตามกำแพงเมื่อกำแพงด้านที่ตามอยู่หายไป
        elif self.following_wall:
             side_is_clear = False
             if self.wall_side == RIGHT:
                 if ps_values[PS_RIGHT_90] < WALL_LOST_THRESHOLD and ps_values[PS_RIGHT_45] < WALL_LOST_THRESHOLD:
                     side_is_clear = True
             elif self.wall_side == LEFT:
                 if ps_values[PS_LEFT_90] < WALL_LOST_THRESHOLD and ps_values[PS_LEFT_45] < WALL_LOST_THRESHOLD:
                     side_is_clear = True
             
             if side_is_clear:
                 self.following_wall = False
                 self.wall_side = NO_SIDE

        # --- คำนวณความเร็ว ---
        
        # ถ้าไม่ได้อยู่ในโหมดตามกำแพง -> เดินหน้าตรง
        if not self.following_wall:
            self.final_speed = [BASE_SPEED, BASE_SPEED]
            return

        # ถ้ากำลังอยู่ในโหมดตามกำแพง
        
        # กรณีฉุกเฉิน: ถ้าเจอทางตันหรือมุมอับ (sensor หน้าทำงาน) -> เลี้ยวหักศอก
        if front_obstacle_detected:
            left_speed = BASE_SPEED
            right_speed = BASE_SPEED
            if self.wall_side == LEFT:
                # ถ้าตามกำแพงซ้ายอยู่ ให้เลี้ยวขวาหักศอก
                left_speed += SHARP_TURN_SPEED
                right_speed -= SHARP_TURN_SPEED
            else:
                # ถ้าตามกำแพงขวาอยู่ ให้เลี้ยวซ้ายหักศอก
                left_speed -= SHARP_TURN_SPEED
                right_speed += SHARP_TURN_SPEED
            self.final_speed = [left_speed, right_speed]
            return

        # กรณีปกติ: เดินตามกำแพงโดยรักษาระยะห่าง (P-controller)
        error = 0
        
        if self.wall_side == RIGHT:
            distance = ps_values[PS_RIGHT_90]
            error = distance - TARGET_DISTANCE # คำนวณค่า error จากระยะห่างเป้าหมาย
            
            # คำนวณค่าการปรับแก้ความเร็ว
            correction = WALL_FOLLOW_KP * error
            
            # ถ้า error > 0 (ใกล้ไป) -> เลี้ยวซ้าย (เพิ่มความเร็วล้อซ้าย, ลดล้อขวา)
            # ถ้า error < 0 (ไกลไป) -> เลี้ยวขวา (ลดความเร็วล้อซ้าย, เพิ่มล้อขวา)
            left_speed = BASE_SPEED + correction
            right_speed = BASE_SPEED - correction
            
        else: # self.wall_side == LEFT
            distance = ps_values[PS_LEFT_90]
            error = distance - TARGET_DISTANCE # คำนวณค่า error
            
            # คำนวณค่าการปรับแก้ความเร็ว
            correction = WALL_FOLLOW_KP * error

            # ถ้า error > 0 (ใกล้ไป) -> เลี้ยวขวา (ลดความเร็วล้อซ้าย, เพิ่มล้อขวา)
            # ถ้า error < 0 (ไกลไป) -> เลี้ยวซ้าย (เพิ่มความเร็วล้อซ้าย, ลดล้อขวา)
            left_speed = BASE_SPEED - correction
            right_speed = BASE_SPEED + correction

        self.final_speed = [left_speed, right_speed]


# --- Main Execution ---
if __name__ == "__main__":
    controller = WallFollower()
    controller.run()