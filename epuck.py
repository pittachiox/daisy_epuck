# -*- coding: utf-8 -*-
#
# Description: Python controller for an e-puck robot implementing a robust
#              wall-following behavior using a PD-controller. This version
#              is a complete rework for stability and precise distance keeping.
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
# Sensor mapping for e-puck
#        Front
#        /   \
# ps7 (L_00) ps0 (R_00)
# ps6 (L_45) ps1 (R_45)
# ps5 (L_90) ps2 (R_90)
#        \   /
# ps4 (L_REAR) ps3 (R_REAR)
#        Back
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
WALL_LOST_THRESHOLD = 80.0      # ค่า sensor ขั้นต่ำที่ด้านข้างเพื่อตัดสินว่ากำแพงหายไป

# --- ค่าคงที่สำหรับจูน PD-controller ในการเดินตามกำแพง ---
# [REWORKED] ปรับจูนค่าสำหรับ PD Controller เพื่อความเสถียรสูงสุด
TARGET_DISTANCE = 120.0         # (เป้าหมาย) ค่า sensor ที่ต้องการ, ค่ายิ่งน้อย -> หุ่นยนต์ยิ่งอยู่ห่างกำแพง
BASE_SPEED = 150                # ความเร็วพื้นฐานขณะเคลื่อนที่
WALL_FOLLOW_KP = 0.5            # (Proportional Gain) อัตราขยายตามระยะห่าง
WALL_FOLLOW_KD = 1.0            # (Derivative Gain) อัตราขยายตามการเปลี่ยนแปลงของระยะห่าง (ช่วยลดการแกว่ง)
SHARP_TURN_SPEED = 200          # ความเร็วในการเลี้ยวหักศอกเมื่อเจอทางตัน

# --- Main Controller Class ---

class WallFollower(Robot):
    """
    A controller class for the e-puck robot that implements
    a robust wall-following behavior using a PD-controller.
    """
    def __init__(self):
        super(WallFollower, self).__init__()
        
        # --- State Variables ---
        self.following_wall = False
        self.wall_side = NO_SIDE
        self.final_speed = [0, 0]
        self.previous_error = 0.0   # สำหรับเก็บค่า error ครั้งก่อนหน้า (ใช้ใน D-controller)
        
        # --- Initialize Devices ---
        self.ps = []
        ps_names = [f'ps{i}' for i in range(NB_DIST_SENS)]
        for name in ps_names:
            sensor = self.getDevice(name)
            sensor.enable(TIME_STEP)
            self.ps.append(sensor)
            
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
            self.left_motor.setVelocity(final_speed_left * 0.00628)
            self.right_motor.setVelocity(final_speed_right * 0.00628)

    def wall_following_controller(self, ps_values):
        """
        Controller หลักที่ใช้ PD-controller เพื่อรักษาระยะห่างจากกำแพง
        """
        # --- อ่านค่า sensor ที่สำคัญ ---
        left_front_val = ps_values[PS_LEFT_00]
        right_front_val = ps_values[PS_RIGHT_00]
        left_side_val = ps_values[PS_LEFT_90]
        right_side_val = ps_values[PS_RIGHT_90]

        front_obstacle_detected = (left_front_val > OBSTACLE_THRESHOLD or
                                   right_front_val > OBSTACLE_THRESHOLD)

        # --- ตรรกะการเปลี่ยนสถานะ (State Machine) ---

        # สถานะ 1: ค้นหากำแพง (Searching)
        if not self.following_wall:
            if front_obstacle_detected:
                # เจอกำแพง -> เปลี่ยนสถานะเป็น Following และเลือกฝั่งที่จะตาม
                self.following_wall = True
                self.previous_error = 0.0 # รีเซ็ตค่า error
                # [IMPROVED] ตัดสินใจว่าจะตามกำแพงฝั่งไหน
                if left_front_val > right_front_val:
                    self.wall_side = LEFT
                else:
                    self.wall_side = RIGHT
            else:
                # ยังไม่เจอกำแพง -> เดินหน้าตรง
                self.final_speed = [BASE_SPEED, BASE_SPEED]
            return

        # สถานะ 2: กำลังเดินตามกำแพง (Following)
        
        # ตรวจสอบว่ากำแพงหายไปหรือไม่
        wall_is_lost = False
        if self.wall_side == RIGHT and right_side_val < WALL_LOST_THRESHOLD and ps_values[PS_RIGHT_45] < WALL_LOST_THRESHOLD:
            wall_is_lost = True
        elif self.wall_side == LEFT and left_side_val < WALL_LOST_THRESHOLD and ps_values[PS_LEFT_45] < WALL_LOST_THRESHOLD:
            wall_is_lost = True
        
        if wall_is_lost:
            # กำแพงหาย -> กลับไปสถานะค้นหา
            self.following_wall = False
            self.wall_side = NO_SIDE
            self.final_speed = [BASE_SPEED, BASE_SPEED]
            return

        # กรณีเจอทางตัน/มุมอับ (sensor หน้าทำงาน) -> เลี้ยวหักศอก
        if front_obstacle_detected:
            if self.wall_side == RIGHT:
                self.final_speed = [-SHARP_TURN_SPEED, SHARP_TURN_SPEED] # เลี้ยวซ้าย
            else: # LEFT
                self.final_speed = [SHARP_TURN_SPEED, -SHARP_TURN_SPEED] # เลี้ยวขวา
            return

        # --- กรณีปกติ: ใช้ PD-controller เพื่อรักษาระยะห่างจากกำแพง ---
        error = 0.0
        distance = 0.0
        
        if self.wall_side == RIGHT:
            distance = right_side_val
        else: # LEFT
            distance = left_side_val
            
        error = distance - TARGET_DISTANCE
        
        # [NEW] คำนวณค่า Derivative
        derivative = error - self.previous_error
        
        # คำนวณค่าการปรับแก้ความเร็วทั้งหมด
        correction = (WALL_FOLLOW_KP * error) + (WALL_FOLLOW_KD * derivative)
        
        # อัปเดตค่า error สำหรับ loop ถัดไป
        self.previous_error = error

        # นำค่า correction ไปปรับความเร็วล้อ
        if self.wall_side == RIGHT:
            # ถ้าใกล้ไป (error > 0) -> เลี้ยวซ้าย (เพิ่มล้อซ้าย, ลดล้อขวา)
            left_speed = BASE_SPEED + correction
            right_speed = BASE_SPEED - correction
        else: # LEFT
            # ถ้าใกล้ไป (error > 0) -> เลี้ยวขวา (ลดล้อซ้าย, เพิ่มล้อขวา)
            left_speed = BASE_SPEED - correction
            right_speed = BASE_SPEED + correction

        # จำกัดความเร็วสูงสุดและต่ำสุดเพื่อความเสถียร
        max_speed = BASE_SPEED * 1.5
        self.final_speed[LEFT] = max(-max_speed, min(left_speed, max_speed))
        self.final_speed[RIGHT] = max(-max_speed, min(right_speed, max_speed))

# --- Main Execution ---
if __name__ == "__main__":
    controller = WallFollower()
    controller.run()