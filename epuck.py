# -*- coding: utf-8 -*-
#
# Description: Python controller for an e-puck robot implementing a robust
#              wall-following behavior. This version uses a dual P-controller
#              for distance and orientation, calculates pose using odometry,
#              and logs the pose to a CSV file.
#
# Original Author: Cyberbotics Ltd.
# Python Conversion & Enhancement: Gemini
#

from controller import Robot
import csv
import math

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

# --- ค่าคงที่สำหรับจูน Controller ---
# [REWORKED] ปรับจูนค่าสำหรับ Controller ที่ควบคุมทั้งระยะห่างและมุม
TARGET_DISTANCE = 150.0         # (เป้าหมาย) ค่า sensor ที่ต้องการจากกำแพง
BASE_SPEED = 200                # ความเร็วพื้นฐานขณะเคลื่อนที่
KP_dist = 0.3                   # (Proportional Gain) อัตราขยายสำหรับรักษาระยะห่าง
KP_angle = 0.2                  # (Proportional Gain) อัตราขยายสำหรับปรับให้หุ่นยนต์ขนานกับกำแพง
SHARP_TURN_SPEED = 200          # ความเร็วในการเลี้ยวหักศอกเมื่อเจอทางตัน

# --- [NEW] E-puck Robot Parameters for Odometry ---
WHEEL_RADIUS = 0.0205  # in meters
AXLE_LENGTH = 0.052    # in meters

# --- Main Controller Class ---

class WallFollower(Robot):
    """
    A controller class for the e-puck robot that implements a robust
    wall-following behavior and calculates its pose using odometry.
    """
    def __init__(self):
        super(WallFollower, self).__init__()
        
        # --- State Variables ---
        self.following_wall = False
        self.wall_side = NO_SIDE
        self.final_speed = [0, 0]
        # [NEW] Odometry pose variables
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_phi = 0.0 # orientation in radians
        
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

        # --- [MODIFIED] ตั้งค่าไฟล์ CSV สำหรับบันทึกข้อมูล Pose ---
        try:
            self.csv_file = open('pose_log.csv', 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            # เขียนส่วนหัวของไฟล์ CSV
            self.csv_writer.writerow(['timestamp', 'x', 'y', 'phi'])
        except IOError as e:
            print(f"Error opening CSV file: {e}")
            self.csv_file = None


    def run(self):
        """
        Main control loop.
        """
        try:
            while self.step(TIME_STEP) != -1:
                # 1. อ่านค่าจากเซ็นเซอร์
                ps_values = [sensor.getValue() for sensor in self.ps]

                # 2. เรียกใช้ Controller หลักเพื่อคำนวณความเร็ว
                self.wall_following_controller(ps_values)
                
                # 3. ดึงค่าความเร็วสุดท้าย
                final_speed_left = self.final_speed[LEFT]
                final_speed_right = self.final_speed[RIGHT]

                # 4. แปลงความเร็วและตั้งค่ามอเตอร์
                left_velocity_rad_s = final_speed_left * 0.00628
                right_velocity_rad_s = final_speed_right * 0.00628
                
                self.left_motor.setVelocity(left_velocity_rad_s)
                self.right_motor.setVelocity(right_velocity_rad_s)

                # 5. [NEW] Odometry Calculation
                delta_t = TIME_STEP / 1000.0 # time step in seconds
                
                # Calculate robot's linear and angular velocities
                v_left = left_velocity_rad_s * WHEEL_RADIUS
                v_right = right_velocity_rad_s * WHEEL_RADIUS
                
                v = (v_right + v_left) / 2.0
                omega = (v_right - v_left) / AXLE_LENGTH
                
                # Update pose
                self.pose_phi += omega * delta_t
                self.pose_x += v * math.cos(self.pose_phi) * delta_t
                self.pose_y += v * math.sin(self.pose_phi) * delta_t

                # [MODIFIED] Print current pose to the console
                print(f"Pose: x={self.pose_x:.3f} m, y={self.pose_y:.3f} m, phi={self.pose_phi:.3f} rad")

                # 6. [MODIFIED] บันทึกข้อมูล Pose ลงไฟล์ CSV
                if self.csv_file:
                    timestamp = self.getTime()
                    self.csv_writer.writerow([timestamp, self.pose_x, self.pose_y, self.pose_phi])
        
        finally:
            # 7. ปิดไฟล์ CSV เมื่อจบการทำงาน
            if self.csv_file:
                self.csv_file.close()
                print("CSV pose log file closed.")


    def wall_following_controller(self, ps_values):
        """
        Controller หลักที่ใช้ P-controller 2 ส่วน เพื่อรักษาระยะห่างและปรับมุมให้ขนานกับกำแพง
        """
        # --- อ่านค่า sensor ที่สำคัญ ---
        left_front_val = ps_values[PS_LEFT_00]
        right_front_val = ps_values[PS_RIGHT_00]
        
        front_obstacle_detected = (left_front_val > OBSTACLE_THRESHOLD or
                                   right_front_val > OBSTACLE_THRESHOLD)

        # --- ตรรกะการเปลี่ยนสถานะ (State Machine) ---

        # สถานะ 1: ค้นหากำแพง (Searching)
        if not self.following_wall:
            if front_obstacle_detected:
                # เจอกำแพง -> เปลี่ยนสถานะเป็น Following และเลือกฝั่งที่จะตาม
                self.following_wall = True
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
        if self.wall_side == RIGHT and ps_values[PS_RIGHT_90] < WALL_LOST_THRESHOLD and ps_values[PS_RIGHT_45] < WALL_LOST_THRESHOLD:
            wall_is_lost = True
        elif self.wall_side == LEFT and ps_values[PS_LEFT_90] < WALL_LOST_THRESHOLD and ps_values[PS_LEFT_45] < WALL_LOST_THRESHOLD:
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

        # --- กรณีปกติ: ใช้ Controller 2 ส่วน (ระยะห่าง + มุม) ---
        
        if self.wall_side == RIGHT:
            # 1. คำนวณค่า Correction จากระยะห่าง
            dist_error = ps_values[PS_RIGHT_90] - TARGET_DISTANCE
            dist_correction = KP_dist * dist_error
            
            # 2. คำนวณค่า Correction จากมุม (พยายามทำให้ ps2 และ ps1 มีค่าเท่ากัน)
            angle_error = ps_values[PS_RIGHT_90] - ps_values[PS_RIGHT_45]
            angle_correction = KP_angle * angle_error
            
            # 3. รวมค่า Correction ทั้งหมด
            total_correction = dist_correction + angle_correction
            
            # 4. ปรับความเร็ว
            # correction > 0 หมายถึง ต้องเลี้ยวซ้าย (ห่างจากกำแพง)
            left_speed = BASE_SPEED + total_correction
            right_speed = BASE_SPEED - total_correction
            
        else: # LEFT
            # 1. คำนวณค่า Correction จากระยะห่าง
            dist_error = ps_values[PS_LEFT_90] - TARGET_DISTANCE
            dist_correction = KP_dist * dist_error
            
            # 2. คำนวณค่า Correction จากมุม (พยายามทำให้ ps5 และ ps6 มีค่าเท่ากัน)
            angle_error = ps_values[PS_LEFT_90] - ps_values[PS_LEFT_45]
            angle_correction = KP_angle * angle_error
            
            # 3. รวมค่า Correction ทั้งหมด
            total_correction = dist_correction + angle_correction
            
            # 4. ปรับความเร็ว
            # correction > 0 หมายถึง ต้องเลี้ยวขวา (ห่างจากกำแพง)
            left_speed = BASE_SPEED - total_correction
            right_speed = BASE_SPEED + total_correction

        # จำกัดความเร็วสูงสุดและต่ำสุดเพื่อความเสถียร
        max_speed = BASE_SPEED * 1.5
        self.final_speed[LEFT] = max(-max_speed, min(left_speed, max_speed))
        self.final_speed[RIGHT] = max(-max_speed, min(right_speed, max_speed))

# --- Main Execution ---
if __name__ == "__main__":
    controller = WallFollower()
    controller.run()