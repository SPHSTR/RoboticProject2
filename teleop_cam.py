import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3 

def detect_black_regions(frame):
    height, width, _ = frame.shape
    section_height = height // 8  # แบ่งภาพออกเป็น 8 ส่วนแนวนอน
    centroids = []  # ลิสต์เก็บ centroid ที่พบ
    edges = []  # ลิสต์เก็บขอบซ้ายและขวา

    # กำหนดช่องตรงกลาง (3 ช่องในแนวนอน แต่ละช่องกว้างไม่เกิน 40px)
    middle_width = 40  # ความกว้างของช่องตรงกลางในแนวนอน
    center_x_min = (width // 2) - middle_width  # ขอบซ้ายของช่องตรงกลาง
    center_x_max = (width // 2) + middle_width  # ขอบขวาของช่องตรงกลาง

    linear_speed = [0.25,0.08,0.40]
    angular_speed = [0.3,0.5,0.2]

    # วาดกรอบช่องตรงกลาง
    cv2.rectangle(frame, (center_x_min, 0), (center_x_max, height), (0, 255, 255), 2)  # วาดกรอบช่องตรงกลางเป็นสีเหลือง

    # วาดจุด centroid ของพื้นที่สีดำ และจุดขอบซ้ายขวา
    for i in range(8):
        y_start = i * section_height
        y_end = (i + 1) * section_height if i < 7 else height  # ส่วนสุดท้ายอาจไม่พอดี
        section = frame[y_start:y_end, :]
        
        # แปลงเป็นขาวดำและหาหน้ากากสีดำ
        gray = cv2.cvtColor(section, cv2.COLOR_BGR2GRAY)
        _, black_mask = cv2.threshold(gray, 80, 100, cv2.THRESH_BINARY_INV)
        
        # หา contour ของพื้นที่สีดำ
        contours, _ = cv2.findContours(black_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(frame[y_start:y_end, :], contours, -1, (0, 255, 0), 2)  # วาดเส้นสีเขียว
        
        # หา centroid และขอบซ้าย-ขวาของพื้นที่สีดำ
        moments = cv2.moments(black_mask)
        if moments["m00"] != 0:
            cx = int(moments["m10"] / moments["m00"])
            cy = int(moments["m01"] / moments["m00"]) + y_start
            centroids.append((cx, cy))
            cv2.circle(frame, (cx, cy), 5, (255, 0, 0), -1)  # วาดจุดสีน้ำเงินที่ centroid
            cv2.putText(frame, f"{cy}", (cx + 10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)  # แสดงค่าตัวเลข
        
        # หาเส้นขอบซ้ายและขวา
        black_pixels = np.where(black_mask[0] == 255)[0]
        if len(black_pixels) > 0:
            left_x = black_pixels[0]
            right_x = black_pixels[-1]
            edges.append(((left_x, cy), (right_x, cy)))
            cv2.circle(frame, (left_x, cy), 5, (0, 255, 255), -1)  # จุดขอบซ้ายสีเหลือง
            cv2.circle(frame, (right_x, cy), 5, (0, 255, 255), -1)  # จุดขอบขวาสีเหลือง
            cv2.line(frame, (left_x, cy), (right_x, cy), (0, 255, 255), 2)  # วาดเส้นระยะห่างขอบซ้าย-ขวา

    # วาดจุดสีแดงด้านขวาของภาพ
    for _, cy in centroids:
        cv2.circle(frame, (width - 20, cy), 5, (0, 0, 255), -1)

    # คำนวณค่า x, y, z สำหรับ 3 จุดที่ใกล้ที่สุด
    Axis = [0, 0, 0]
    centroids_sorted = sorted(centroids, key=lambda c: (c[0] - width//2)**2 + (c[1] - height//2)**2)
    closest_centroids = centroids_sorted[:3]
    avg_x = np.mean([c[0] for c in closest_centroids])

    if len(centroids) >= 6:  # > 5 = walk       else   rotate
        # คำนวณค่า z
        if abs(avg_x - width // 2) <= 70:
            Axis[0] = 1 * linear_speed[2]
            if avg_x < width // 2:
                Axis[2] = -1 * angular_speed[2]
            else:
                Axis[2] = 1 * angular_speed[2]
        elif abs(avg_x - width // 2) <= 100:
            Axis[0] = 1 * linear_speed[0]
            if avg_x < width // 2:
                Axis[2] = -1 * angular_speed[0]
            else:
                Axis[2] = 1 * angular_speed[0]
        # < 80  mid     < 60 fast
    else :
        if abs(avg_x - width // 2) <= 30:
            Axis[0] = 1 * linear_speed[1]
            if avg_x < width // 2:
                Axis[2] = -1 * angular_speed[1]
            else:
                Axis[2] = 1 * angular_speed[1]

    # แสดงค่า x, y, z
    cv2.putText(frame, f"x = {Axis[0]}", (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.putText(frame, f"y = {Axis[1]}", (5, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.putText(frame, f"z = {Axis[2]}", (5, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

    return frame , Axis

class VideoTeleop(Node):
    def __init__(self):
        super().__init__('teleop_cam')
        
        self.cnt_vel_pub = self.create_publisher(Vector3, 'cnt_vel', 10)
        self.cap = cv2.VideoCapture(2)  # Open USB Camera

        if not self.cap.isOpened():
            self.get_logger().error("Error: Could not open camera")
            return

        self.timer = self.create_timer(0.05, self.timer_callback)  

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error("Failed to capture frame")
            return
        
        processed_frame, value = detect_black_regions(frame)

        self.publish_callback(value[0], value[1], value[2])

        cv2.imshow('Black Detection', processed_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.destroy_node()
            rclpy.shutdown()

    def publish_callback(self, x, y, theta):
        control_output = Vector3()
        control_output.x = float(x)
        control_output.y = float(y)
        control_output.z = float(theta)
        self.cnt_vel_pub.publish(control_output)

    def destroy_node(self):
        self.cap.release()
        cv2.destroyAllWindows()
        super().destroy_node()



def main(args=None):
    rclpy.init(args=args)
    node = VideoTeleop()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()