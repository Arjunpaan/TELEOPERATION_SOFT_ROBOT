import rclpy
from rclpy.node import Node
import time
import math
import pigpio

SERVO_PINS = [17, 18, 27, 22, 23, 24]  # BCM pins for 6 servos
MIN_PULSE = 500
MAX_PULSE = 2500

# Mechanical properties
rd = 10.0 / 1000.0  # disk radius in meters
rp = 20.0 / 1000.0  # pulley radius in meters

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        self.pi = pigpio.pi()
        if not self.pi.connected:
            self.get_logger().error("Could not connect to pigpio. Start it with: sudo pigpiod")
            exit(1)

        for pin in SERVO_PINS:
            self.pi.set_mode(pin, pigpio.OUTPUT)

        self.command_file = '/home/arjun/flask_command_logger/received_commands.txt'
        self.last_line = ""
        self.timer = self.create_timer(2.0, self.control_servos)

    def control_servos(self):
        try:
            with open(self.command_file, 'r') as file:
                lines = file.readlines()
                if not lines:
                    return
                last = lines[-1].strip()
                if last == self.last_line:
                    return
                self.last_line = last

                if 'alpha=' in last and 'theta=' in last:
                    parts = last.split('Command:')[-1].strip()
                    alpha_str, theta_str = parts.split(',')
                    alpha_deg = float(alpha_str.replace('alpha=', '').strip())
                    theta_deg = float(theta_str.replace('theta=', '').strip())

                    alpha = math.radians(alpha_deg)
                    theta = math.radians(theta_deg)

                    self.get_logger().info(f"Received α={alpha_deg}°, θ={theta_deg}°")

                    self.run_inverse_kinematics(alpha, theta)
                else:
                    self.get_logger().warn("Invalid format. Use: alpha=VALUE,theta=VALUE")
        except Exception as e:
            self.get_logger().error(f"File read error: {str(e)}")

    def run_inverse_kinematics(self, alpha, theta):
        delta_Lv = rd * theta

        # Real tendon length changes
        delta_L = [
            math.cos(alpha) * delta_Lv,
            math.cos((2 * math.pi / 3) - alpha) * delta_Lv,
            math.cos((4 * math.pi / 3) - alpha) * delta_Lv
        ]

        servo_angles = [(delta / (2 * math.pi * rp)) * 360.0 for delta in delta_L]

        for i in range(3):
            angle = max(0, min(180, 90 + servo_angles[i]))
            self.move_slowly(SERVO_PINS[i], angle)
            self.move_slowly(SERVO_PINS[i + 3], angle)

    def angle_to_pulse(self, angle):
        return int(MIN_PULSE + (MAX_PULSE - MIN_PULSE) * (angle / 180.0))

    def move_slowly(self, pin, target_angle, delay=0.03):
        current_angle = 90
        step = 1 if target_angle > current_angle else -1
        for angle in range(int(current_angle), int(target_angle) + step, step):
            pulse = self.angle_to_pulse(angle)
            self.pi.set_servo_pulsewidth(pin, pulse)
            time.sleep(delay)

    def destroy_node(self):
        for pin in SERVO_PINS:
            self.pi.set_servo_pulsewidth(pin, 0)
        self.pi.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ServoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
