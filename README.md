# Flexible Tendon-Driven Continuum Manipulator for Teleoperation using ROS

## Project Overview
This project presents a teleoperated soft robotic arm miming biological manipulators like tentacles and trunks using a tendon-driven flexible continuum manipulator. Built on a Raspberry Pi 4 and controlled via a Wi-Fi-enabled web interface, this system leverages ROS 2 Humble, inverse kinematics, and PWM servo control to achieve 3D directional bending.

##So below the ros 2 node complete code is available in the ros2 workbench here basic overview 
flask web code basic overview setup overview how to control everything and installation guide, and mathematics behind the algorithm is available on the below research paper by Firdaus Bhaiya

Md Modassir Firdaus and Madhu Vadali. 2023. Virtual Tendon-Based Inverse Kinematics of Tendon-Driven Flexible Continuum Manipulators. In
Advances In Robotics - 6th International Conference of The Robotics Society
(AIR 2023), July 05–08, 2023, Ropar, India. ACM, New York, NY, USA, 5 pages.
https://doi.org/10.1145/3610419.3610491  

---

##  Step-by-Step Implementation

### 1.  **Processor Selection: Raspberry Pi 4 (RPI4)**
- **Reason**: ROS 2 Humble requires Ubuntu 22.04 LTS, which is compatible with RPi 4.

### 2.  **Installing Ubuntu 22.04 LTS on Raspberry Pi 4**
- **Tool Used**: Raspberry Pi Imager or balenaEtcher
- **Steps**:
  - Download the 64-bit Ubuntu 22.04 LTS image.
  - Flash the image onto a microSD card.
  - Insert it into the Pi and power it up.
- **Initial Terminal Commands**:
  ```bash
  sudo apt update && sudo apt upgrade -y
  ```
  > Ensures the system is updated to the latest security patches and package versions.

### 3.  **Installing ROS 2 Humble on RPi 4**
- Followed the official installation guide for ROS 2 Humble on Ubuntu 22.04.


  **Step 1: Set Locale**
  ```bash
  sudo apt update && sudo apt install locales
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
  export LANG=en_US.UTF-8
  ```

  **Step 2: Add ROS 2 GPG Key**
  ```bash
  sudo apt install curl gnupg lsb-release
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
  ```

  **Step 3: Add ROS 2 Repo to apt sources**
  ```bash
  echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  ```

  **Step 4: Install ROS 2 Humble Desktop**
  ```bash
  sudo apt update
  sudo apt install ros-humble-desktop
  ```

  **Step 5: Source ROS setup**
  ```bash
  echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
  source ~/.bashrc
  ```

  **Step 6: Install ROS 2 Dependencies**
  ```bash
  sudo apt install python3-colcon-common-extensions python3-pip python3-rosdep
  sudo rosdep init
  rosdep update
  ```

---

##  Installing Required Libraries

### Python Libraries
```bash
pip3 install flask numpy pigpio
```

### GPIO and PWM Control Setup
```bash
sudo apt install pigpio python3-pigpio
sudo systemctl enable pigpiod
sudo systemctl start pigpiod
```
> `pigpiod` daemon must be running to send PWM signals to servos.

---
Physical Setup & Mechanical Design
3D Printed Pulleys:

Designed in Fusion 360 with a 20 mm radius

Mounted on MG995 servo horns using M3 screws

Servo Mounting Box:

Laser cut acrylic or 3D printed enclosure for servo mounting

Secured with standoffs and brackets

Nylon Tube Backbone:

Length: ~175 mm, Diameter: 8 mm

Flexible yet durable to allow bending without permanent deformation

Aluminum or PLA Disks:

Outer diameter: ~10 mm

Holes drilled at 120° apart for tendon routing

Placed at regular intervals (every 15 mm)

Tendon Routing:

Tendons inserted through front disk and passed through holes in disks

Anchored to pulleys which rotate and change tendon length

Camera Mounting:

Small USB/PI camera placed inside or under the belly of the manipulator

Connected to Pi for real-time observation

Power Supply:

12V 5A SMPS connected to buck converter

Voltage regulated to 6V and supplied to servo rail

Raspberry Pi powered independently or via USB-C

Mechanical Support Frame:

Base made from MDF or acrylic

Holds the servo box and manipulator securely

## ROS 2 Humble Node for Servo Control

### Package Structure
- `my_flexible_arm/`
  - `package.xml`
  - `CMakeLists.txt`
  - `launch/`
  - `scripts/`
    - `servo_controller.py`

### servo_controller.py
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import pigpio

SERVO_PINS = [17, 18, 27, 22, 23, 24]  # BCM GPIO pins
RP = 20.0  # Pulley radius in mm

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_controller')
        self.subscriber = self.create_subscription(
            Float32MultiArray,
            'flexible_arm_input',
            self.callback,
            10
        )
        self.pi = pigpio.pi()

    def callback(self, msg):
        alpha, theta = msg.data
        rd = 10.0  # disk radius
        delta_lv = rd * theta
        delta_l = [
            delta_lv * math.cos(alpha),
            delta_lv * math.cos((2 * math.pi / 3) - alpha),
            delta_lv * math.cos((4 * math.pi / 3) - alpha)
        ]
        servo_angles = [int((d / RP) * 1800 / math.pi) for d in delta_l]  # Convert to microseconds

        for i, pin in enumerate(SERVO_PINS):
            self.pi.set_servo_pulsewidth(pin, 1500 + servo_angles[i % 3])

rclpy.init()
node = ServoController()
rclpy.spin(node)
rclpy.shutdown()
```

### Terminal Commands to Build and Run:
```bash
cd ~/ros2_ws
colcon build --packages-select my_flexible_arm
source install/setup.bash
ros2 run my_flexible_arm servo_controller
```

---
Flask Web Interface for Teleoperation
python
Copy
Edit
from flask import Flask, request
import rclpy
from std_msgs.msg import Float32MultiArray

app = Flask(__name__)
rclpy.init()
node = rclpy.create_node('web_interface')
publisher = node.create_publisher(Float32MultiArray, 'flexible_arm_input', 10)

@app.route('/', methods=['POST'])
def control():
    alpha = float(request.form['alpha'])
    theta = float(request.form['theta'])
    msg = Float32MultiArray()
    msg.data = [alpha, theta]
    publisher.publish(msg)
    return 'Command sent.'

app.run(host='0.0.0.0', port=5000)
📐 Inverse Kinematics — Deep Dive
Virtual Tendon Model
𝛿
𝐿
𝑣
=
𝑟
𝑑
⋅
𝜃
δL 
v
​
 =r 
d
​
 ⋅θ

Mapping to tendons:

𝛿
𝐿
𝑎
=
cos
⁡
(
𝛼
)
⋅
𝛿
𝐿
𝑣
𝛿
𝐿
𝑏
=
cos
⁡
(
2
𝜋
3
−
𝛼
)
⋅
𝛿
𝐿
𝑣
𝛿
𝐿
𝑐
=
cos
⁡
(
4
𝜋
3
−
𝛼
)
⋅
𝛿
𝐿
𝑣
δL 
a
​
 =cos(α)⋅δL 
v
​
 
δL 
b
​
 =cos( 
3
2π
​
 −α)⋅δL 
v
​
 
δL 
c
​
 =cos( 
3
4π
​
 −α)⋅δL 
v
​
 
Conversion to PWM:

Servo Angle
=
𝛿
𝐿
𝑟
𝑝
Servo Angle= 
r 
p
​
 
δL
​


##  Web UI to ROS 2 Integration
- User inputs α (plane angle) and θ (bending angle)
- Flask sends data to ROS 2 node
- ROS 2 node computes and commands servos via GPIO

---

## Final Summary
- ROS 2 Humble is the **core framework** used for real-time teleoperation
- Python + Flask used for web-based user input
- `pigpio` handles PWM signals precisely to each servo
- Project integrates embedded systems, soft robotics, web technologies, and mathematical modeling for a real-world application

---
