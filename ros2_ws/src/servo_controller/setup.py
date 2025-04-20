from setuptools import setup

package_name = 'servo_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arjun',
    maintainer_email='arjun@todo.todo',
    description='Servo control package using ROS 2 and rclpy',
    license='MIT-0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_node = servo_controller.servo_node:main'
        ],
    },
)
