from setuptools import find_packages, setup

package_name = 'amr_hardware'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/hardware_mirror.launch.py']),
        ('share/' + package_name + '/config', ['config/hardware.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='akshay',
    maintainer_email='you@example.com',
    description='Real-robot drivers (Dynamixel drive, BNO085 IMU, HC-SR04 sonar) that mirror the Webots sim.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dynamixel_drive = amr_hardware.dynamixel_drive:main',
            'imu_node = amr_hardware.imu_node:main',
            'ultrasonic_node = amr_hardware.ultrasonic_node:main',
        ],
    },
)
