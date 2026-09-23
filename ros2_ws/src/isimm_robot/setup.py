import os
from glob import glob
from setuptools import setup

package_name = 'isimm_robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'udev'), glob('udev/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Club Robotique ISIMM',
    maintainer_email='robot@isimm.tn',
    description='Robot autonome ISIMM : moteurs, capteurs, sécurité.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'motor_controller_node = isimm_robot.motor_controller_node:main',
            'esp32_sensor_node = isimm_robot.esp32_sensor_node:main',
            'safety_node = isimm_robot.safety_node:main',
            'robot_status_node = isimm_robot.robot_status_node:main',
        ],
    },
)
