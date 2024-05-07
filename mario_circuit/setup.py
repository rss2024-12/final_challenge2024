import os
import glob
from setuptools import setup

package_name = 'mario_circuit'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/'+package_name+"/computer_vision", glob.glob(os.path.join('mario_circuit/computer_vision', '*.py'))),
        ('share/mario_circuit/launch', glob.glob(os.path.join('launch', '*launch.xml'))),
        ('share/mario_circuit/launch', glob.glob(os.path.join('launch', '*launch.py')))
    ],  
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jospeh',
    maintainer_email='jrales@mit.edu',
    description='Visual Servoing ROS2 package',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driving_controller = mario_circuit.driving_controller:main',
            'line_detector = mario_circuit.line_detector:main',
            'homography_transformer = mario_circuit.homography_transformer:main',
            'homography_test = mario_circuit.homography_test:main',
            'cone_box_display = mario_circuit.cone_box_display:main',
            'mask_display = mario_circuit.mask_display:main'
        ],
    },
)
