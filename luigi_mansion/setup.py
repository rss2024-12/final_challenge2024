from setuptools import setup
import glob
import os

package_name = 'luigi_mansion'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/luigi_mansion/launch', glob.glob(os.path.join('launch', '*launch.*'))),
        # ('share/luigi_mansion/launch', glob.glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'params'), glob.glob('params/*.yaml')),
        ('share/luigi_mansion/lanes', glob.glob(os.path.join('lanes', '*.traj'))),
        ('lib/'+package_name+'/stop_detector_helpers',glob.glob(os.path.join('luigi_mansion/stop_detector_helpers', '*.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='racecar',
    maintainer_email='gonzaya2003a@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'city_drive = luigi_mansion.city_drive_node:main',
            'shell_drive = luigi_mansion.shell_drive_node:main',
            'basement_point_pub = luigi_mansion.basement_point_publisher:main',
            'stop_detector = luigi_mansion.stop_detector:main',
            'stoplight_detector = luigi_mansion.stoplight_detector:main',
        ],
    },
)
