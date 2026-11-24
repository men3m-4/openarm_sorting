from glob import glob

from setuptools import setup

package_name = 'visual_manipulation_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='men3m',
    maintainer_email='muhammad.abdelmoniem4@gmail.com',
    description='ArUco detection, marker pose estimation and object tracking',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detector = visual_manipulation_perception.aruco_detector_node:main',
            'fiducial_object_tracker = visual_manipulation_perception.object_tracker_node:main',
        ],
    },
)
