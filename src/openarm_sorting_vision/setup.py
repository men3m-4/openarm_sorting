from glob import glob

from setuptools import setup

package_name = 'openarm_sorting_vision'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='men3m',
    maintainer_email='muhammad.abdelmoniem4@gmail.com',
    description='RGB-D red/blue cube detection for the OpenArm cup-sorting simulation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cube_detector = openarm_sorting_vision.cube_detector:main',
            'detection_validator = openarm_sorting_vision.detection_validator:main',
        ],
    },
)
