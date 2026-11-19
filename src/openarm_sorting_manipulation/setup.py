from glob import glob

from setuptools import setup

package_name = 'openarm_sorting_manipulation'

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
    description='MoveIt 2 pick-and-place coordinator for the OpenArm cup-sorting simulation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cup_sorting_coordinator = openarm_sorting_manipulation.coordinator:main',
            'task_prompt = openarm_sorting_manipulation.task_prompt:main',
            'check_cup_contents = openarm_sorting_manipulation.check_cup_contents:main',
        ],
    },
)
