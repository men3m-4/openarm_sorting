from glob import glob

from setuptools import setup

package_name = 'visual_manipulation_tasks'

setup(
    name=package_name,
    version='0.1.0',
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
    description='Vision-guided manipulation task coordinators (portafilter install / remove)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'portafilter_task = visual_manipulation_tasks.portafilter_task:main',
            'task_prompt = visual_manipulation_tasks.task_prompt:main',
        ],
    },
)
