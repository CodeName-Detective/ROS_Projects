from setuptools import setup
import os
from glob import glob
package_name = 'ros2_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), ['config/parameters.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_executable = ros2_demo.publisher:main',
            'subscriber_executable = ros2_demo.subscriber:main',
            'server_executable = ros2_demo.server:main',
            'client_executable = ros2_demo.client_async:main',
            'action_server_executable = ros2_demo.action_server:main',
            'action_client_executable = ros2_demo.action_client:main'
        ],
    },
)
