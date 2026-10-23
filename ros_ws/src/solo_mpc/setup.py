from glob import glob
from setuptools import find_packages, setup

setup(
    name='solo_mpc', version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/solo_mpc']),
        ('share/solo_mpc', ['package.xml', 'LICENSE-MPC', 'UPSTREAM.json']),
        ('share/solo_mpc/launch', glob('launch/*.launch.py')),
        ('share/solo_mpc/config', glob('config/*.yaml')),
        ('share/solo_mpc/scenarios', glob('scenarios/*.yaml')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Solo project team', maintainer_email='maintainer@example.invalid',
    description='Modular multi-UAV target tracking MPC with Gazebo perception',
    license='MIT',
    entry_points={'console_scripts': [
        'controller = solo_mpc.controller_node:main',
        'setpoints = solo_mpc.setpoint_node:main',
        'frames = solo_mpc.frame_node:main',
        'marker_detector = solo_mpc.perception_node:main',
        'target = solo_mpc.target_node:main',
        'mission = solo_mpc.mission:main',
        'mission_supervisor = solo_mpc.mission_node:main',
        'evaluate = solo_mpc.evaluate:main',
    ]},
)
