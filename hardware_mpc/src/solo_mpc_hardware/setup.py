from glob import glob

from setuptools import find_packages, setup


setup(
    name='solo_mpc_hardware',
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/solo_mpc_hardware']),
        ('share/solo_mpc_hardware', ['package.xml', 'LICENSE-MPC', 'UPSTREAM.json']),
        ('share/solo_mpc_hardware/launch', glob('launch/*.launch.py')),
        ('share/solo_mpc_hardware/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Solo project team',
    maintainer_email='maintainer@example.invalid',
    description='Measured-state MPC trajectories and optional marker detection',
    license='MIT',
    entry_points={'console_scripts': [
        'controller = solo_mpc_hardware.controller_node:main',
        'marker_detector = solo_mpc_hardware.perception_node:main',
    ]},
)
