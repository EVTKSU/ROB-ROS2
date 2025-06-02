from setuptools import setup

package_name = 'minirob_auto'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # 1) The three launch files bundle into share/minirob_auto/launch
        ('share/' + package_name + '/launch', [
            'launch/minirob_auto_launch.py',
        ]),
        # 2) The marker file
        ('share/' + package_name, ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Three‑node split for OAK-D YOLOv8 → steer_pct → percent_cmds → UDP/raw /auto_commands',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            # Each of our three nodes
            'lane_detection_node = minirob_auto.lane_detection_node:main',
            'lane_traversal_node = minirob_auto.lane_traversal_node:main',
            'auto_control_node = minirob_auto.auto_control_node:main',
        ],
    },
)
