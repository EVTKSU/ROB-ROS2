from setuptools import setup

package_name = 'minirob_auto'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # Install our launch file into share/minirob_auto/launch
        ('share/' + package_name + '/launch', ['launch/minirob_auto.launch.py']),
        # Install the resource marker
        ('share/' + package_name, ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Self-contained minirob auto mode: OAK-D YOLOv8 segmentation → auto_commands',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'minirob_auto_node = minirob_auto.minirob_auto_node:main',
        ],
    },
)
