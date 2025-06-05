from setuptools import setup

package_name = 'video_recorder'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        # Install our launch file into share/video_recorder/launch
        ('share/' + package_name + '/launch', ['launch/video_recorder.launch.py']),
        # Install the resource marker
        ('share/' + package_name, ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='OAK-D video recorder + real-time telemetry annotator',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'video_recorder_node = video_recorder.video_recorder_node:main',
            'frame_annotator_node = video_recorder.frame_annotator_node:main',
        ],
    },
)
