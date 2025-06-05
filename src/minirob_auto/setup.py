from setuptools import setup

package_name = 'minirob_auto'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],

    # <<<<<<<<<<<<<<<< Add these three lines exactly as shown >>>>>>>>>>>>>>>>
    data_files=[
        # 1) ament marker
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # 2) your package.xml
        ('share/' + package_name, ['package.xml']),
        # 3) (optional) any launch files you might have
        ('share/' + package_name + '/launch', ['launch/minirob_auto.launch.py']),
    ],
    # <<<<<<<<<<<<<<<< end data_files >>>>>>>>>>>>>>>>

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS 2 package for mini‐robot autonomy',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'auto_control_node = minirob_auto.auto_control_node:main',
            'lane_detection_node = minirob_auto.lane_detection_node:main',
            'lane_traversal_node = minirob_auto.lane_traversal_node:main',
        ],
    },
)
