from setuptools import setup

package_name = 'pi_teensy_udp'

setup(
    name=package_name,
    version='0.0.0',

    # === 1) Make sure this matches the directory under src/ exactly ===
    packages=[package_name],

    # === 2) Install the ament marker + package.xml + any launch files ===
    data_files=[
        # a) ament index “marker” (zero-byte file in resource/)
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # b) copy package.xml under share/pi_teensy_udp/
        ('share/' + package_name, ['package.xml']),
        # c) copy the launch file under share/pi_teensy_udp/launch/
        ('share/' + package_name + '/launch', ['launch/pi_teensy_udp.launch.py']),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS 2 package for UDP communication on Raspberry Pi 5',
    license='MIT',
    tests_require=['pytest'],

    entry_points={
        'console_scripts': [
            'udp_sender       = pi_teensy_udp.udp_sender:main',
            'udp_receiver     = pi_teensy_udp.udp_receiver:main',
            'pseudo_auto_mode = pi_teensy_udp.pseudo_auto_mode:main',
        ],
    },
)
