from setuptools import setup

package_name = 'pi_teensy_udp'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pi_teensy_udp.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS 2 package for UDP communication on Raspberry Pi 5',
    license='MIT',
    tests_require=['pytest'],
    entry_points={ 'console_scripts': [ 'udp_sender = pi_teensy_udp.udp_sender:main', 'udp_receiver = pi_teensy_udp.udp_receiver:main', 'pseudo_auto_mode = pi_teensy_udp.pseudo_auto_mode:main', ], },

)
