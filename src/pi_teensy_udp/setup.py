from setuptools import setup

package_name = 'minirob_auto'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # 1) install an ament marker so colcon-ros can index this package
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # 2) install package.xml under share/<package_name>/
        ('share/' + package_name, ['package.xml']),
        # 3) (optional) if you have launch files, uncomment and adjust:
        # ('share/' + package_name + '/launch', ['launch/<your_launch_file>.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS 2 package for … (brief description of minirob_auto)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Uncomment and edit the following lines to expose any Python nodes:
            # 'node_name = minirob_auto.node_script:main',
        ],
    },
)
