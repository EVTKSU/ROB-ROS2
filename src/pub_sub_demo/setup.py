from setuptools import find_packages, setup  # Standard Python packaging tools

# The name of your package (must match the folder name containing your Python code)
package_name = 'pub_sub_demo'

# This function sets up your ROS 2 Python package using setuptools
setup(
    name=package_name,               # The package name used in ROS tooling
    version='0.0.0',                 # Version of your package
    packages=find_packages(exclude=['test']),  # Find all Python packages except test/

    # These data files are necessary for ROS indexing and launch file installation
    data_files=[
        # Register your package with the ROS 2 ament index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Ensure package.xml is installed with the package
        ('share/' + package_name, ['package.xml']),
        # Ensure launch file gets installed to the correct location
        ('share/' + package_name + '/launch', ['launch/pub_sub_demo.launch.py']),
    ],

    install_requires=['setuptools'],  # Dependencies required at installation time
    zip_safe=True,                    # True means the package can be used as a .zip archive

    # Maintainer metadata
    maintainer='colin',
    maintainer_email='colinhaskins2022@gmail.com',
    description='Minimal pub/sub demo for ROS 2 Jazzy',
    license='MIT',

    # Needed if you add tests using pytest
    tests_require=['pytest'],

    # Command-line tools (what you can run with `ros2 run`)
    entry_points={
        'console_scripts': [
            'talker = pub_sub_demo.talker:main',
            'listener = pub_sub_demo.listener:main',
        ],
    },
)
