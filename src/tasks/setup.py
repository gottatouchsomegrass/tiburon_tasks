from setuptools import find_packages, setup

package_name = 'tasks'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dipankar',
    maintainer_email='dipankarghosh2006@gmail.com',
    description='ROS 2 tasks for turtlesim',
    license='TODO: License declaration',
    # This is the standard way to declare test dependencies in ROS 2
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # More descriptive names for your executables
            'star_drawer = tasks.q1:main',
            'spiral_drawer = tasks.q2:main',
            'shape_server = tasks.q3_server:main',
            'shape_client = tasks.q3_client:main',
            'param_spiral_drawer = tasks.q4:main',
        ],
    },
)

