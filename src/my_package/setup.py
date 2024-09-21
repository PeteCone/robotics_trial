from setuptools import find_packages, setup

package_name = 'my_package'

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
    maintainer='Peter',
    maintainer_email='peter.conant@sjsu.edu',
    description='Examples of minimal publisher/subscriber using rclpy',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #'my_node = my_package.my_node:main'
            'map = my_package.map_node:main',
            'goal_and_start = my_package.goal_and_start_node:main',
            'path = my_package.path_node:main'
        ],
    },
)
