from setuptools import find_packages, setup

package_name = 'driver_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/driver.launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kencheng',
    maintainer_email='ooklibaioo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'driver_node = driver_ros2.driver_node:main',
            'bottle_pickup_node = driver_ros2.bottle_pickup_node:main',
            'bottle_place_node = driver_ros2.bottle_place_node:main',
        ],
    },
)
