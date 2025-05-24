from setuptools import find_packages, setup

package_name = 'nav2_commander'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
        'launch/nav2_commander_launch.py',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='macvmccw',
    maintainer_email='ooklibaioo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'target_node = nav2_commander.target_node:main',
            'init_node = nav2_commander.init_node:main',
            'bottle_target_node = nav2_commander.bottle_target_node:main',
        ],
    },
)
