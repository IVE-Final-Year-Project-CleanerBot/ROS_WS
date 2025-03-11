from setuptools import find_packages, setup

package_name = 'video_streamer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/video_streamer_launch.py']),
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
            'video_stream = video_streamer.video_stream:main'
        ],
    },
)
