from setuptools import find_packages, setup

package_name = 'yolo_detect'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config/model', [
            'config/model/yolo11.pt',
        ]),
        ('share/' + package_name + '/launch', [
        'launch/yolo_launch.py',
        ]),
    ],
    install_requires=[
        'setuptools',
        'opencv-python',
        'ultralytics',
        'cv_bridge',
        ],
    zip_safe=True,
    maintainer='macvmccw',
    maintainer_email='ooklibaioo@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detect_node = yolo_detect.yolo_detect_node:main',
            'bottle_detection_node = yolo_detect.bottle_detection_node:main',
            'bottle_navigation_node = yolo_detect.bottle_navigation_node:main',
            'midas_node = yolo_detect.midas_node:main',
            'bottle_position_node = yolo_detect.bottle_position_node:main',
            'bottle_nav2position_node = yolo_detect.bottle_nav2position_node:main',
        ],
    },
)
