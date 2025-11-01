from setuptools import find_packages, setup

package_name = 'usb_cams'

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
    maintainer='mike',
    maintainer_email='michael.lythgoe@spaceconcordia.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'img_pub_ = usb_cams.image_publisher:main',
            'img_sub = usb_cams.image_subscriber:main',
            'yolo_pub = usb_cams.yolov8_ros2_pub:main',
            'yolo_sub = usb_cams.yolov8_ros2_sub:main'
        ],
    },
)
