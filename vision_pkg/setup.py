from setuptools import find_packages, setup

package_name = 'vision_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools',  'rclpy', 'numpy', 'opencv-python', 'ultralytics'],
    zip_safe=True,
    maintainer='kamiab',
    maintainer_email='kamiab@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "image_publisher= vision_pkg.image_publisher_node:main",
            "image_subscriber= vision_pkg.image_subscriber_node:main",
            "obj_place_detector= vision_pkg.obj_place_detector_node:main"
        ],
    },
)
