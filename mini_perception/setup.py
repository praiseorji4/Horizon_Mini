from setuptools import find_packages, setup

package_name = 'mini_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mini',
    maintainer_email='todo@todo.com',
    description='YOLO-based object detection node for the mini robot.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'yolo_detector = mini_perception.yolo_detector:main',
        ],
    },
)
