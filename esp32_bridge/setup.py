from setuptools import find_packages, setup

package_name = 'esp32_bridge'

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
    maintainer='botcanh',
    maintainer_email='pkhoi1809@gmail.com',
    description='Serial bridge between ROS2 and ESP32 for differential-drive robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bridge_node = esp32_bridge.bridge_node:main',
        ],
    },
)
