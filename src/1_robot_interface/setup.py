from setuptools import find_packages, setup

package_name = 'robot_interface'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'fastapi', 'uvicorn', 'pydantic'],
    zip_safe=True,
    maintainer='binhnguyenduc',
    maintainer_email='binh.nguyen.e1@ctuav.vn',
    description='Gateway between external Tablet App and ROS 2 navigation stack',
    license='MIT',
    entry_points={
        'console_scripts': [
            'tablet_bridge_node = robot_interface.tablet_bridge_node:main',
        ],
    },
)

