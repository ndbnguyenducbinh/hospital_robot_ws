from setuptools import find_packages, setup

package_name = 'robot_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='binhnguyenduc',
    maintainer_email='binh.nguyen.e1@ctuav.vn',
    description='LaserScan filtering and depth pointcloud processing for Hospital Robot',
    license='MIT',
    entry_points={
        'console_scripts': [
        ],
    },
)

