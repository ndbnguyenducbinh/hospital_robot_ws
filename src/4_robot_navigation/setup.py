from setuptools import find_packages, setup

package_name = 'robot_navigation'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='binhnguyenduc',
    maintainer_email='binh.nguyen.e1@ctuav.vn',
    description='Nav2 integration, semantic location lookup, and mission task manager for Hospital Robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'task_manager_node = robot_navigation.task_manager_node:main',
        ],
    },
)

