from setuptools import find_packages, setup

package_name = 'loadcell_ros2'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pi',
    maintainer_email='vjetstore.com@gmail.com',
    description='Loadcell node for RevPi A',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'loadcell_node = loadcell_ros2.loadcell_node:main',
        ],
    },
)
