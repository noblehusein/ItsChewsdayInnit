from setuptools import find_packages, setup

package_name = 'motion_controller'

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
    maintainer='root',
    maintainer_email='noblehusein6@gmail.com',
    description='Subscribes to target positions and publishes velocity commands',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motion_controller_node = motion_controller.motion_controller_node:main'
        ],
    },
)
