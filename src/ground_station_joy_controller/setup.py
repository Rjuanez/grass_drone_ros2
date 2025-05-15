from setuptools import find_packages, setup

package_name = 'ground_station_joy_controller'

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
    maintainer='ruben',
    maintainer_email='ruben.juanez@estudiantat.upc.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'joy_controller = ground_station_joy_controller.joy_controller_node:main',
          'joy_reader_mac = ground_station_joy_controller.joy_reader_mac_m1_node:main',
        ],
    },
)
