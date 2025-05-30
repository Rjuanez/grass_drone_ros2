from setuptools import find_packages, setup

package_name = 'grass_drone_camera'

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
    maintainer_email='ruben@juanez.name',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "aruco_detection_node = grass_drone_camera.simple_aruco:main",
            "get_images_node = grass_drone_camera.get_images:main"
            "send_images = grass_drone_camera.send_images:main"
        ],
    },
)
