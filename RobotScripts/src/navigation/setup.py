from setuptools import find_packages, setup

package_name = 'navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial', 'pynmea2', 'firebase-admin'],
    zip_safe=True,
    maintainer='patrick',
    maintainer_email='patrick.doolittle@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_node = navigation.NavigationNode:main',
            'gps_node = navigation.gpsNode:main',
            'firestore_waypoint_listener_node = navigation.firestoreWaypointListener:main',
            'obstacle_avoidance = navigation.obstacle_avoidance:main',
            'geofence_node = navigation.geofenceControllerNode:main',
        ],
    },
)
