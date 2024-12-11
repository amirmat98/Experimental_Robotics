from setuptools import setup

package_name = 'ros2_aruco'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AmirMahdi Matin',
    maintainer_email='5884715@studenti.unige.it',
    description='Package description',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_node = ros2_aruco.aruco_node:main',
            'aruco_generate_marker = ros2_aruco.aruco_generate_marker:main',
            'marker_detector = ros2_aruco.marker_detector:main',
            'camera_marker_detector = ros2_aruco.camera_marker_detector:main'
        ],
    },
)
