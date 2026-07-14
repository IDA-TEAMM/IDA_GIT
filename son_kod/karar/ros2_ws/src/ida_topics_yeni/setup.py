from setuptools import setup
 
package_name = 'ida_topics'
 
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='IDA Takımı',
    maintainer_email='ida@example.com',
    description='IDA sistem topic nodeları',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'sensor_node = ida_topics.sensor_node:main',
            'perception_node = ida_topics.perception_node:main',
            'control_node = ida_topics.control_node:main',
            'decision_node = ida_topics.decision_node:main',
            # Sürücüler (son_kod entegrasyonu 2026-07-14 — ros2 run için)
            'livox_driver_node = ida_topics.livox_driver_node:main',
            'oakd_driver_node = ida_topics.oakd_driver_node:main',
            'gps_imu_driver_node = ida_topics.gps_imu_driver_node:main',
            'kamera_kayit_node = ida_topics.kamera_kayit_node:main',
            'telemetri_node = ida_topics.telemetri_node:main',
            'local_map_node = ida_topics.local_map_node:main',
        ],
    },
)
 
