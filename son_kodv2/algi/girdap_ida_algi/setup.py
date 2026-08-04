from setuptools import setup

package_name = 'girdap_ida_algi'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/algi.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team GIRDAP',
    maintainer_email='girdap@example.com',
    description='GIRDAP IDA - duba algilama ve gecit gorev mantigi (TEKNOFEST 2026)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'duba_gecis_navigator = girdap_ida_algi.duba_gecis_navigator:main',
        ],
    },
)
