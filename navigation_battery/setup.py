from setuptools import setup

package_name = 'navigation_battery'

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
    maintainer='Mohamed Msallak',
    maintainer_email='you@example.com',
    description='Battery monitor for ROSA',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'battery_monitor = navigation_battery.battery_monitor:main',
        ],
    },
)
