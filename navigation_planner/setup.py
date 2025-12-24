from setuptools import setup

package_name = 'navigation_planner'

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
    description='Navigation planner for ROSA',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            # Already existing
            'navigation_planner = navigation_planner.planner:main',

            # New components
            'benchmark_coordinator_rosa = navigation_planner.benchmark_coordinator_rosa:main',
            'fake_action_executor = navigation_planner.fake_action_executor:main',
            'map_generator = navigation_planner.map_generator:main',

            # Launch file (optional if it has a main())
            'benchmark_fake_launch = navigation_planner.benchmark_fake.launch:main',
        ],
    },
)
