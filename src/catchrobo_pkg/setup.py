from setuptools import find_packages, setup

package_name = 'catchrobo_pkg'

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
    maintainer='altair',
    maintainer_email='106.nogu@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'tests': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'can_node = catchrobo_pkg.can_node:main',
            'pid_node = catchrobo_pkg.pid_node:main',
            'planning_node = catchrobo_pkg.planning_node:main',
            'web_socket_node = catchrobo_pkg.web_socket_node:main',
            'dualshock3_node = catchrobo_pkg.dualshock3_node:main',
        ],
    },
)
