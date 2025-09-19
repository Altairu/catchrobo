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
            'gamepad_node = catchrobo_pkg.gamepad_node:main',
            'serial_can_gui_node = catchrobo_pkg.serial_can_gui_node:main',
            'camera_node = catchrobo_pkg.camera_node:main',
        ],

    },
)
