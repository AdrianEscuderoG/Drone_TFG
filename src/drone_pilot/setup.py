from setuptools import find_packages, setup

package_name = 'drone_pilot'

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
    maintainer='adrianeg',
    maintainer_email='aescgar3@upv.edu.es',
    description='Piloto autónomo: máquina de estados, PID y generador de misiones.',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'pilot_node = drone_pilot.pilot_node:main',
            'mission_generator = drone_pilot.mission_generator:main',
        ],
    },
)
