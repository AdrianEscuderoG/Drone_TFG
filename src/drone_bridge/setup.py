from setuptools import find_packages, setup

package_name = 'drone_bridge'

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
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'receiver_node_ardu = drone_bridge.receiver_node_ardu:main',
            'emitter_node = drone_bridge.emitter_node:main',
            'image_receiver_node = drone_bridge.image_receiver_node:main',
            'calib_recorder_node = drone_bridge.calib_recorder_node:main',
        ],
    },
)
