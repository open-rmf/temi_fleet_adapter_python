from setuptools import setup

package_name = 'temi_fleet_adapter_python'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cnboonhan',
    maintainer_email='charayaphan.nakorn.boon.han@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "full_control_fleet_adapter=temi_fleet_adapter_python.full_control_fleet_adapter:main",
            "bridge_websocket_server=temi_fleet_adapter_python.bridge_websocket_server:main"
        ],
    },
)
