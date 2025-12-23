from setuptools import find_packages, setup
package_name = 'carla_agent_controller'

setup(
    name=package_name,
    version='1.0.0',
    # packages=find_packages(exclude=['test']),
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name+ '/config', ['config/params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kazushifujimoto2',
    maintainer_email='kazushifujimoto2@tier4.jp',
    description='carla_agent_controller with Autoware',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'carla_agent_controller = carla_agent_controller.node:main',
        ],
    },
)
