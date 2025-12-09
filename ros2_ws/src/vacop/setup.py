from setuptools import find_packages, setup

package_name = 'vacop'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/robot_launch.py']),
        ('share/' + package_name + '/resource', ['resource/my_robot.urdf']),
        ('share/' + package_name + '/config', ['config/controllers.yaml']),
        ('share/' + package_name + '/worlds', [
            'worlds/village.wbt',
            'worlds/vacop_body.obj'
            ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeanbaptiste',
    maintainer_email='jeanbaptiste@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'teleop = vacop.teleop_vacop:main',
            'camera_view = vacop.camera_display:main',
        ],
    },
)
