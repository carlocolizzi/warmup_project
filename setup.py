from setuptools import setup

package_name = 'warmup_project'

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
    maintainer='carlo',
    maintainer_email='carlo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher = warmup_project.publisher:main',
            'subscriber = warmup_project.subscriber:main',
            'teleop = warmup_project.teleop:main',
            'wall_follower = warmup_project.wall_follower:main',
            'wall_follower_2 = warmup_project.wall_follower_copy:main',
            'drive_square = warmup_project.drive_square:main',
            'person_follower = warmup_project.person_follower:main',
            'obstacle_avoider = warmup_project.obstacle_avoider:main',
            'obstacle_avoider_2 = warmup_project.obstacle_avoider_2:main',
            'finite_state_controller = warmup_project.finite_state_controller:main'
        ],
    },
)
