from setuptools import find_packages, setup

package_name = 'ros2_project_el22jy'

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
    maintainer='cscajb',
    maintainer_email='x.wang16@leeds.ac.uk',
    description='Exercises 1–4: colour detection and robot control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'first_step = ros2_project_el22jy.First_Step:main',
            'second_step = ros2_project_el22jy.Second_Step:main',
            'third_step = ros2_project_el22jy.Third_Step:main',
            'fourth_step = ros2_project_el22jy.Fourth_Step:main',
            'all_in_one = ros2_project_el22jy.All_in_One:main',
            'wasd_control = ros2_project_el22jy.wasd_control:main',
        ],
    },
)

