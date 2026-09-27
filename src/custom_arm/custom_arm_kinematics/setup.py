from setuptools import find_packages, setup

package_name = 'custom_arm_kinematics'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='atharva',
    maintainer_email='atharva@todo.todo',
    description='Kinematics, IK coordinate solver, and pick-and-place telemetry for 6-DOF custom robotic arm',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ik_coordinate_solver = custom_arm_kinematics.ik_coordinate_solver:main',
            'pick_and_place_telemetry = custom_arm_kinematics.pick_and_place_telemetry:main',
        ],
    },
)
