from setuptools import setup

package_name = 'charmie_arm_ufactory'

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
    maintainer='charmie',
    maintainer_email='charmie@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "arm = charmie_arm_ufactory.arm:main",
            "arm_demonstration = charmie_arm_ufactory.arm_demonstration:main",
            "arm_open_doors = charmie_arm_ufactory.arm_open_doors:main",
            "arm_debug = charmie_arm_ufactory.arm_debug:main",
            "arm_cutlery = charmie_arm_ufactory.arm_cutlery:main",
            "arm_hello = charmie_arm_ufactory.arm_hello:main",
            "arm_pick_place_all_objects = charmie_arm_ufactory.arm_pick_place_all_objects:main",
            "arm_serve_breakfast = charmie_arm_ufactory.arm_serve_breakfast:main",
            "arm_storing_groceries = charmie_arm_ufactory.arm_storing_groceries:main",
            "arm_carry_my_luggage = charmie_arm_ufactory.arm_carry_my_luggage:main",
            "arm_receptionist = charmie_arm_ufactory.arm_receptionist:main",
            "arm_clean_table = charmie_arm_ufactory.arm_clean_table:main",

        ],
    },
)
