from setuptools import setup
import os
from glob import glob

package_name = 'charmie_carry_luggage'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ruisilva',
    maintainer_email='ruisilva@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "carry_my_luggage = charmie_carry_luggage.carry_my_luggage:main",
            "carry_my_luggage_robocup2023 = charmie_carry_luggage.carry_my_luggage_robocup2023:main",
        ],
    },
)
