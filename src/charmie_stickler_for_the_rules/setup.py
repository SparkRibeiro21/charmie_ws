from setuptools import setup
import os
from glob import glob

package_name = 'charmie_stickler_for_the_rules'

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
    maintainer='bruno',
    maintainer_email='bruno@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             "stickler_for_the_rules = charmie_stickler_for_the_rules.stickler_for_the_rules:main",
             "stickler_for_the_rules_qualification_24 = charmie_stickler_for_the_rules.stickler_for_the_rules_qualification_24:main"
        ],
    },
)
