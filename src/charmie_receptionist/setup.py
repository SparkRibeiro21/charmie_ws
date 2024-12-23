from setuptools import setup
import os
from glob import glob

package_name = 'charmie_receptionist'

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
    maintainer='utilizador',
    maintainer_email='tiagoribeiro80@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "receptionist = charmie_receptionist.receptionist:main",
            "receptionist_robocup2023 = charmie_receptionist.receptionist_robocup2023:main"
        ],
    },
)
