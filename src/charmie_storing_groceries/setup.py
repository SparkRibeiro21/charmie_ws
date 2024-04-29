from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'charmie_storing_groceries'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
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
		"storing_groceries_v0 = charmie_storing_groceries.storing_groceries_v0:main",
        "storing_groceries_v0_with_open_wardrobe = charmie_storing_groceries.storing_groceries_v0_with_open_wardrobe:main"
        ],
    },
)
