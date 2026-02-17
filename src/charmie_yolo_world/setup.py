from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'charmie_yolo_world'

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
    maintainer='tiago',
    maintainer_email='tiagoribeiro80@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        "yolo_world = charmie_yolo_world.yolo_world:main",
        "configure_visual_prompt_images = charmie_yolo_world.configure_visual_prompt_images:main",
        "get_images_for_visual_prompts = charmie_yolo_world.get_images_for_visual_prompts:main"
        ],
    },
)
