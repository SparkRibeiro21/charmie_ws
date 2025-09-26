from setuptools import setup
import os
from glob import glob

package_name = 'charmie_debug'

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
            "calibrate_door_handle = charmie_debug.calibrate_door_handle:main",
            "debug_arm = charmie_debug.debug_arm:main",
            "debug_audio = charmie_debug.debug_audio:main",
            "debug_detect_distance_door = charmie_debug.debug_detect_distance_door:main",
            "debug_face_recognition = charmie_debug.debug_face_recognition:main",
            "debug_inspection_person_detection = charmie_debug.debug_inspection_person_detection:main",
            "debug_llm = charmie_debug.debug_llm:main",
            "debug_navigation_localization = charmie_debug.debug_navigation_localization:main",
            "debug_pick_front = charmie_debug.debug_pick_front:main",
            "debug_pick_object_depth = charmie_debug.debug_pick_object_depth:main",
            "debug_pick_top = charmie_debug.debug_pick_top:main",
            "debug_search_for_person = charmie_debug.debug_search_for_person:main",
            "debug_sound_classification = charmie_debug.debug_sound_classification:main",
		    "debug_spkr_face = charmie_debug.debug_spkr_face:main",
		    "debug_visual = charmie_debug.debug_visual:main",
            "nav2_actions_client = charmie_debug.nav2_actions_client:main",
            "node_template = charmie_debug.node_template:main",
            "rosbag_2_video_converter = charmie_debug.rosbag_2_video_converter:main",
            "save_video_for_dataset = charmie_debug.save_video_for_dataset:main",
		    "task_template = charmie_debug.task_template:main",
		    "test_actions_client = charmie_debug.test_actions_client:main",
		    "test_actions_server = charmie_debug.test_actions_server:main"
        ],
    },
)
