from setuptools import setup

package_name = 'charmie_speakers'

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
    maintainer='utilizador',
    maintainer_email='tiagoribeiro80@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		"speakers = charmie_speakers.speakers:main",
		"speakers_with_save = charmie_speakers.speakers_with_save:main",
		"save_speaker_files = charmie_speakers.save_speaker_files:main"
        ],
    },
)
