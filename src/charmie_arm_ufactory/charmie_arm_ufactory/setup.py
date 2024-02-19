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
            "arm_ufactory = charmie_arm_ufactory.arm_ufactory:main",
            "arm_debug = charmie_arm_ufactory.arm_debug:main",
            "arm_cutlery = charmie_arm_ufactory.arm_cutlery:main"
        ],
    },
)
