from setuptools import setup

package_name = 'charmie_point_cloud'

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
		"point_cloud = charmie_point_cloud.point_cloud:main",
		"save_images_test_point_cloud = charmie_point_cloud.save_images_test_point_cloud:main"
        ],
    },
)
