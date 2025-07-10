from setuptools import find_packages, setup
package_name = 'uav_mission'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='matteus',
    maintainer_email='victormatteus@distente.ufg.br',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission = uav_mission.mission:main',
            'fase1 = uav_mission.fase1:main'
        ],
    },
)
