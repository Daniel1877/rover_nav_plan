from setuptools import setup

package_name = 'launch_helpers'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Miro Voellmy',
    maintainer_email='miro.voellmy@esa.int',
    description='Contains helper scripts to be used in ROS2 python launch files.',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
