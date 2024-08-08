from setuptools import find_packages, setup
from glob import glob

package_name = 'pathplanner_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/pathplanner/paths', glob('pathplanner/paths/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='henrylec',
    maintainer_email='henry@lunar-glass.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'action_builder = pathplanner_ros.main:main'
        ],
    },
)
