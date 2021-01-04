from setuptools import setup
from glob import glob

package_name = 'path_score'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),  # Empty file to stop colcon complaints

        ('share/' + package_name, glob('launch/*.launch.py')),

        ('share/' + package_name, glob('rviz/*.rviz')),

        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='suraj',
    maintainer_email='suraj.rathi00@gmail.com',
    description='TODO: Package description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = path_score.main:main',
        ],
    },
)
