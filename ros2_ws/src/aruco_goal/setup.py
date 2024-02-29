from setuptools import find_packages, setup

package_name = 'aruco_goal'

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
    maintainer='computer',
    maintainer_email='camwolff02@gmail.com',
    description='TODO: Package description',
    license='Apache Lisence 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_goal_node = aruco_goal.aruco_goal_node:main',
        ],
    },
)
