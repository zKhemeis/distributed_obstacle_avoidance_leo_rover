from glob import glob

from setuptools import find_packages, setup


package_name = 'leo_heuristic_navigation'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='Khemeis Ala Zribi',
    maintainer_email='khemeis.zribi@stud.tu-darmstadt.de',
    description='Classical Dijkstra navigation baseline for Leo Rover.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'dijkstra_navigation = '
            'leo_heuristic_navigation.ros_dijkstra_node:main',
            'evaluate_dijkstra = '
            'leo_heuristic_navigation.evaluate_dijkstra:main',
        ],
    },
)
