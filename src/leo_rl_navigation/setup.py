from glob import glob

from setuptools import find_packages, setup

package_name = 'leo_rl_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy', 'gymnasium'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='khemeis.zribi@stud.tu-darmstadt.de',
    description='Headless Gymnasium navigation environment for the Leo Rover.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'leo_rl_env_smoke = leo_rl_navigation.env_smoke:main',
            'leo_rl_train = leo_rl_navigation.train_ppo:main',
            'leo_rl_evaluate = leo_rl_navigation.evaluate_policy:main',
        ],
    },
)
