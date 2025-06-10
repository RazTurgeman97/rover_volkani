from setuptools import find_packages, setup

package_name = 'experiment_manager'

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
    maintainer='user',
    maintainer_email='user@example.com',
    description='Lifecycle node orchestrating greenhouse experiments',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'experiment_manager = experiment_manager.experiment_manager_node:main',
        ],
    },
)
