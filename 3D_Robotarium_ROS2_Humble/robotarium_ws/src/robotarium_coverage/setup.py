from setuptools import find_packages, setup

package_name = 'robotarium_coverage'

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
    maintainer='abhinav',
    maintainer_email='abhinav@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'pf_coverage = robotarium_coverage.pf_coverage:main',
        'sweep_coverage = robotarium_coverage.sweep_coverage:main',
        'frontier_coverage = robotarium_coverage.frontier_coverage:main',
        'voronoi_coverage = robotarium_coverage.voronoi_coverage:main',
        ],
    },
)
