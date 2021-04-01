from setuptools import setup

package_name = 'digitaltwin'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,'launch'), glob('*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='michaelkapteyn',
    maintainer_email='mkapteyn@mit.edu',
    description=' ROS 2 package that implements dynamic structural health monitoring for a UAV via a digital twin imbued with a probabilistic graphical model.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'asset = digitaltwin.asset:main',
            'twin = digitaltwin.twin:main',
            'logger = digitaltwin.logger:main',
        ],
    },
)
