from setuptools import setup
import os
from glob import glob

package_name = 'pub_drone_detection2_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'drone_detection2.py = pub_drone_detection2_pkg.drone_detection2:main',
            'pid.py = pub_drone_detection2_pkg.pid:main',
            'vesc_twist_node_custom.py = pub_drone_detection2_pkg.vesc_twist_node_custom:main',
            'adafruit_servo_node_custom.py = pub_drone_detection2_pkg.adafruit_twist_node_custom:main',
        ],
    },
)



