from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'inspire_hand_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'urdf/meshes/visual'),  glob('urdf/meshes/visual/*.glb')),
        (os.path.join('share', package_name, 'urdf/meshes/collision'), glob('urdf/meshes/collision/*.obj')),
        # add launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Seunghyeok Back',
    maintainer_email='shback@kimm.re.kr',
    description='ROS Wrapper for Inspire Hand',
    requires=['pyserial', 'catkin_pkg', 'lark'],
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test = inspire_hand_driver.test:main',
            'hand_wrapper = inspire_hand_driver.hand_wrapper:main',
            'inspire_hand_driver = inspire_hand_driver.inspire_hand_driver:main',
            'test_open_close = inspire_hand_driver.test_open_close:main',
            'test_retargeting = inspire_hand_driver.test_retargeting:main',
            'cmd_from_vision_pro = inspire_hand_driver.cmd_from_vision_pro:main',
        ],
    },
)
