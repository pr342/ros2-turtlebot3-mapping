from setuptools import setup
import os
from glob import glob

package_name = 'my_doosan_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

        # ✅ INSTALL URDF FILES
        (os.path.join('share', package_name, 'description', 'urdf'),
         glob('description/urdf/*.urdf')),

        # ✅ INSTALL MESH FILES CORRECTLY
        (os.path.join('share', package_name, 'description', 'meshes', 'm1013_white'),
         glob('description/meshes/m1013_white/*.dae')),
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='protima',
    maintainer_email='protima@todo.todo',
    description='Doosan M1013 description package',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)

