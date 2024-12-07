import os
from setuptools import setup, __version__ as setuptools_version
from packaging.version import Version
from glob import glob

package_name = 'mycobot_description'

use_dash_separated_options = Version(setuptools_version) < Version("58.0.0")


setup_cfg_content = """
[develop]
{script_option}=$base/lib/{package_name}

[install]
{install_scripts_option}=$base/lib/{package_name}
""".format(
    package_name=package_name,
    script_option='script-dir' if use_dash_separated_options else 'script_dir',
    install_scripts_option='install-scripts' if use_dash_separated_options else 'install_scripts'
)

with open("setup.cfg", "w") as f:
    f.write(setup_cfg_content)

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/obj'+'/xacro',glob("obj/xacro/*")),
        ('share/' + package_name + '/obj'+'/mesh',glob("obj/mesh/*")),
        ('share/' + package_name + '/obj'+'/urdf',glob("obj/urdf/*")),
        ('share/' + package_name + '/rviz' ,glob("rviz/*")),
        ('share/' + package_name + '/config' ,glob("config/*")),


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eric',
    maintainer_email='ceo.ekang@clawforhumanity.ca',
    description='mycobot_description pkg created by Elephant Robotics',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
