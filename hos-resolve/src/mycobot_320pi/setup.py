import os
from setuptools import setup, __version__ as setuptools_version
from packaging.version import Version
from glob import glob

package_name = 'mycobot_320pi'

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
        (os.path.join('share', package_name, "launch"), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, "config"), glob('config/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='u2',
    maintainer_email='u2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_display = mycobot_320pi.camera_display:main',
            'cfh_command = mycobot_320pi.cfh_command:main',
            'cfh_relay = mycobot_320pi.cfh_relay:main',
            'detect_marker = mycobot_320pi.detect_marker:main',
            'follow_display = mycobot_320pi.follow_display:main',
            'following_marker = mycobot_320pi.following_marker:main',
            'listen_real_of_topic = mycobot_320pi.listen_real_of_topic:main',
            'listen_real = mycobot_320pi.listen_real:main',
            'opencv_camera = mycobot_320pi.opencv_camera:main',
            'simple_gui = mycobot_320pi.simple_gui:main',
            'slider_control = mycobot_320pi.slider_control:main',
            'teleop_keyboard = mycobot_320pi.teleop_keyboard:main',
        ],
    },
)
