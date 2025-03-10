from setuptools import find_packages, setup

package_name = 'dabakama_bot'

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
    maintainer='root',
    maintainer_email='aarongumba2016@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "joy_stick_analog=dabakama_bot.joy_stick_analog:main",
            "joystick_converter=dabakama_bot.joystick_converter:main",
            "wheelspeed=dabakama_bot.wheel_speed:main"
            
        ],
    },
)
