from distutils.core import setup

from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=[
        'kelpie_hardware_interface',
        'kelpie_hardware_interface.kelpie_input_interface',
        'kelpie_hardware_interface.kelpie_peripheral_interface',
        'kelpie_hardware_interface.kelpie_servo_interface'
    ],
    package_dir={'': 'src'}
)

setup(**d)
