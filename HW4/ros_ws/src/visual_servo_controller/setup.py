from setuptools import setup

package_name = 'visual_servo_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ramana',
    maintainer_email='saikrn112@gmail.com',
    description='Visual Servo Controller for RR bot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
             'visual_servo_controller= visual_servo_controller.visual_servo:main',
        ],
    },
)
