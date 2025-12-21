from setuptools import setup

package_name = 'ats_keyboard_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Keyboard node to publish track/cancel PlanCommand to /system2/plan_cmd',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'keyboard_control = ats_keyboard_control.keyboard_control_node:main',
        ],
    },
)
