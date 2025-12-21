from setuptools import find_packages, setup

package_name = 'ats_system1'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # (필요 시) launch/ 등 추가
        # ('share/' + package_name + '/launch', ['launch/xxx.launch.py']),
    ],
    install_requires=[
        'setuptools',
        'jsonschema>=4.0.0',   
    ],
    zip_safe=True,
    maintainer='geunpilpark',
    maintainer_email='engiman0401@gmail.com',
    description='System-1 executor for ATS (plan orchestration, Nav2, scan/track).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'system1_executor_node = ats_system1.nodes.system1_executor_node:main',
        ],
    },
)