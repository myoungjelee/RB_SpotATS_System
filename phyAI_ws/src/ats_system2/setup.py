# ats_system2/setup.py
from setuptools import setup

package_name = 'ats_system2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name, package_name + '.utils'],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        # pip로 설치하지만, 있으면 좋음 (없어도 동작은 함)
        'langchain-openai',
        'langchain-core',
        'pydantic<3',
    ],
    zip_safe=True,
    maintainer='geunpil',
    maintainer_email='you@example.com',
    description='LLM-based System-2 planner for Spot+ATS as ROS2 node',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'system2_node = ats_system2.system2_node:main',
        ],
    },
)
