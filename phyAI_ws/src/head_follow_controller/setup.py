from setuptools import setup
package_name = 'head_follow_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],  # ← 디렉터리명과 동일해야 함
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch로 실행할 거면 이 줄 필요
        ('share/' + package_name + '/launch', ['launch/head_follow.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Move Spot body to align with ATS camera heading (TF body->Camera).',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'head_follow_controller = head_follow_controller.head_follow_controller:main',
        ],
    },
)
