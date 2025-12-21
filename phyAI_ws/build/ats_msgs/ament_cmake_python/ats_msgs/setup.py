from setuptools import find_packages
from setuptools import setup

setup(
    name='ats_msgs',
    version='0.0.1',
    packages=find_packages(
        include=('ats_msgs', 'ats_msgs.*')),
)
