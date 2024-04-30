from setuptools import setup, find_packages

setup(
    name='pyRobotiqGripper',
    version='1.0.0',
    packages=find_packages(),
    install_requires=[
        minimalmodbus
    ],
    author='Benoit CASTETS',
    author_email='opensourceeng@email.com',
    description='Python Driver for Robotiq Grippers via Modbus RTU',
    long_description=open('README.md').read(),
    long_description_content_type='text/markdown',
    url='https://github.com/castetsb/pyRobotiqGripper',
    classifiers=[
        'Programming Language :: Python :: 3',
        # Add more classifiers as needed
    ],
)
