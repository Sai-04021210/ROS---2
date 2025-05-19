from setuptools import find_packages, setup

package_name = 'basic_py_examples'

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
    maintainer='ram',
    maintainer_email='inthedarkshades0008@gmail.com',
    description='Basic ROS2 Python Pub-Sub Example',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_publisher = basic_py_examples.simple_publisher:main',
            'simple_subscriber = basic_py_examples.simple_subscriber:main',
            'parameters_demo = basic_py_examples.parameters:main',
        ],
    },
)