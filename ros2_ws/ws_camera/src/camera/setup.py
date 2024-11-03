
from setuptools import setup

package_name = 'camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
    ('share/' + package_name, ['package.xml']),  # Include the package.xml explicitly
    ('share/' + package_name, ['camera/settings.json']),
],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hakaw',
    maintainer_email='hakaw@todo.todo',
    description='ROS2 array publisher and subscriber',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'array_publisher = camera.publisher:main',
            'array_subscriber = camera.subscriber:main',
        ],
    },
)
