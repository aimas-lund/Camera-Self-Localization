from setuptools import setup

package_name = 'A8Camera'

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
    maintainer='Vicente Bosch',
    maintainer_email='s222928@dtu.dk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_talker = A8Camera.a8_camera_publisher:main',
            'listener = A8Camera.a8_subscriber:main',
            'gimbal_talker = A8Camera.a8_gimball_publisher:main',
            'streamer = A8Camera.a8_stream:main',
        ],
    },
)
