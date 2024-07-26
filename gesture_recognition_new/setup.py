from setuptools import setup

package_name = 'gesture_recognition_new'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pradheep',
    maintainer_email='mantissa6789@gmail.com',
    description='Gesture recognition using OpenCV for TurtleBot 3 control',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gesture_recognition = gesture_recognition.gesture_recognition:main',
        ],
    },
)
