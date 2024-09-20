from setuptools import setup

setup(
    name='rclpy_launch_demo',
    version='0.0.0',
    packages=[],
    py_modules=[
        'pub_sub.pub', 
        'pub_sub.sub',
        'pub_sub.key',
        ],
    install_requires=['setuptools'],
    author='John-Paul Ore',
    author_email='jwore@ncsu.edu',
    maintainer='John-Paul Ore',
    maintainer_email='jwore@ncsu.edu',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='Package containing tutorials showing how to use the rclpy API.',
    license='Apache License, Version 2.0',
    test_suite='test',
    entry_points={
        'console_scripts': [
            'pub = pub_sub.pub:main',
            'key = pub_sub.key:main',
            'sub = pub_sub.sub:main',
        ],
    },
)
