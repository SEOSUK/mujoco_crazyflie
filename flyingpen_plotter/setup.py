from setuptools import setup, find_packages

package_name = 'flyingpen_plotter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'pyqtgraph',
    ],
    zip_safe=True,
    maintainer='seosuk',
    maintainer_email='seosuk@example.com',
    description='Realtime dashboard node for Crazyflie topics',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'realtime_plotter = flyingpen_plotter.realtime_plotter:main',
            'data_logging_python = flyingpen_plotter.data_logging_python:main',
            'data_logging_drone_states = flyingpen_plotter.data_logging_drone_states:main',            
            'data_logging_arrow_pwm = flyingpen_plotter.data_logging_arrow_pwm:main',                        
            'data_logging_rpy_acceleration = flyingpen_plotter.data_logging_rpy_acceleration:main',
            'data_logging_chat_frame = flyingpen_plotter.data_logging_chat_frame:main',
            'normal_vector_debug = flyingpen_plotter.Normal_vector_debug:main',

        ],
    },
)
