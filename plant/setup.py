from setuptools import setup
import os

package_name = 'plant'


def collect_data_files(base_dir: str):
    """
    Recursively install all files under base_dir into:
      share/<package_name>/<base_dir>/...
    """
    data_files = []
    for root, _, files in os.walk(base_dir):
        if not files:
            continue
        install_dir = os.path.join('share', package_name, root)
        src_files = [os.path.join(root, f) for f in files]
        data_files.append((install_dir, src_files))
    return data_files


data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

# ✅ data/ 폴더 전체( xml, toml, stl 등 ) 설치
data_files += collect_data_files('data')

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='seosuk',
    maintainer_email='seosuk@todo.todo',
    description='MuJoCo crazyflie plant loader',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plant = plant.plant:main',
        ],
    },
)

