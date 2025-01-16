from setuptools import find_packages, setup
import os
import glob

package_name = 'rosyolo'

resource_files = glob.glob(os.path.join('resource', '**', 'pt'), recursive=True)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # ('share/ament_index/resource_index/data',
        #  resource_files),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hyl',
    maintainer_email='hylreg@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rosyolo = rosyolo.rosyolo:main',
        ],
    },
)
