from setuptools import setup

package_name = 'pointcloud_filtering'

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
    maintainer='student',
    maintainer_email='arvid.sorfeldt@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pointcloud_sub = pointcloud_filtering.pointcloud_subscriber:main',
            'example_service = pointcloud_filtering.example_server:main'
        ]
    },
)
