from setuptools import find_packages, setup

package_name = 'image_action_server'

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
    maintainer='tamer',
    maintainer_email='johnatabrayan@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_action_server = image_action_server.image_action_server:main',
            'image_action_client = image_action_server.image_action_client:main',
        ],
    },
)
