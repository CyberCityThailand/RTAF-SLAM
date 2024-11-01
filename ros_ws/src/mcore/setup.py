from setuptools import find_packages, setup

package_name = 'mcore'

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
    maintainer='mark',
    maintainer_email='chanonmark5@hotmail.com',
    description='Mallanoo Core Designed for Drone Navigation',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "mallanoo_agent = mcore.mallanoo_agent:main",
            "avoidance_node = mcore.object_avoidance:main",
        ],
    },
)
