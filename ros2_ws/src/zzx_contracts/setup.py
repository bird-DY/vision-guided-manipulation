from setuptools import find_packages, setup


package_name = 'zzx_contracts'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zzx',
    maintainer_email='764472556@qq.com',
    description='Runtime semantic validators for Zzxrobot ROS 2 interfaces.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'target_contract_monitor = zzx_contracts.target_contract_monitor:main',
        ],
    },
)
