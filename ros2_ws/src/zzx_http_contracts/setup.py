from setuptools import find_packages, setup

setup(
    name='zzx_http_contracts', version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/zzx_http_contracts']),
        ('share/zzx_http_contracts', ['package.xml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='zzx', maintainer_email='764472556@qq.com',
    description='Loopback-only competition HTTP protocol regression harness.',
    license='MIT', tests_require=['pytest'],
    entry_points={'console_scripts': ['competition_http_fake = zzx_http_contracts.fake:main']},
)
