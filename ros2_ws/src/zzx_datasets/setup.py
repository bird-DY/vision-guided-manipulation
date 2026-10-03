from setuptools import find_packages, setup

setup(
    name='zzx_datasets', version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/zzx_datasets']),
        ('share/zzx_datasets', ['package.xml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='zzx', maintainer_email='764472556@qq.com',
    description='Traceable RGB-D manifests and read-only offline replay.',
    license='MIT', tests_require=['pytest'],
    entry_points={'console_scripts': ['rgbd_dataset = zzx_datasets.cli:main']},
)
