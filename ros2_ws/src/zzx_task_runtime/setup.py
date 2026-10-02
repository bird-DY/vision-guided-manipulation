from setuptools import find_packages, setup

setup(
    name='zzx_task_runtime', version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/zzx_task_runtime']),
        ('share/zzx_task_runtime', ['package.xml']),
        ('share/zzx_task_runtime/examples', ['examples/joint_request.json']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='zzx', maintainer_email='764472556@qq.com',
    description='Durable idempotent dispatch for robot operations.',
    license='MIT', tests_require=['pytest'],
    entry_points={'console_scripts': [
        'durable_move_arm = zzx_task_runtime.cli:main',
    ]},
)
