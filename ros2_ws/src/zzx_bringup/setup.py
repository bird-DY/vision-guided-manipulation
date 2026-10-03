from setuptools import find_packages, setup

setup(name='zzx_bringup', version='0.1.0', packages=find_packages(exclude=['test']),
      data_files=[('share/ament_index/resource_index/packages', ['resource/zzx_bringup']),
                  ('share/zzx_bringup', ['package.xml']),
                  ('share/zzx_bringup/launch', ['launch/stack.launch.py']),
                  ('share/zzx_bringup/config', ['config/profiles.json'])],
      install_requires=['setuptools'], zip_safe=True, maintainer='zzx',
      maintainer_email='764472556@qq.com', license='MIT',
      description='Managed robot bringup profiles.', tests_require=['pytest'])
