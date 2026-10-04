from setuptools import find_packages, setup

setup(name='zzx_camera', version='0.1.0', packages=find_packages(exclude=['test']),
      data_files=[('share/ament_index/resource_index/packages', ['resource/zzx_camera']),
                  ('share/zzx_camera', ['package.xml']),
                  ('share/zzx_camera/launch', ['launch/camera.launch.py']),
                  ('share/zzx_camera/config', ['config/vendor_sources.json',
                                             'config/cyclonedds_wsl.xml', 'config/record_qos.yaml'])],
      install_requires=['setuptools'], zip_safe=True, maintainer='zzx',
      maintainer_email='764472556@qq.com', license='MIT',
      description='Single-owner camera ingress.', tests_require=['pytest'],
      entry_points={'console_scripts': ['camera_manager = zzx_camera.node:main']})
