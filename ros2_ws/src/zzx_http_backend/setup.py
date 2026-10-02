from setuptools import find_packages, setup

setup(name='zzx_http_backend', version='0.1.0', packages=find_packages(exclude=['test']),
      data_files=[('share/ament_index/resource_index/packages', ['resource/zzx_http_backend']),
                  ('share/zzx_http_backend', ['package.xml']),
                  ('share/zzx_http_backend/config', ['config/capabilities.json'])],
      install_requires=['setuptools'], zip_safe=True, maintainer='zzx',
      maintainer_email='764472556@qq.com', license='MIT',
      description='Competition HTTP ROS Action adapter.', tests_require=['pytest'],
      entry_points={'console_scripts': ['http_action_backend = zzx_http_backend.node:main']})
