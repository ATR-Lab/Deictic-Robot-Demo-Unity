from setuptools import setup

setup(name='deictic_control', version='0.1.0', packages=['deictic_control'],
      data_files=[('share/ament_index/resource_index/packages', ['resource/deictic_control']),
                  ('share/deictic_control', ['package.xml', 'LICENSE'])],
      install_requires=['setuptools', 'numpy', 'scipy>=1.11,<2'],
      extras_require={'fusion': ['gtsam==4.2.2'], 'test': ['pytest']},
      maintainer='ATR Lab', maintainer_email='atr@kent.edu',
      description='Egocentric frame fusion, deictic reaching and bimanual K1 simulation teleoperation',
      license='Apache-2.0',
      entry_points={'console_scripts': ['deictic_control = deictic_control.node:main']})
