from setuptools import setup, find_packages

setup(name='quadruped_envs',
      version='0.0.1',
      install_requires=['gym'],
      packages=find_packages(include=['quadruped_envs', 'quadruped_envs.*']),
)