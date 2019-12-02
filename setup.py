from setuptools import setup, find_packages
import warnings

DEPENDENCY_PACKAGE_NAMES = ['numpy', 'pinocchio', 'meshcat']

def check_dependencies():
    missing_dependencies = []
    for package_name in DEPENDENCY_PACKAGE_NAMES:
        try:
            __import__(package_name)
        except ImportError:
            print(package_name)
            missing_dependencies.append(package_name)

    if missing_dependencies:
        raise ValueError(
            'Missing dependencies: {}. Check the installation instructions at https://github.com/rstrudel/poppyarm.'.format(
                missing_dependencies))

check_dependencies()

setup(
    name='poppyarm',
    version='0.0.1',
    author='Robin Studel',
    author_email='robin.strudel@inria.fr',
    description='Poppy Ergo Jr Arm moved with Pinocchio',
    packages=find_packages(),
)
