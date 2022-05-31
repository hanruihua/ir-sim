from setuptools import setup
import sys

setup(
    name='ir_sim2',
    py_modules=['ir_sim2'],
    version= '1.0',
    install_requires=[
        'matplotlib',
        'numpy',
        'scipy',
        'pyyaml',
        'pynput',
        'imageio',
        'pathlib',
        'cvxpy'
    ],
    description="A python based robot simulator framework for the intelligent robotics navigation and learning",
    author="Han Ruihua",
)