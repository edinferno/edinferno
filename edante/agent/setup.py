#!/urs/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup
d = generate_distutils_setup() # Now it should work...
d['packages'] = ['agent']
d['package_dir'] = {'': 'src'}

setup(**d) # magic!
