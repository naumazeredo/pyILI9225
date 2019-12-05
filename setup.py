# Workaround for issue in Python 2.7.3
# See http://bugs.python.org/issue15881#msg170215
try:
    import multiprocessing
except ImportError:
    pass

try:
    # Try using ez_setup to install setuptools if not already installed.
    from ez_setup import use_setuptools
    use_setuptools()
except ImportError:
    # Ignore import error and assume Python 3 which already has setuptools.
    pass

from setuptools import setup, find_packages


classifiers = ['Development Status :: 3 - Alpha',
               'Operating System :: POSIX :: Linux',
               'License :: OSI Approved :: MIT License',
               'Intended Audience :: Developers',
               'Programming Language :: Python :: 2.7',
               'Programming Language :: Python :: 3',
               'Topic :: Software Development',
               'Topic :: System :: Hardware']

setup(name              = 'pyILI9225',
      version           = '0.1.0',
      author            = 'Naum Azeredo',
      author_email      = 'naumazeredo@gmail.com',
      description       = 'Library to control an ILI9225 TFT LCD display.',
      license           = 'MIT',
      classifiers       = classifiers,
      url               = 'https://github.com/naumazeredo/pyILI9225',
      dependency_links  = ['wiringpi'],
      install_requires  = ['wiringpi==2.46.0'],
      packages          = find_packages())
