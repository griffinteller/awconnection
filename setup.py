from distutils.core import setup
setup(
  name = 'abrconnection',
  packages = ['abrconnection'],
  version = '0.1',
  license='MIT',
  description = 'An interface between Python and Autonomous Battle Royale',
  author = 'Griffin Teller',
  author_email = 'griffinteller@gmail.com',
  url = 'https://github.com/griffinteller/abrconnection',
  download_url = 'https://github.com/user/reponame/archive/v_01.tar.gz',
  keywords = ['Gaming', 'Educational', 'Robotics'],
  install_requires=[
          'validators',
          'beautifulsoup4',
      ],
  classifiers=[
    'Development Status :: 2 - Pre-Alpha',
    'Intended Audience :: Students',
    'Topic :: Software Development :: Build Tools',
    'License :: OSI Approved :: MIT License',
    'Programming Language :: Python :: 3'
  ],
)
