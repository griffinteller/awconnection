import setuptools

with open('README.md') as f:
    README = f.read()

setuptools.setup(
    author="Griffin Teller",
    author_email="griffinteller@gmail.com",
    name='abrconnection',
    license="MIT",
    description='An interface between Python and Autonomous Battle Royale',
    version='v0.0.3',
    long_description=README,
    url='https://github.com/griffinteller/abrconnection',
    packages=setuptools.find_packages(),
    python_requires=">=3.6",
    classifiers=[

        'Development Status :: 2 - Pre-Alpha',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3',
        'Topic :: Software Development :: Libraries',
        'Topic :: Software Development :: Libraries :: Python Modules',
        'Intended Audience :: Education',
    ],
)
