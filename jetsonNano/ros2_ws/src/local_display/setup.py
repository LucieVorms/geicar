from setuptools import find_packages, setup

package_name = 'local_display'

setup(
    name=package_name,
    version='0.0.0',
    # Packages to export
    packages=find_packages(exclude=['test']),
    # Files we want to install, specifically launch files
    data_files=[
        # Install marker file in the package index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Include our package.xml file    
        ('share/' + package_name, ['package.xml']),
    ],
    # This is important as well
    install_requires=['setuptools'],
    zip_safe=True,
    author='Lucie Vorms',
    author_email='vorms@insa-toulouse.fr',
    maintainer='Lucie Vorms',
    maintainer_email='151724807+LucieVorms@users.noreply.github.com',
    keywords=['display', 'images'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: TODO',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    # Like the CMakeLists add_executable macro, you can add your python
    # scripts here.
    entry_points={
        'console_scripts': ['images_subscriber_node = local_display.images_subscriber_node:main'
        ],
    },
)
