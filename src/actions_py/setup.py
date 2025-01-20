from setuptools import find_packages, setup

package_name = 'actions_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='amrit',
    maintainer_email='amrit@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={  # Modern replacement for `tests_require`
        'test': ['pytest'],  # Add more test dependencies if needed
    },
    entry_points={
        'console_scripts': [
            "count_until_server = actions_py.count_until_server:main"
        ],
    },
)
