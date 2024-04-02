from setuptools import find_packages, setup

package_name = 'task2_pkg'

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
    maintainer='nida',
    maintainer_email='nida@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "circle_node=task2_pkg.circle_publisher:main",
        "triangle_node=task2_pkg.triangle_publisher:main",
        "rectangle_node=task2_pkg.rectangle_publisher:main",
        "ellipse_node=task2_pkg.ellipse_publisher:main"
        ],
    },
)
