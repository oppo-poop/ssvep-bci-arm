from setuptools import find_packages, setup

package_name = 'roarm_task'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/scenes', ['scenes/demo.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ljl',
    maintainer_email='ljl@localhost',
    description='取放任务层：七个动作原语 + 序列状态机 + 终端入口',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task = roarm_task.cli:main',
        ],
    },
)
