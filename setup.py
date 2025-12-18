from setuptools import find_packages, setup

package_name = 'forward_kinemaitics'

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
    maintainer='mauri',
    maintainer_email='mauri@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "joint_pub=forward_kinemaitics.joints_pub:main","joint_sub=forward_kinemaitics.joints_sub:main",
            "fwd_rr=forward_kinemaitics.fwd_rr:main","fwd_kr6=forward_kinemaitics.fwd_kr6:main","fwd_ur5=forward_kinemaitics.fwd_ur5:main",
            "jinv_kr6=forward_kinemaitics.jinv_kr6:main",
            "jinv_ur5=forward_kinemaitics.jinv_ur5:main",
            
        ],
    },
)
