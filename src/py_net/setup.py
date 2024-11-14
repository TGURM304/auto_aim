from setuptools import find_packages, setup

package_name = 'py_net'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools',
                      'openvino'
                      ],
    zip_safe=True,
    maintainer='interweave',
    maintainer_email='interweave@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_node = py_net.test_node:main',  # 测试节点
            'number = py_net.number_ov:main'      # 数字(装甲板图案)检测-OpenVino
        ],
    },
)
