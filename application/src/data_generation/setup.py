from setuptools import find_packages, setup

package_name = 'data_generation'

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
    maintainer='alfredo',
    maintainer_email='alfredo.hardjoprawiro@st.oth-regensburg.de',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'export_pcl = data_generation.export_pcl:main',
            'export_img = data_generation.export_img:main',
            'waypoints_driver = data_generation.waypoints_driver:main',
        ],
    },
)
