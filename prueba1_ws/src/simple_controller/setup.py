from setuptools import find_packages, setup

package_name = 'simple_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='Osvaldo Flores Oropeza',
    maintainer_email='osvaldofloresoropeza@hotmail.com',
    description='Nodo puente entre ROS 2 y Arduino/ESP32 mediante SimpleController',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # Formato: 'nombre_del_ejecutable = carpeta_del_paquete.nombre_del_script:funcion_main'
            'simple_controller_node = simple_controller.simple_controller_node:main',
        ],
    },
)