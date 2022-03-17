from setuptools import setup

package_name = 'speech_service'

setup(
    name=package_name,
    version='0.0.0',
    packages=['scripts'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ito-masaki',
    maintainer_email='ito.masaki@em.ci.ritsumei.ac.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'speech_service = scripts.speech_service:main',
            'recognition = scripts.recognition:main',
            'synthesis = scripts.synthesis:main'
        ],
    },
)
