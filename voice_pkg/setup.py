import os
from glob import glob
from setuptools import setup

package_name = 'voice_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
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
            'select_answer = voice_pkg.select_answer:main',
            'speech_recognition = voice_pkg.speech_recognition:main',
            'speech_synthesis = voice_pkg.speech_synthesis:main',
        ],
    },
)
