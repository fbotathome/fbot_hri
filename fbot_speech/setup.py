from setuptools import find_packages, setup
import os
import glob

package_name = 'fbot_speech'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'fbot_speech'), glob.glob('fbot_speech/*.py')),
        (os.path.join('share', package_name, 'speech_plugins'), glob.glob('speech_plugins/*.py'))
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='stihl-bolsistas',
    maintainer_email='stihl-bolsistas@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'speech_synthesizer = fbot_speech.speech_synthesizer:main',
            'speech_recognizer = fbot_speech.speech_recognizer:main',
            'detector_hotword_node = fbot_speech.detector_hotword_node:main',
            'audio_player = fbot_speech.audio_player:main',
            'asr_riva = fbot_speech.speech_recognizer_riva:main',
            ]
    },
)



    # scripts=[
    #     os.path.join('scripts', 'wav_to_mouth.py'),
    #     os.path.join('scripts', 'detect_hotword.py'),
    # ],