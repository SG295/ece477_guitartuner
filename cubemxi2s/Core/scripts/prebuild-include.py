from os import path, walk
import SCons.Errors

Import('env')
try:
    import configparser
except ImportError:
    import ConfigParser as configparser

config = configparser.ConfigParser()
config.read('platformio.ini')
prebuild_include = config.get('env', 'prebuild_include').split()

source_paths = set()
header_paths = set()
for included in prebuild_include:
    if not path.exists(included):
        raise SCons.Errors.UserError(
            "prebuild-include.py could not find path '%s'" % included)
    for root, dirs, files in walk(included):
        for file in files:
            if file.endswith(('.c', '.cpp', '.s')):
                source_paths.add(root.replace('\\', '/'))
            if file.endswith(('.h', '.hpp')):
                header_paths.add(root.replace('\\', '/'))

if not 'SRC_FILTER' in env:
    env['SRC_FILTER'] = ['+<Src/>']
env['SRC_FILTER'] += ['+<%s/>' % source_path for source_path in source_paths]

if not 'BUILD_FLAGS' in env:
    env['BUILD_FLAGS'] = []
env['BUILD_FLAGS'] += ['-I' + header_path for header_path in header_paths]