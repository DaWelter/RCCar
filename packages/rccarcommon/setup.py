#!/usr/bin/env python
import setuptools
from setuptools import setup
# For protobuf compiler
import subprocess
# Custom build step (https://seasonofcode.com/posts/how-to-add-custom-build-steps-and-commands-to-setuppy.html)
import setuptools.command.build_py
import sys

def CheckHasProtoc():
  try:
    subprocess.check_call(['protoc', '--version'])
    return True
  except OSError:
    return False


class CompileProtobufMsg(setuptools.Command):
  description = 'Compile the protobuf definitions into python modules'
  user_options = []
  
  def initialize_options(self):
    pass
  
  def finalize_options(self):
    pass
  
  def run(self):
    if CheckHasProtoc():
      subprocess.check_call([
        'protoc','--python_out=./rccarcommon/', 
        '-I./rccarcommon' ,'-I../../bldcdriver/msg/',
        '../../bldcdriver/msg/motor.proto'])
      subprocess.check_call([
        'protoc','--python_out=./rccarcommon/', 
        '-I./rccarcommon','-I../../bldcdriver/msg/',
        './rccarcommon/rccar.proto'])
    else:
      sys.stderr.write("Warning: Not generating protobuf message bindings because protoc compiler is not working.\n")


class CustomBuild(setuptools.command.build_py.build_py):

    def run(self):
      self.run_command('protobuf')
      setuptools.command.build_py.build_py.run(self)



setup(
  name='rccarcommon',
    cmdclass = {
      'protobuf' : CompileProtobufMsg,
      'build_py' : CustomBuild
      },
    version='0.0.1',
    description='The RC Car code',
    author='Michael Welter',
    author_email='michael@welter-4d.de',
    packages=['rccarcommon'],
    zip_safe=False)
