# -*- coding: utf-8 -*-

'''
'''

###############################################################################

from __future__ import print_function

__author__	    = 'Milos Subotic'
__email__	    = 'milos.subotic.sm@gmail.com'
__copyright__   = 'MIT'

###############################################################################

import os
import sys
import glob
import re
import inspect
import errno
import fnmatch

###############################################################################

# Whole expression.
_showRegex = re.compile(r'\bshow\s*\(\s*(.*)\s*\)')

def show(var):
	varName = ''
	for line in inspect.getframeinfo(inspect.currentframe().f_back)[3]:
		m = _showRegex.search(line)
		if m:
			varName = m.group(1)
			break
	print('{0} = {1}'.format(varName, var))

###############################################################################

VERB  = 0
DEBUG = 1
INFO  = 2
WARN  = 3
ERROR = 4
FATAL = 5

__MSG_PRINT_TYPE = True
def msg_print_type(tf):
	global __MSG_PRINT_TYPE
	__MSG_PRINT_TYPE = tf

def msg(msg_type, *args, **kwargs):
	global __MSG_PRINT_TYPE
	if msg_type == VERB:
		color = "\x1b[37m"
		msg_type_str = "verbose"
	elif msg_type == DEBUG:
		color = "\x1b[92m"
		msg_type_str = "debug"
	elif msg_type == INFO:
		color = "\x1b[94m"
		msg_type_str = "info"
	elif msg_type == WARN:
		color = "\x1b[93m"
		msg_type_str = "warning"
	elif msg_type == ERROR:
		color = "\x1b[91m"
		msg_type_str = "error"
	elif msg_type == FATAL:
		color = "\x1b[91m"
		msg_type_str = "fatal"
	else:
		raise AssertError("Wrong msg_type!")
	
	if __MSG_PRINT_TYPE:
		m = msg_type_str + ":"
	else:
		m = ""
	
	print(color + m, sep = '', end = '')
	print(*args, **kwargs, sep = '', end = '')
	print("\x1b[0m", sep = '', end = '') # Return to normal.
	print()

	if msg_type == FATAL:
		sys.exit(1)


def warn(*args, **kwargs):
	print('WARN: ', *args, file = sys.stderr, **kwargs)

def error(*args, **kwargs):
	print('ERROR: ', *args, file = sys.stderr, **kwargs)
	sys.exit(1)

###############################################################################

def correct_path(path):
	return path.replace('\\', '/')
	
def file_exists(path):
	return os.path.isfile(path)

''' mkdir -p functionality:  '''
def mkdir_p(path):
	try:
		os.makedirs(path)
	except OSError as exc: # Python >2.5
		if exc.errno == errno.EEXIST and os.path.isdir(path):
			pass
		else:
			raise

def recursive_glob(pattern, directory = '.'):
	found = []
	for root, dirs, files in os.walk(str(directory), followlinks = True):
		dirs = map(lambda s: s + '/', dirs)
		for base_name in files:
			if fnmatch.fnmatch(base_name, pattern):
				found.append(os.path.join(root, base_name))
		for base_name in dirs:
			if fnmatch.fnmatch(base_name, pattern):
				found.append(os.path.join(root, base_name))			
	return found

###############################################################################
