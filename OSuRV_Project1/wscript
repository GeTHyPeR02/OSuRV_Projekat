	#!/usr/bin/env python3
# encoding: utf-8

'''
@author: Milos Subotic <milos.subotic.sm@gmail.com>
@license: MIT

@brief: Waf script just for distclean and dist commands.
'''

###############################################################################

import os
import fnmatch
import shutil
import datetime

import waflib

###############################################################################

APPNAME = 'OSuRV_Project'

top = '.'

###############################################################################

def recursive_glob(pattern, directory = '.'):
	if pattern.startswith('/'):
		for root, dirs, files in os.walk(directory, followlinks = True):
			if root == directory:
				for f in files:
					if fnmatch.fnmatch('/' + f, pattern):
						yield os.path.join(root, f)
				for d in dirs:
					if fnmatch.fnmatch('/' + d + '/', pattern):
						yield os.path.join(root, d)
	else:
		for root, dirs, files in os.walk(directory, followlinks = True):
			for f in files:
				if fnmatch.fnmatch(f, pattern):
					yield os.path.join(root, f)
			for d in dirs:
				if fnmatch.fnmatch(d + '/', pattern):
					yield os.path.join(root, d)

def collect_git_ignored_files():
	for gitignore in recursive_glob('.gitignore'):
		with open(gitignore) as f:
			base = os.path.dirname(gitignore)
			
			ignore_patterns = []
			not_ignore_patterns = [] # Those starting with !
			for pattern in f.readlines():
				pattern = pattern.rstrip() # Remove new line stuff on line end.
				if pattern.startswith('!'):
					not_ignore_patterns.append(pattern[1:])
				else:
					ignore_patterns.append(pattern)
			
			not_ignore = set([])
			for pattern in not_ignore_patterns:
				for f in recursive_glob(pattern, base):
					not_ignore.add(f)
			
			for pattern in ignore_patterns:
				for f in recursive_glob(pattern, base):
					if f not in not_ignore:
						yield f

###############################################################################

def distclean(ctx):
	for fn in collect_git_ignored_files():
		if os.path.isdir(fn):
			shutil.rmtree(fn)
		else:
			os.remove(fn)

def dist(ctx):
	now = datetime.datetime.now()
	time_stamp = '{:d}-{:02d}-{:02d}-{:02d}-{:02d}-{:02d}'.format(
		now.year,
		now.month,
		now.day,
		now.hour,
		now.minute,
		now.second
	)
	ctx.arch_name = '../{}-{}.zip'.format(APPNAME, time_stamp)
	ctx.algo = 'zip'
	ctx.base_name = APPNAME
	# Also pack git.
	waflib.Node.exclude_regs = waflib.Node.exclude_regs.replace(
'''
**/.git
**/.git/**
**/.gitignore''', '')
	# Ignore waf's stuff.
	waflib.Node.exclude_regs += '\n**/.waf*'
	
###############################################################################
