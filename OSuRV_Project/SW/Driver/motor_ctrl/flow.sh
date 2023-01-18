#!/bin/bash

exit 0

make prerequisites

make stop clean build start

make install
