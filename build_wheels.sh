#!/bin/bash
set -e -x
cd /io

for PYBIN in /opt/python/*3[67]*/bin; do
    "${PYBIN}/pip" install cython
    "${PYBIN}/python" setup.py bdist_wheel
done

# Bundle external shared libraries into the wheels
for whl in dist/*.whl; do
    auditwheel repair "$whl" --plat manylinux2010_x86_64 -w dist/
done

