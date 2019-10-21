#!/bin/bash
set -e -x
cd /io

for PYBIN in /opt/python/*3[67]*/bin; do
    "${PYBIN}/pip" install cython
    "${PYBIN}/python" setup.py bdist_wheel
done

# Bundle external shared libraries into the wheels
for whl in dist/*.whl; do
    auditwheel repair "$whl" --plat $PLAT -w dist/
done

