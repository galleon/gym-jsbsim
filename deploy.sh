#!/bin/bash
if [[ "$TRAVIS_OS_NAME" != "linux" ]]; then
  python3 setup.py bdist_wheel --build-number=$TRAVIS_BUILD_NUMBER || python setup.py bdist_wheel --build-number=$TRAVIS_BUILD_NUMBER
  twine upload --username "galleon" --password "$PASSWORD" dist/*.whl
else
  twine upload --username "galleon" --password "$PASSWORD" dist/*manylinux*.whl
fi
