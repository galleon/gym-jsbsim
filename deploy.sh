#!/bin/bash
if [[ "$TRAVIS_OS_NAME" != "linux" ]]; then
  python3 setup.py bdist_wheel || python setup.py bdist_wheel
  twine upload --username "galleon" --password "$PASSWORD" dist/*.whl
else
  twine upload --username "galleon" --password "$PASSWORD" dist/*manylinux*.whl
fi
