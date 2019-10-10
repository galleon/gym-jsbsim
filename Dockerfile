FROM alpine
FROM python:3.7

RUN mkdir /home/gym-jsbsim
WORKDIR /home/gym-jsbsim

RUN pip install .

CMD ["/bin/bash"]