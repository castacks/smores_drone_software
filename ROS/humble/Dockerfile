FROM theairlab/dtc:x86-05-stable-thirdparty

ENV DEBIAN_FRONTEND=noninteractive

# Create a user with the same UID and GID as the host user
ARG HOST_UID=1000
ARG HOST_GID=1000

RUN apt-get update -y && \
    apt-get upgrade -y

RUN groupadd -g ${HOST_GID} docked && \
    useradd -m -u ${HOST_UID} -g ${HOST_GID} -s /bin/bash drone && \
    usermod -aG sudo drone && \
    echo "drone ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/drone && \
    chmod 0440 /etc/sudoers.d/drone && \
    echo "export PATH=$PATH:/home/drone/.local/bin" >> /home/drone/.bashrc

USER drone
WORKDIR /home/drone

COPY pip-requirements.txt pip-requirements.txt

RUN pip3 install -r pip-requirements.txt