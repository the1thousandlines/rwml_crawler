FROM althack/ros2:foxy-dev 

# ** [Optional] Uncomment this section to install additional packages. **
#
# ENV DEBIAN_FRONTEND=noninteractive
# RUN apt-get update \
#    && apt-get -y install --no-install-recommends <your-package-list-here> \
#    #
#    # Clean up
#    && apt-get autoremove -y \
#    && apt-get clean -y \
#    && rm -rf /var/lib/apt/lists/*
# ENV DEBIAN_FRONTEND=dialog
RUN pip3 install jupyter
RUN pip3 install pyserial
# RUN echo "export PATH:$PATH:/home/ros/.local/bin" >> /home/ros/.bashrc
RUN pip3 install jupyterlab-pygments
RUN pip3 install opencv-python
RUN pip3 install matplotlib
RUN pip3 install gym stable-baselines3
RUN sudo apt update
RUN sudo apt-get install ffmpeg libsm6 libxext6  -y
# Set up auto-source of workspace for ros user
ARG WORKSPACE
RUN echo "if [ -f ${WORKSPACE}/install/setup.bash ]; then source ${WORKSPACE}/install/setup.bash; fi" >> /home/ros/.bashrc