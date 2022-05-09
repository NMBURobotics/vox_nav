## Build with:
##   docker build -t lcas.lincoln.ac.uk/lcas/vox_nav:docker .
##
## Run with:
##   docker-compose up
##
## Kill with:
##   docker-compose down

FROM tiryoh/ros2-desktop-vnc:foxy
LABEL maintainer="Fetullah Atas<fetulahatas1@gmail.com>"

RUN apt-get update; apt-get -y upgrade



# Creating ros2_ws
RUN mkdir -p ~/ros2_ws/src
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && \
                  cd ~/ros2_ws/ && \
                  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --parallel-workers 4"

# Installing dependecies
RUN . /opt/ros/foxy/setup.sh; \
    apt install -y python3-colcon-common-extensions && \
    apt install -y python3-rosdep2 xdotool psmisc gnome-terminal && \
    apt install -y ros-foxy-twist-mux ros-foxy-navigation2 && \
    apt install -y python3-colcon-common-extensions && \
    apt install -y python3-vcstool && \
    apt install -y xdotool && \
    apt install -y coinor-libipopt-dev && \
    rosdep update && cd ~/ros2_ws && \
    rosdep install -y -r -q --from-paths src --ignore-src --rosdistro foxy 
 
# Cloning repos and building
RUN cd ~/ros2_ws && wget https://raw.githubusercontent.com/jediofgever/vox_nav/foxy/underlay.repos && vcs import src < underlay.repos --recursive 
RUN cd ~/ros2_ws && vcs pull src 

# install CUDA
RUN /bin/bash -c 'export cuda=11.4 ; \
                  source ~/ros2_ws/src/vox_nav/scripts/install_cuda_ubuntu.sh; \
                  echo "export PATH=/usr/local/cuda-11.4/bin${PATH:+:${PATH}}" >> /home/ubuntu/.bashrc;  \
                  echo "export LD_LIBRARY_PATH=/usr/local/cuda-11.4/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}" >> /home/ubuntu/.bashrc;'
 
RUN /bin/bash -c 'cd ~/ros2_ws; \
                  source /opt/ros/foxy/setup.bash; \
                  rosdep install -y -r -q --from-paths src --ignore-src --rosdistro foxy; \
                  export PATH=/usr/local/cuda-11.4/bin${PATH:+:${PATH}}; \
                  export LD_LIBRARY_PATH=/usr/local/cuda-11.4/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}; \
                  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DWITH_IPOPT=true --packages-select casadi ompl ACADO cupoch fast_gicp; \
                  sudo cp ~/ros2_ws/install/ompl/lib/libompl.so* /usr/local/lib/; \
                  sudo cp ~/ros2_ws/install/casadi/lib/libcasadi.so* /usr/local/lib/;'
 
RUN /bin/bash -c '. /opt/ros/foxy/setup.bash; cd /home/ubuntu/ros2_ws; \
                  echo "source /home/ubuntu/ros2_ws/install/setup.bash" >> /home/ubuntu/.bashrc; \
                  echo "export NO_AT_BRIDGE=1" >> /home/ubuntu/.bashrc'

# Building vox_nav
RUN /bin/bash -c 'source /opt/ros/foxy/setup.bash; \
                  source ~/ros2_ws/install/setup.bash; \
                  cd ~/ros2_ws && source build/ACADO/acado_env.sh; \
                  export PATH=/usr/local/cuda-11.4/bin${PATH:+:${PATH}}; \
                  export LD_LIBRARY_PATH=/usr/local/cuda-11.4/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}; \
                  cd ~/ros2_ws && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DACADOS_WITH_QPOASES=ON -DACADO_CODE_IS_READY=ON -DUSE_VGICP_CUDA=ON --packages-skip-regex archived --packages-skip casadi ompl ACADO cupoch; \
                  source ~/ros2_ws/install/setup.bash'

# export NO_AT_BRIDGE fixes an error in the rqt:
# "AT-SPI: Error retrieving accessibility bus address: org.freedesktop.DBus.Error.ServiceUnknown:
# The name org.a11y.Bus was not provided by any .service files"

RUN cd /tmp && curl -fOL https://github.com/cdr/code-server/releases/download/v3.12.0/code-server_3.12.0_amd64.deb && dpkg -i code-server_3.12.0_amd64.deb && rm code-server_3.12.0_amd64.deb
RUN bash -c 'echo -e "[supervisord]\nredirect_stderr=true\nstopsignal=QUIT\nautorestart=true\ndirectory=/root\n\n[program:codeserver]\ndirectory=/home/ubuntu\ncommand=/usr/bin/code-server --auth none --bind-addr 0.0.0.0:8888\nuser=ubuntu\nenvironment=DISPLAY=:1,HOME=/home/ubuntu,USER=ubuntu" > /etc/supervisor/conf.d/codeserver.conf'