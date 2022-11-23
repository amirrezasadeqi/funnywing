# Some notes about Our docker status

At the moment the wing_base docker image is at minimum level which we can use it for our
work. ROS noetic, ArduPilot SITL simulation, neovim v0.8.0 and my neovim configurations have
been installed(we can have code compeletion in docker).

So the only things that differ with the local installation of working environment is:

1. SwiftGust fork of ardupilot_gazebo have not been cloned and compiled and also the Gazebo environment variabless
   have not been set up in the .bashrc file.
2. FunnyWing repository has not been cloned and setup(it means, the soft links and files that we need to copy in different
   places of the system, have not been created/copied.)
3. I think we need to install pyqt5 packages to run GUI application.
4. note that we need to run xhost + commnad, before running GUI applications.
5. Using bridge network in docker containers cause the container to have an IP address so we can easily use it.(for example
for running neovim server inside the docker.)
6. My there be some more things which I have forgot at the moment and in future I will add them and ... .

At the moment these works are not at a high level of priority, and the project needs some other major things to add, so at the momment
I work on other things and If I need to use these docker images, I can setup the necessary things manually. So let's use this and enjoy
it and in the future I will ... 

## How to use neovim and neovide for remote developement in docker

The neovim GUIs are some kind of clients and if we run neovim headless server in docker container, in the local
system we can use neovide to connect to that server and edit codes in docker and get code compeletions from the
sources installed and seted up in the container. To do this we use below code snippets:
```
# Run this in the container to startup a neovim server
nvim --listen 172.17.0.2:80 --headless

# Run below command in the local machine to connect into neovim server
neovide --remote-tcp 172.17.0.2:80
```

**Now the remote developement setup using neovim is ready to use .**

#### **__/Note\ __ that the contents are this project dockerfiles are  very valuable and can be used in future projects.**

At the moment I will save the base image in a compressed tar file using `docker save -o ...`
command and load it in my laptop using `docker load -i ...` and do my in-docker developement.

                      ### Enjoy it!!!!!!!!!!!!

