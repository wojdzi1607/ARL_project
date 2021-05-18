# Projekt ARL
## Cel projektu
Celem projektu było stworzenie środkowiska, opartego o robota Ardrone z możliwością
autonomicznego podążania za robotem Husky.

## Instalacja dockera
Pobrać pliki dockera [stąd](https://drive.google.com/file/d/1qRobw97PCbPGaR0gGytjJ35a-JCViHB-/view?usp=sharing).

Wymagane są sterowniki Nvidia takie jak na hoście.
Wersję można sprawdzić komendą: ```nvidia-smi```.
Np. dla wersji 460.73.01 potrzeba pliku **_NVIDIA-Linux-x86_64-460.73.01.run_**.
Sterowniki można pobrać [stąd](https://www.nvidia.pl/Download/index.aspx?lang=pl).
Nazwe wersji trzeba zmienić też w pliku ```./build.sh```. Następnie budujemy obraz z tego pliku,
i uruchamiamy ```./run.sh```.

## Instalacja tum_simulator
Zainstalować tum_simulator z:
https://github.com/angelsantamaria/tum_simulator.git

Nie zapomnieć zainstalować poniższych paczek:
```
sudo apt-get install ros-<your_version>-hector-*
sudo apt-get install ros-indigo-ardrone-autonomy
```
Podmienić folder **_cvg_sim_gazebo_** z oryginalnego repozytorium na folder
**_cvg_sim_gazebo_** z tego repozytorium.
## Instalacja odsługi pada (opcjonalnie)
```
source /opt/ros/indigo/setup.bash
roscd
svn checkout https://svncvpr.informatik.tu-muenchen.de/cvpr-ros-pkg/trunk/ardrone_helpers
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:`pwd`/ardrone_helpers
rosmake ardrone_joystick
rosmake joy
```
source: [here](http://wiki.ros.org/tum_simulator)
## Instalcja wymaganych paczek
Zainstalować w **_catkin_ws_** dodatkowe paczki gazebo:

https://github.com/ros-simulation/gazebo_ros_pkgs/tree/indigo-devel

## Budowanie catkin_ws
w **_catkin_ws/src_** tylko foldery **_gazebo_ros_pkgs_** i **_tum_simulator_**.
Następnie w catkin_ws uruchomić ```catkin_make ```

## Husky
Zainstalować z oricjalnego repozytorium:
http://wiki.ros.org/husky_gazebo/Tutorials/Simulating%20Husky

Podmienić foldery **_huksy_control_**, **_huksy_description_** i **_huksy_gazebo_**
na te z folderu **_husky_**

## Instalacja obsługi klawiatury dla Husky
Zainstalować z oricjalnego repozytorium: 
http://wiki.ros.org/teleop_twist_keyboard

W pliku **_teleop_twist_keyboard.py_** w lini 82 zmienić nazwę topica na ```'/husky/cmd_vel'```

## Instrukcja uruchomienia całego środowiska
po stronie hosta (opcjonalnie - jeśli wystąpił error z DISPLAY):
```
xhost +local:`docker inspect --format='{{ .Config.Hostname }}' ubuntu_ros_indigo`
```
po stronie dockera:
```
docker start ubuntu_ros_indigo && docker exec -it ubuntu_ros_indigo bash
export DISPLAY=:0
cd ~/share/catkin_ws
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$PWD/models
source /opt/ros/indigo/setup.bash
source devel/setup.bash
roslaunch cvg_sim_gazebo ardrone_testworld.launch
```
Program autonomicznego podążania drona został zaimplementowany w pliku _/cvg_sim_gazebo/scripts/**autonomous.py**_ 
