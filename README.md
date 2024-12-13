# COURS - PX4 & ROS 2 Humble — Sujet

Created by: Julien Geneste
Created time: 5 décembre 2024 17:40
Type: COURS
État: Done

> Des questions vous sont posées au fur et à mesure de votre avancement. Elles sont la pour voir si vous comprenez ce que vous faites mais aussi pour vous guider. Afin de pouvoir vous mettre une note, vous devez me rendre un compte rendu avec les réponses à ces questions (très synthétique) à m’envoyer à julien.geneste@bordeaux-inp.fr.
> 

# Installer ROS 2 (Ubuntu 22.04 amd64)

## **Partie 1 : Mise en place d’un environnement de développement**

**Étape 1 : Installation**

1.	Installer **ROS2 Humble**.

```bash
sudo apt update && sudo apt install
locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

export LANG=en_US.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL [https://raw.githubusercontent.com/ros/rosdistro/master/ros.key](https://raw.githubusercontent.com/ros/rosdistro/master/ros.key) -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] [http://packages.ros.org/ros2/ubuntu](http://packages.ros.org/ros2/ubuntu) $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt upgrade -y
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
source /opt/ros/humble/setup.bash && echo "source /opt/ros/humble/setup.bash" >> .bashrc

sudo apt install python3-pip

pip install --user -U empy==3.3.4 pyros-genmsg setuptools
```

2.	Installer les outils PX4, **Gazebo**.

> Qu’est-ce que PX4 ? Quels sont les outils proposés par ce dernier qui nous seront utiles ?
> 

```bash
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
cd PX4-Autopilot/
```

1. Installer le client XRCE

> Qu’elle est la fonction du client XRCE ? Comment pourriez vous appeler ce type de logiciel ?
> 

```bash
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make -j8
sudo make install
sudo ldconfig /usr/local/lib/
```

Lancer le logiciel :

```bash
MicroXRCEAgent
```

> Le client XRCE ne se lance pas. Pourquoi ? Qu’elles sont les paramètres à rajouter dans notre cas ? Que permette les autres paramètres ?
> 

Dans un nouveau terminal lancer la simulation :

```bash
make px4_sitl gz_x500
```

Vous pouvez maintenant fermer la simulation.

## Partie 2 : Mise en place d’une workspace ROS

Créez une workspace ROS et clonez les paquets suivants :

```bash
mkdir -p ~/ws_sensor_combined/src/
cd ~/ws_sensor_combined/src/
```

> Quels sont les fonctions de chaque package ?
> 

```bash
git clone https://github.com/PX4/px4_msgs.git
git clone https://github.com/PX4/px4_ros_com.git
```

Vous pouvez maintenant compiler la workspace en utilisant la commande :

```bash
cd ..
colcon build
```

> Le package px4_msgs ne compile pas. Pourquoi ?
> 

> Quels sont les différents fichiers à “source” avant de lancer le node ? Comment lancer le node ?
> 

> Une fois le node lancé, il ne se passe rien. Pourquoi ? Une fois ce problème résolu, qu’elles sont les informations renvoyées par le node ?
> 

**Étape 2 : Création d’un package “OffBoard”**

**Partie 1 :** Vous pouvez couper le node lancé à l’étape précédente afin de lancer le node suivant :

```bash
ros2 run px4_ros_com offboard_control
```

> Encore une fois, il ne se pass rien. Pourquoi ? (logiciel à installer)
> 

Si ce dernier ne veut pas se lancer, il est possible qu’il soit nécessaire d’installer libfuse2.

Vous pouvez relancer le node. Maintenant, si vous regardez dans la simulation, vous pouvez voir le drone décoller et rester statique à environ 5m.

**Partie 2 : Génération de trajectoire**

A l’aide de la documentation, mettez en place un node offboard qui permet au drone de suivre une trajectoire arbitraire.

[https://docs.px4.io/main/en/ros2/offboard_control.html](https://docs.px4.io/main/en/ros2/offboard_control.html)

**Partie 3 : Avoidance [BONUS]**

Ajoutez un lidar 2D au modèle du drone et modifiez le script précédemment créé afin de faire de l’éviction d’obstacle.

Bonne chance.