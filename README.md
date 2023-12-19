# Vol en intérieur avec ROS

## Prérequis

* Lycée et +
* Notions de Python et commandes dans un terminal
* Clef USB bootable avec l'environnement ROS

## Diapositives

[../vol_drone_interieur_ros.pdf]()

## Quelques alias
Les alias sont des racourcis qui nous permettent d'éxectuer des commandes recurentes plus rapidement. Exemple ?

```bash
alias sb="source ~/.bashrc"
```

## Travaux pratiques

## A. Simulation
Pour mieux comprendre comment fonctionne l'assemblage ROS & drone, nous allons dans un premier temps essayer de faire voler un drone dans un environnement de simulation. Cet environnement de simulation permet de tester nos package ROS sans avoir de perte ou de casse de matériel. 
Dans un premier temps, il vous ait demandé de mettre en place l'environnement de simulation et de réussir à faire décoler le drone à environ 2m de hauteur et rester statique. Ensuite, nous ajouterons un package qui permet de simuler un système de MOtion CAPture (MOCAP). Une fois ce package ajouté, nous allons désactiver le GPS du drone et vérifier que ce dernier peut toujours décoller normalement. Enfin, nous ajouterons un package qui permet de faire de l'avoidance. Ce package se base sur l'ajout de différent capteurs sur le drone. La partie pratique de l'avoidance est en bonus.

### 1. Installer ROS
Avant de pouvoir commencer, nous devons mettre en place notre environnement de développement ROS. Nous allons donc installer ROS dans sa version `noetic` ainsi que Gazebo et Rviz. Gazebo est un logiciel de simulation d'environnement. C'est dans ce logiciel que nous pourrons voir notre drone évoluer lors des différentes simulations. Rviz est un logiciel d'analyse qui nous permet de debuguer notre système lors des simulations.

Une fois l'installation de ROS terminée, nous pouvons vérifier notre travail avec la commande suivante :
```bash
source /opt/ros/noetic/setup.bash
```

Afin de se faciliter le travail et d'avoir à taper cette commande tout le temps, nous pouvons l'ajouter à notre fichier `~/.bashrc` :

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```

> Quel type d'alias pouvons nous créer ici ?

Maintenant que nous avons installer le coeur de ROS, nous pouons installer ses dépendances. Ces dernières vont nous permettre de créer et gérer notre environnement de développement. Pour les installer, tapez la commande suivante.

```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

Il est alors possible d'initialiser `rosdep` avec les commandes suivantes :

> Que permet de faire la commande `rosdep` ? Pourquoi est il nécessaire d'utiliser les droits super-utilisateur ?

```bash
sudo rosdep init
rosdep update
```

Une fois ceci fait, nous allons installer les paquets qui permettent de créer la `workspace` :

```bash
sudo apt install ros-noetic-catkin python3-catkin-tools -y
sudo ln -s /usr/bin/python3 /usr/bin/python
```

Nous allons maintenant pouvoir installer un emulateur pour le controleur de vol du drone.

### 2. Installer PX4
Maintenant que nous avons un environnement ROS opérationnel, nous devons télécharger le firmware PX4 qui possède des outils permettant de simuler entièrement le controleur de vol (Pixhawk). Ainsi, nous pourrons avoir les mêmes interfaces de pilotage que dans la réalité.

Dans un premier temps, nous devons récupérer le repertoire sur Github et ensuite nous devons le compiler :

```bash
cd ~

git clone -b v1.13.2 https://github.com/PX4/PX4-Autopilot.git --recursive /home/$USER/PX4-Firmware/
```

<details>
<summary>Avec l'aide de la documentation, quelles sont les commandes que nous devons ensuite executer pour compiler le firmware ?</summary>

```bash
./PX4-Autopilot/Tools/setup/ubuntu.sh
echo 'source ~/PX4-Autopilot/Tools/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default' >> /home/$USER/.bashrc
echo 'export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot' >> /home/$USER/.bashrc
echo 'export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot/Tools/sitl_gazebo' >> /home/$USER/.bashrc
echo 'export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/usr/lib/x86_64-linux-gnu/gazebo-9/plugins' >> /home/$USER/.bashrc

source ~/.bashrc

cd ~/PX4-Autopilot
make px4_sitl_default gazebo
```

> Quel argument peut-on rajouter à la commande `make` pour espérer une compilation plus rapide ?

</details>
<br>
Une fois la dernière commande lancée, la compilation du firmware va démarrer. Ensuite, une simulation va commencer. Vous pouvez regarder le resultat de la simulation dans la fenêtre de Gazebo.


> Normalement, d'après la documentation, au lancement de la simulation le drone doit décoller. Pourquoi ce n'est pas le cas ?

Vous pouvez alors télécharger et lancer QGroundControl qui est le logiciel qui permet de communiquer avec le drone :

```bash
wget https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
chmod +x ./QGroundControl.AppImage
./QGroundControl.AppImage
```

Une fois ce dernier lancé, il va automatiquement détecter le drone dans la simulation et se connecter.

> Selon vous, comment pouvons nous résoudre le problème de radiocammande ?

Vous pouvez maintenant fermer la simulation en tapant `Ctrl+C` dans le terminal où vous l'avez lancée. Nous allons maintenant essayer de lier la simulation du firmware et ROS. 

### 3. "Offboard Flight Mode"
Notre environnement de développement est enfin opérationel. Nous allons maintenant créer une workspace qui va accueillir les différents packages ROS dont nous avons besoin et ceux que nous allons créer.

Pour créer la workspace, nous devons dabord créer un dossier et ensuite dire à ROS d'initialisé la workspace :

```bash
mkdir -p ~/mocapfly_ws/src
cd ~/mocapfly_ws
catkin init
wstool init src
```

Ensuite, on installe MAVLink. 

> Quel est le rôle de MAVLink ?

```bash
rosinstall_generator --rosdistro noetic mavlink | tee /tmp/mavros.rosinstall
```

> Quel est le rôle de MAVROS ?

```bash
rosinstall_generator --rosdistro noetic --upstream mavros | tee -a /tmp/mavros.rosinstall
```

On merge la WS puis on installe les packages :

```bash
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src -j8
rosdep update
rosdep install --from-paths src --ignore-src -y --rosdistro noetic
```

Nous pouvons maintenant build la WS avec catkin :

```bash
catkin build
```

Ensuite, on charge les outils de la WS :

```bash
source devel/setup.bash
```

Nous allons maintenant créer notre premier package qui permet de passer le FCU en mode OFFBOARD. Ceci, permettra à ROS de devenir le "pilote" du FCU. Nous pourrons donc créer des missions grâce à ROS.

> Quelles sont les particularités du mode OFFBOARD ?

On se rend dans la WS :

```bash
roscd  # Should cd into ~/mocapfly_ws/devel
cd .. 
cd src
```

On créé le nouveau package :

```bash
catkin_create_pkg offboard_py rospy
```

Ensuite, on rebuild la WS, on recharge les outils et nous pouvont commencer à coder :

```bash
cd .. # Assuming previous directory to be ~/mocapfly_ws/src
catkin build
source ../devel/setup.bash
```

> Le paquet `offboard_py` à t'il bien été compilé ?

On se rend dans le dossier du package et ensuite on créé un dossier pour stocker notre package : 

```bash
roscd offboard_py
mkdir scripts
cd scripts
```

Dans un premier temps on créé le fichier du controller et on le rend executable :

```bash
touch offb_node.py
chmod +x offb_node.py
```

Ensuite, on utilise le code suivant pour permettre le passage en mode OFFBOARD :

```bash
vim offb_node.py # you may use $ code . 
```

<details>
<summary>A l'aide de la documentation, écrivez un package qui permet de faire passer le drone en OFFBOARD et de le faire décoller.</summary>
Le code :

```python
#! /usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

current_state = State()

def state_cb(msg):
    global current_state
    current_state = msg


if __name__ == "__main__":
    rospy.init_node("offb_node_py")

    state_sub = rospy.Subscriber("mavros/state", State, callback = state_cb)

    local_pos_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    
    rospy.wait_for_service("/mavros/cmd/arming")
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)    

    rospy.wait_for_service("/mavros/set_mode")
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)
    

    # Setpoint publishing MUST be faster than 2Hz
    rate = rospy.Rate(20)

    # Wait for Flight Controller connection
    while(not rospy.is_shutdown() and not current_state.connected):
        rate.sleep()

    pose = PoseStamped()

    pose.pose.position.x = 0
    pose.pose.position.y = 0
    pose.pose.position.z = 2

    # Send a few setpoints before starting
    for i in range(100):   
        if(rospy.is_shutdown()):
            break

        local_pos_pub.publish(pose)
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True

    last_req = rospy.Time.now()

    while(not rospy.is_shutdown()):
        if(current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
            if(set_mode_client.call(offb_set_mode).mode_sent == True):
                rospy.loginfo("OFFBOARD enabled")
            
            last_req = rospy.Time.now()
        else:
            if(not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if(arming_client.call(arm_cmd).success == True):
                    rospy.loginfo("Vehicle armed")
            
                last_req = rospy.Time.now()

        local_pos_pub.publish(pose)

        rate.sleep()
```

</details>
<br>

Maintenant que nous avons créé le package nous devons créer un launch file :

```bash
roscd offboard_py
mkdir launch
cd launch 
touch start_offb.launch
chmod +x start_offb.launch
```

Pour le fichier `start_offb.launch` mettre le code suivant :
*Attention, `gcs_bridge` doit être configuré en fonction de vos paramètres réseaux.*

```xml
<?xml version="1.0"?>
<launch>
	<!-- Bridge to get QGroundControl UDP link *to configure* -->
	<node pkg="mavros" type="gcs_bridge" name="qgroundcontrol" output="screen" args="_gcs_url:='udp://@127.0.0.1'"/>
	<!-- Include the MAVROS node with SITL and Gazebo -->
	<include file="$(find px4)/launch/mavros_posix_sitl.launch">
	</include>

	<!-- Our node to control the drone -->
	<!-- Écrivez le code nécessaire pour démarrer votre node -->
</launch>
```

Nous devons maintenant exporter quelques variables d'environnement afin de pouvoir démarrer la simulation correctement. Pour exporter ces variables nous allons éditer le fichier `~./.bashrc` et ajouter les lignes suivantes à la fin du fichier :

*Note : Si vous l'avez déjà fait à l'étape précédente, contentez-vous de source le fichier `.bashrc`*

> Expliquez ce que permettent de faire les commandes suivantes.

```bash
source ~/PX4-Autopilot/Tools/simulation/gazebo/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default
source /opt/ros/noetic/setup.bash
source ~/mocapfly_ws/devel/setup.bash
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot/Tools/sitl_gazebo
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/usr/lib/x86_64-linux-gnu/gazebo-9/plugins
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/usr/local/lib/gazebo
```


Nous pouvons ensuite lancer la simulation et vérifier le comportement du drone :

```bash
roslaunch offboard_py start_offb.launch
```

> Maintenant que nous avons vu comment donner le controle du drone à ROS, modifiez le package Offboard pour que le drone décolle et fasse la trajectoire de votre choix avant d'attérir.

### 4. MOCAP & UAV
ROS est maintenant en mesure de pouvoir communiquer avec le drone et de pouvoir le piloter. Cependant, pour le moment il utilise un "faux" GPS qui permet de positionner le drone correctement et de le déplacer. Dans un environnement clos, il n'est pas possible pour le drone de recevoir les données GPS qui permettent au drone de ce déplacer. Dans ce cas, nous pouvons utiliser un système de motion capture qui va traquer un tag disposé sur le drone. Le système de MOCAP nous donne alors  la position du drone dans l'espace. Nous devons fournir cette position au drone pour qu'il puisse utiliser ces données à la place du GPS, et ainsi pouvoir voler.

Dans un premier temps, il faut installer ce package. Pour ce faire, placer vous dans le dossier `src` de votre workspace et taper les commandes suivantes :

```bash
cd ~/mocapfly_ws/src
git clone https://github.com/Kariboo-Corp/mocap_simulator.git
```

Une fois le paquet installer, il est nécessaire de modifier notre launch file afin de lancer le node `mocap_simulator` lors du démarrage de la simulation. Pour ce faire, il faut retourner dans le package `offboard_py` et éditer le fichier `start_offb.launch`.

Il faut y rajouter les lignes suivantes qui permettent de lancer le node MOCAP et de publier les positions du drone :

```xml
<node pkg="mocap_simulator"
	type="mocap_simulator_node.py"
	name="vrpn_client_node"
	output="screen">
	<param name="frame_rate" value="100"/>
	<param name="publish_tf" value="true"/>
	<param name="fixed_frame_id" value="local_origin"/>
	<rosparam param="model_list">[]</rosparam>
</node>
```

> Quel sont les fonctions des différents paramètres ?

Enfin, nous allons rajouter la ligne suivante :

```xml
<node pkg="topic_tools" type="relay" name="mocap_relay_mavros" output="screen" args="/vrpn_client_node/iris/pose /mavros/vision_pose/pose"/>
```

> Que permet de faire cette ligne ?

Installez le paquet suivant :

```bash
sudo apt-get install ros-kinetic-vrpn-client-ros
````

> Quel est la fonction de ce paquet ?

Vous pouvez maintenant source tous les fichiers :

```bash
source /opt/ros/noetic/setup.bash
source ~/mocapfly_ws/devel/setup.bash
source ~/.bashrc
```

Nous pouvons maintenant relancer la simulation et aller sur QGroundControl pour désactiver le GPS du drone et dire à ce dernier d'utiliser les données du système MOCAP.
Pour désactiver le GPS, il faut aller dans "Vehicle Setup" puis dans "Parameters" et chercher `has_gps` est passer ce paramètre à `disable`.

Vous devez maintenant `restart` complètement la simulation pour que le nouveau paramètre soit appliqué. 

> Le drone refuse de passer en mode OFFBOARD. Pourquoi ? Quelles modifications faut-il faire pour autoriser l'armement ?

<details>
<summary>Solution</summary>
Nous avons désactivé le GPS du drone, cependant, nous n'avons pas indiqué au drone d'utiliser une autre source de données de position. Pour ce faire, il faut changer le paramètre `EKF2_AID_MASK` à 24.
</details>

## B. Pour aller plus loin !

Si vous avez réussi à faire voler votre drone correctement dans la simulation avec le système de positionnement MOCAP, alors vous pouvez essayer d'intégrer le Package ROS : [PX4-Avoidance](https://github.com/PX4/PX4-Avoidance).

Bon courage !