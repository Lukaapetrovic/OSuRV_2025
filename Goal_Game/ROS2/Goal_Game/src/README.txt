# Servo Golman ROS1 C++ Project

Ovaj projekat omogućava kontrolu jednog servo motora koji predstavlja golmana, pomoću tastature, koristeći **Raspberry Pi 2** i **ROS1 (Noetic)**. Servo motor može da se pomera ulevo i udesno, a vraća se u početni položaj kada se taster pusti.

UPUTSVO ZA POKRETANJE:

1. **Kreiraj Catkin workspace** (ako već ne postoji):

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash

2. Kloniraj ili kopiraj projekat u workspace:

cd ~/catkin_ws/src
git clone <tvoj-fork-url> servo_golman

3. Kompajliraj projekat:

cd ~/catkin_ws
catkin_make
source devel/setup.bash

4. Pokreni ROS Master:

roscore

5. Pokreni servo čvor:

rosrun servo_golman servo_node

6. Pokreni keyboard čvor u drugom terminalu:

rosrun servo_golman keyboard_node

7. Kontrola serva:

Pritiskom a → servo se pomera ulevo
Pritiskom d → servo se pomera udesno
Pustanjem tastera → servo se vraća u početni položaj
q → izlazak iz programa

---servo_node.cpp---
Pretplaćuje se na /servo_angle topic
Prima poruke tipa Int32 sa uglom serva
Koristi wiringPi i softPwm za kontrolu servo motora
Mapira ugao 0–180 na PWM signal i pomera servo

---keyboard_node.cpp---
Čita pritiske tastera sa tastature u realnom vremenu
Publikuje ugao serva na /servo_angle topic
Implementiran “hold” mehanizam: servo ostaje na pomerenom položaju dok se taster drži, vraća se u početni položaj kada se pusti
