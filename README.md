# TP Acquisition & Commande

![control-power](https://user-images.githubusercontent.com/94643384/211193128-17471c6e-8b9d-458f-86f8-35a7823f964f.svg)


## Auteurs 

* **Pierre Bellanger** _alias_ [@XPXBX](https://github.com/XPXBX)
* **Dylan Kararji** _alias_ [@DylanKM](https://github.com/DylanKM)

## Date

**MSC**  _S9 2022-2023_ 

## Sommaire 

[1. Hardware](#1-Hardware)

[2. Objectifs du TP](#2-Objectifs-du-TP)

[3. Planning](#3-Planning)

[4. Configuration des pins](#4-Configuration-des-pins)

[5. Console UART](#5-Console-UART)


## 1. Hardware

* une carte STM-32 Nucleo-STM32G474RE
* un PCB pour relier la carte STM au driver
* un driver 3-phase Microchip
* un moteur MCC

## 2. Objectifs du TP

* _**Réaliser un shell pour commander le hacheur**_
* _**Réaliser la commande des 4 transistors du hacheur en commande complémentaire décalée**_
* _**Faire l'acquisition des différents capteurs**_
* _**Réaliser l'asservissement en temps réel**_


## 3. Planning
 
**Séance 1 :** 

Installation de GitHub Desktop et mise en place du projet GitHub TP Acquisition de commandes. 
Configuration de la Stm32 et travail intitial sur shell. 

**Séance 2 :** 

Réalisation du shell. 
Generation des 4 signaux PWM en commande complementaire decalee.
calcul et mise en place du temps mort.
verification sur oscilloscope.

**Séance 3 :**

Commande du rapport cyclique sur shell.
Mise en place du ISO_RESET.
Cablage du moteur et du hacheur.
Demarrage moteur puis test avec plusieurs valeurs de duty cycle .


## 4. Configuration des pins

Pour un bon fonctionnement, on commence soigneusement à configurer notre processeur STM. Au cour de l'avancée du TP, certains élements seront ajoutés tel que l'ADC

<p align="center">
<img width="491" alt="Config1" src="https://user-images.githubusercontent.com/94643384/211192837-d62acc56-cc75-4d3c-840b-541190ca4972.PNG">
</p>

## 5. Console UART

Dans un premier temps nous allons paramétrer la liaison UART de la carte STM32-G474RE


Elle doit nous permettre :

* d'afficher les caractères que l'on envoie 
* de traiter la chaîne de caractères rédigée lorsque que la touche "ENTER" est exécutée
* de pouvoir gerer plusieurs chaînes de caractères
* renvoyer les données des fonctions appelées

**_Voici les fonctions rédigées pour les commandes de console :_**
   
* help
  - _Permet d'afficher toutes les commandes disponibles sur le shell._
  
* pinout
  - _Renvoie la liste des broches utilisée ainsi que leur fonctionnalité._

* start
  - _Permet d'allumer le moteur CC_
  
* stop
  - _Permet d'arrêter le moteur_ 

## 6. Commande MCC Basique

### Génération de 4 PWM

On genère quatre PWM pour le driver, avec un tel cahier des charges

* Fréquence de la PWM : 16kHz
* Temps mort minimum : 2us
* Résolution minimum : 10bits.

On configure le Timer 1 sur le fichier.ioc en activant "PWM Generation"  sur CH1 & CH2 en générant aussi les PWM complémentaires (CH1N & CH2N)

Pour le choix de la fréquence, on determine le nombre du "Counter Period" avec la formule F_pwm = F_clck/(ARR+1)(PSC+1) avec F_clck = 170Mhz

On fixe le PSC à 0 donc pour que F_pwm = 16kHz, ARR+1 = 10625, ainsi **le counter period est de 5312** puisqu'il s'agit d'une **commande PWM complémentaire décalée**

#### Explication commande complémentaire décalée

La commande complémentaire décalée permet de ne créer au borne du moteur, qu'une tension continue positive ou négative, contrairement à la commande complémentaire simple qui crée une tension alternative. Cette dernière provoque de fortes variations de courant ce qui ralentit le moteur et pourrait mettre en sécurité le driver

On l'observe grâce à la comparaison des commandes vu dans le cours

<img width="418" alt="ccd50" src="https://user-images.githubusercontent.com/94643384/211207348-5cbd6708-1e1b-4ef8-87f1-914069bb14a5.PNG"> <img width="424" alt="cc50" src="https://user-images.githubusercontent.com/94643384/211207429-94eb8271-6f7d-4a61-9b27-a499f90f546b.PNG">


<img width="422" alt="ccd85" src="https://user-images.githubusercontent.com/94643384/211207445-93b20144-c78c-4ce3-b3fc-e07549aef18e.PNG"> <img width="423" alt="cc85" src="https://user-images.githubusercontent.com/94643384/211207472-0f091e2d-c1e0-4a24-b739-4d14efb13ffb.PNG">


Ainsi, on rentre dans les paramètres du _TIMER 1_  :


<p float="left">
  <img src="https://user-images.githubusercontent.com/94643384/211208500-9e54ed38-b1c3-4bb3-9155-098a528c1b31.PNG" width="300" />
  <img src="https://user-images.githubusercontent.com/94643384/211208502-1c368b54-266d-43d4-b2a7-879729b46e8b.PNG" width="300" /> 
  <img src="https://user-images.githubusercontent.com/94643384/211208505-e4848d8a-0f4d-48e9-956e-a89a878b1ee4.PNG" width="300" />
</p>

**Pour le choix de la valeur du DeadTime : **



