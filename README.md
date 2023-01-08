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
   
    -qui affiche toutes les commandes disponibles




