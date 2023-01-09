# TP Acquisition & Commande

![e n s e a-msc](https://user-images.githubusercontent.com/94643384/211223916-f7f0b5c8-966f-4bcb-b7ef-f247f3367a62.svg)
![control-power](https://user-images.githubusercontent.com/94643384/211193128-17471c6e-8b9d-458f-86f8-35a7823f964f.svg)




## Auteurs 

* **Pierre Bellanger** _alias_ [@XPXBX](https://github.com/XPXBX)
* **Dylan Kararji** _alias_ [@DylanKM](https://github.com/DylanKM)

## Date

**MSC**  _S9 2022-2023_ 

## Lien documentation Doxygen

## Sommaire 

[1. Hardware](#1-Hardware)

[2. Objectifs du TP](#2-Objectifs-du-TP)

[3. Planning](#3-Planning)

[4. Configuration des pins](#4-Configuration-des-pins)

[5. Console UART](#5-Console-UART)

[6. Commande MCC Basique](#6-Commande-MCC-Basique)


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
   
* **help**
  - _Permet d'afficher toutes les commandes disponibles sur le shell._
 
 ```
 
			if(strcmp(argv[0],"help")==0)
			{
				HAL_UART_Transmit(&huart2, "Voici la liste des commandes:\r\n", sizeof("Voici la liste des commandes:\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "  - help\r\n", sizeof("  - help\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "	>> Affiche la liste des commandes\r\n", sizeof("	>> Affiche la liste des commandes\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "  - set PA5 (0 ou 1)\r\n", sizeof("  - set PA5 (0 ou 1)\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "	>> Allume ou eteint la LED\r\n", sizeof("	>> Allume ou eteint la LED\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "  - pinout\r\n", sizeof("  - pinout\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "	>> Affiche la liste des PIN utilises et leurs utilisations\r\n", sizeof("	>> Affiche la liste des PIN utilises et leurs utilisations\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "  - start\r\n", sizeof("  - start\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "	>> Demarre la generation de PWM\r\n", sizeof("	>> Demarre la generation de PWM\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "  - stop\r\n", sizeof("  - stop\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "	>> Stop la generation de PWM\r\n", sizeof("	>> Stop la generation de PWM\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "  - alpha\r\n", sizeof("  - alpha\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "	>> Modifie la valeur du rapport cyclique Alpha entre 0 et 1\r\n", sizeof("	>> Modifie la valeur du rapport cyclique Alpha entre 0 et 1\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "  - reset\r\n", sizeof("  - reset\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "	>> Reinitialise le systeme\r\n", sizeof("	>> Reinitialise le systeme\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "  - speed = XXXX\r\n", sizeof("  - speed = XXXX\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "	>> Regle la vitesse à XXXX [-3000 a 3000] RPM\r\n", sizeof("	>> Regle la vitesse à XXXX [-3000 a 3000] RPM\r\n"), HAL_MAX_DELAY);
			}

```
  
* **pinout**
  - _Renvoie la liste des broches utilisée ainsi que leur fonctionnalité._

```
			else if(strcmp(argv[0],"pinout")==0)
			{
				HAL_UART_Transmit(&huart2, "liste des PIN utilises :\r\n", sizeof("liste des PIN utilises :\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "	PA5  : Allumer/eteindre la LED", sizeof("	PA5  : Allumer/eteindre la LED\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "	PA8  : PWM 1\r\n", sizeof("	PA8  : PWM 1\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "	PA9  : PWM 2\r\n", sizeof("	PA9  : PWM 2\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "	PA11 : PWM 1N\r\n", sizeof("	PA11 : PWM 1N\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "	PA12 : PWM 2N\r\n", sizeof("	PA12 : PWM 2N\r\n"), HAL_MAX_DELAY);
				HAL_UART_Transmit(&huart2, "	PC3  : Reset\r\n", sizeof("	PC3  : Reset\r\n"), HAL_MAX_DELAY);
			}

```

* **start**
  - _Permet d'allumer le moteur CC_

```

			else if(strcmp(argv[0],"start")==0)
			{

				Start_PWM();
				CCR_Alpha(50);
				
			}
```

* **stop**
  - _Permet d'arrêter le moteur_ 

```

			else if(strcmp(argv[0],"stop")==0)
			{

			Stop_PWM();

			}

```

* **set PA5**
  - _Une fonction qui allume/éteint la LED connectée à la pin PA5 de notre carte_

```


			else if(strcmp(argv[0],"set")==0)
			{
				if(strcmp(argv[1],"PA5")==0)
				{
					HAL_GPIO_WritePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin, atoi(argv[2]));
					sprintf(uartTxBuffer,"Switch on/off led : %d\r\n",atoi(argv[2]));
					HAL_UART_Transmit(&huart2, uartTxBuffer, 32, HAL_MAX_DELAY);
				}
				else
				{
					HAL_UART_Transmit(&huart2, cmdNotFound, sizeof(cmdNotFound), HAL_MAX_DELAY);
				}

			}

```



***NB : certains codes seront explicités et expliqués au cours du Readme***

## 6. Commande MCC Basique

### 6.1 Génération de 4 PWM

On genère quatre PWM pour le driver, avec un tel cahier des charges

* Fréquence de la PWM : 16kHz
* Temps mort minimum : 2µs
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


<p float="justify">
  <img src="https://user-images.githubusercontent.com/94643384/211208500-9e54ed38-b1c3-4bb3-9155-098a528c1b31.PNG" width="300" /> <img src="https://user-images.githubusercontent.com/94643384/211208502-1c368b54-266d-43d4-b2a7-879729b46e8b.PNG" width="300" /> <img src="https://user-images.githubusercontent.com/94643384/211208505-e4848d8a-0f4d-48e9-956e-a89a878b1ee4.PNG" width="300" />
</p>



**Pour le choix de la valeur du DeadTime :**

On nous demande une valeur de DeadTime de 2µs, ainsi on doit choisir la configuration dans le fichier.ioc du Dead Time Generator setup entre 0 et 255.

D'après la documentation dans le cours, on explique la configuration 

<p align="center">
<img width="441" alt="configDT" src="https://user-images.githubusercontent.com/94643384/211220035-980cf4f8-d915-4eee-93ea-9f444945df31.PNG">
</p>

_Ici notre fréquence d'horloge est de 170MHz, DT = 2µs, calculons DTG[6:0] si DTG[7:5]= 0_

DTG[6:0] = DT/T_clock = 2.10^-6*170.10^6 = 340 > 127 (valeur décimale maximun sur 7 bits) 

_On ne pas écrire DTG sur ses 7 bits avec cette configuration, il faut donc passer la suivante_

_si DTG[7:5]= 10x,_ 

DTG[5:0] = DT/(2*T_clock) - 64 = 106 > 63

_De même, on passe à la suivante,_

_si DTG[7:5] =110,_

DTG[4:0] = DT/(8*T_clock) - 32 = 10,5 < 31

_Le nombre peut être écrit sur ses bits descriptifs ainsi on prendre la configuration avec DTG[7:5] = 110_

**D'où DTG[7:0] = 0x11010011 = 210 (pour un DeadTime légèrement supérieur à 2µs)**

<p align="center">
<img width="244" alt="DeadTime" src="https://user-images.githubusercontent.com/94643384/211284990-d312b118-be9a-496a-b16d-d3c9f6f8b538.PNG">
</p>

### 6.2 Prise en main du hacheur

On branche ensuite la  carte Nucleo au PCB permettant de la lier avec le hacheur.  Il s'agit d'un hacheur de 3 phases, on le branche seulement à deux bras (parmi Y,R ou B)

On relie ainsi les **sorties PWM, PA12, PA11, PA9 et PA8** aux bras, en reliant les PWM aux entrées "TOP" et les PWM complémentaires aux entrées "BOTTOM".

On note à la pin 33 d'après la datasheet du Power Module, une commande **ISO_RESET** qui commande l'allumage du hacheur. La séquence de détection est un front descendant, l'idée est donc de changer l'état d'un GPIO appelé **ISO_RESET** initialement à l'état **HIGH**, et de le passer à l'état **LOW** 

_On a donc le code pour la fonction Reset _ 

* **Reset**

```

			else if (strcmp(argv[0],"reset")==0)
			{
				HAL_GPIO_WritePin(ISO_RESET_GPIO_Port, ISO_RESET_Pin, 1);
				HAL_Delay(1);
				HAL_GPIO_WritePin(ISO_RESET_GPIO_Port, ISO_RESET_Pin, 0);
			}


```


### 6.3. Commande start

Le hacheur à besoin d'une séquence d'amorçage pour obtenir une tension de sortie.

La fonction **Start** doit prendre en compte : 

* Un allumage via le shell à la commande "start"
* Un allumage par une pression sur le bouton bleu sur notre carte 

Avec ceci, on met en route les PWM qui auront un DutyCycle à 50%, soit une tension de sortie nulle pour le moteur, il sera donc prêt à tourner.

* **CCR_Alpha(alpha)**
  - Selectionne le DutyCycle des PWM

```
void CCR_Alpha(int alpha)
{
	Alpha1 = (alpha*TIM1 -> ARR)/100;
	Alpha2 = TIM1 -> ARR-Alpha1;
	TIM1->CCR1 = Alpha1;
	TIM1->CCR2 = Alpha2;
}
```



### 6.4. Premiers tests

_On branche ensuite le moteur pour effectuer des tests de rotation avec de telles consigne dans cet ordre défini :_

* Rapport cyclique de 50%
* Rapport cyclique de 70%
* Rapport cyclique de 100%
* Rapport cyclique de 0%

Le moteur se coupe lors du passage de 50% à 70% ainsi que de 70% à 100%. En effe à cause des fortes variations de courants lors de changement brusques de consigne, le hacheur se met en sécurité. le témoin _HALL OVERCURRENT_ s'allume en rouge.


### 6.5. Définition de la vitesse

Pour des raisons pratiques, on souhaite maintenant commander le moteur avec une consigne de vitesse avec la fonction **"speed"**
Il faut par protection réaliser un test afin de limiter la vitesse à 3000 tr/min.

Ensuite, on converti la valeur en commande d'alpha, avec une fonction **Conversion_alpha(int)**

* **Conversion_Alpha**

```
int Conversion_Alpha(int vitesse)
{
	int ValeurAlpha = ((vitesse + 3000)/60);
	CCR_Alpha(ValeurAlpha);
	return ValeurAlpha;
}

```
**Expliquer la conversion**


## 7. Capteurs de courant et position

Objectif :

* Mettre en place la mesure de courant à partir de la résistance de shunt
* Mettre en place la mesure de position du moteur à partir de la roue codeuse

### 7.1. Mesure de courant

_Pour faire l'asservissement en courant, il est nécessaire de faire des mesures à chaque période de PWM sur les phases utilisées pour commander le moteur._

On relève la tension des pins 16 et 35, qui correspondent aux capteurs de courant des broches Red et Yellow.

**A Relever**

On utilise ensuite un ADC sur la Nucleo, qui va permet de convertir une valeur analogique en une valeur numérique afin d'obtenir la valeur moyenne de courant de chaque branche. 

Afin de traiter plusieurs informations, on utilise le DMA. Le DMA (Direct Memory Access) est un Mécanisme qui permet l'accèss direct à la mémoire vive sans passer par le processeur permettant ainsi une accélération assez importante des performances pour les bus d'entrées/sorties.

**A EXPLIQUER Configuration sur STM**


### 7.2 Mesure de la vitesse

A partir de l'encodeur du moteur, on récupère la vitesse de rotation du moteur





## 8. Asservissement

Une fois tous les mesures du courant et de vitesse obtenues, il nous reste à réaliser les asservissements :
* Asservissement en courant en veillant à ne pas dépasser le courant maximum : le hacheur va limiter cette valeur.
* Asservissement en vitesse.

