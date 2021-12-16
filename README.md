# RANE_SOFTWARE
This repository will store all program for the futur RANE robot from HIVE ROBOTICS compagny.

# v1.1
* intégration de redis pour une gestion plus segmenter du programme.

# TODO
* completer l'integration des encodeurs en envoyant le bon format de donné.
* faire un programme debug qui affiche l'état de toute les variables et subscriber/publisher.
* faire le programme local environnment comprehension et descision.
* ajouter la velocity du robot pour le LECD.
* faire le launcher de tout les programmes.
* quand il reçoit une commande manuel durant une phase d'autonomie, on set up un timer avec "new manuel commande"
qui va stopper les process automatique durant 1000 ms, sauf une commande special qui va tout arreter.