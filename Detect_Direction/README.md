

Pour lancer le script se mettre dans le dossier gps_files et taper python3 Detect_Direction.py

Je cherche l'angle entre 2 points start et end.

dans le cas STPI -> GEI : start > end   on garde le trajectoire 
dans le cas GEI -> STPI : start < end   on inverse le trajectoire 

Je calcule la direction actuele de la voiture = le droite de 2 points (avant et derrière) de voiture; si la voiture au début ou près de bout de trajet donc prend 2 point devant la voiture
Je calcule le direction à venir entre 2 points de départ et d'arrivée
--> l'angle entre 2 direction .
IF cet angle > 5 THEN tourner à droite
    ELSIF < 5 THEN tourner à gauche
    ELSE aller tout droit 
