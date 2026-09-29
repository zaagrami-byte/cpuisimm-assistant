# Frontière IA / Robot Core

    AI = compréhension + conversation + outils haut niveau
    ROBOT CORE = contrôle physique + sécurité + ROS 2 + Nav2
    AI → demande        ROBOT CORE → décide comment l'exécuter en sécurité

## L'IA peut
Comprendre, répondre, parler, lire l'état/la pose, demander `NavigateToPose`, annuler, demander l'arrêt
(`/emergency_stop`), observer et signaler des anomalies.

## L'IA ne peut jamais
Publier `/cmd_vel` ; toucher PWM, gains, footprint, Nav2, costmaps, collision_monitor, safety_node, TF,
odom, LiDAR, SLAM ; exécuter une commande shell ; réarmer l'arrêt d'urgence ; appeler un outil hors
liste blanche (`ai/tools/__init__.py`).

## Calibration
"Calibre le robot" → l'IA explique que c'est une opération d'ingénierie et ne modifie RIEN.
Elle peut relayer des observations (ex. vitesse mesurée < vitesse demandée) ; la correction se fait
uniquement dans `ros2_ws/` (`robot_params.yaml`, `nav2_params.yaml`…) par un ingénieur.

## Règle de contribution
Aucun code sous `ai/` ne doit importer de message `geometry_msgs/Twist` en publication, ni écrire
dans `ros2_ws/`. Un nouvel outil = un `Tool` explicite dans le registre + un test dans `ai/tests/`.
