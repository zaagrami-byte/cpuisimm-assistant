# ISIMM Assistant — procédure de démarrage et de sauvegarde

Ce document décrit le démarrage du robot sur le Raspberry Pi et de l'interface opérateur sur le PC, ainsi que les vérifications à faire avant utilisation.

> **État de référence :** branche `feature/operator-ui`. La configuration doit être considérée comme fonctionnelle uniquement après les contrôles ci-dessous. Cette documentation ne garantit pas l'absence de défauts : le robot doit être testé à vitesse réduite, roues dégagées, avant tout déplacement au sol.

## 1. Architecture et règles importantes

- **Raspberry Pi 4** (`robotcpu-pi`) : ROS 2 Jazzy, capteurs, odométrie RF2O, SLAM Toolbox, Nav2, multiplexeur AUTO/MANUAL, sécurité et moteurs.
- **PC Ubuntu** (`ca2025rt`) : interface opérateur `operator_ui`, RViz et traitements IA.
- Dépôt sur les deux machines : `~/isimm`.
- Workspace ROS 2 : `~/isimm/ros2_ws`.
- Carte de localisation : `/home/isimmassistant/isimm/maps/isimm_main` (les fichiers `.posegraph` et `.data` doivent être présents).
- **Ne pas lancer deux fois** `navigation.launch.py` ou `robot.launch.py`. Une seule instance de chaque pile doit fonctionner.
- Ne pas modifier les paramètres ou la chaîne de commande pendant un essai sans sauvegarder l'état actuel et vérifier le changement.
- Chaîne de commande navigation : Nav2 → velocity smoother → multiplexeur AUTO/MANUAL → collision monitor → `/cmd_vel` → safety node → `/safe_cmd_vel` → contrôleur moteur.
- L'interface publie `/cmd_vel_teleop`; elle ne doit jamais commander directement les moteurs.

## 2. Démarrage du robot — Raspberry Pi

Ouvrir **un terminal sur le Pi** :

```bash
cd ~/isimm
source /opt/ros/jazzy/setup.bash
source ~/isimm/ros2_ws/install/setup.bash
```

Vérifier que les périphériques série sont présents :

```bash
ls -l /dev/isimm_arduino /dev/isimm_rplidar
ls -l /dev/isimm_esp32
```

Les deux premiers sont requis par le diagnostic. L'ESP32/IMU peut être absent et être signalé comme avertissement selon la configuration.

Vérifier que la carte de localisation existe :

```bash
ls -lh /home/isimmassistant/isimm/maps/isimm_main.posegraph
ls -lh /home/isimmassistant/isimm/maps/isimm_main.data
```

Lancer **une seule fois** la navigation :

```bash
ros2 launch isimm_robot navigation.launch.py map_file:=/home/isimmassistant/isimm/maps/isimm_main
```

Laisser ce terminal ouvert. Pour arrêter proprement la pile, utiliser `Ctrl+C` et attendre l'arrêt des nœuds avant de relancer.

### Vérifier la navigation

Dans **un deuxième terminal sur le Pi** :

```bash
source /opt/ros/jazzy/setup.bash
source ~/isimm/ros2_ws/install/setup.bash
cd ~/isimm
bash scripts/check_robot.sh navigation
```

Avant de déplacer le robot, rechercher et corriger tous les `FAIL`. Examiner aussi les `WARN`. Le script est un diagnostic en lecture seule.

Contrôles manuels utiles :

```bash
ros2 node list | sort | uniq -c | awk '$1>1'
ros2 topic echo --once /cmd_vel_mode
ros2 topic echo --once /motor_status
ros2 topic echo --once /safety_state
ros2 topic info /cmd_vel
ros2 topic info /safe_cmd_vel
```

La première commande ne doit afficher aucune ligne de nœud dupliqué. En navigation, vérifier que `/cmd_vel` a un seul publisher (collision monitor), `/safe_cmd_vel` a un seul publisher et que `/motor_status` vaut `OK`. Au repos, `NO_CMD_YET` ou `CMD_TIMEOUT` peut être un état de sécurité normal.

## 3. Démarrage de l'interface opérateur — PC Ubuntu

Sur le PC, vérifier que le dépôt contient le dossier `operator_ui` et synchroniser la même branche/version que celle utilisée sur le Pi :

```bash
cd ~/Desktop/isimm
git status --short --branch
git switch feature/operator-ui
git pull --ff-only origin feature/operator-ui
```

Ne pas exécuter `git switch` avec des modifications locales non sauvegardées. Si le dépôt se trouve ailleurs, adapter uniquement le chemin `cd`.

Préparer l'environnement Python (à faire une première fois, puis le réutiliser) :

```bash
cd ~/Desktop/isimm
source /opt/ros/jazzy/setup.bash
source ~/Desktop/isimm/ros2_ws/install/setup.bash
python3 -m venv --system-site-packages operator_ui/.venv
source operator_ui/.venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r operator_ui/requirements.txt
```

À chaque démarrage ultérieur, activer l'environnement et lancer les vérifications :

```bash
cd ~/Desktop/isimm
source /opt/ros/jazzy/setup.bash
source ~/Desktop/isimm/ros2_ws/install/setup.bash
source operator_ui/.venv/bin/activate
python -m operator_ui --check-gamepad
```

Si le contrôle est satisfaisant, démarrer l'interface dans ce terminal :

```bash
python -m operator_ui
```

Garder le terminal ouvert. Arrêter l'interface avec `Ctrl+C`.

### Réseau ROS 2 entre PC et Pi

Les deux machines doivent être sur le même réseau et utiliser le même domaine DDS. Si `ROS_DOMAIN_ID` n'est pas défini sur les deux, ROS 2 utilise normalement le domaine par défaut `0`. Vérifier sur **chaque machine** :

```bash
echo "ROS_DISTRO=$ROS_DISTRO"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-0}"
ros2 node list
```

Si les machines ne se voient pas, vérifier d'abord le Wi-Fi/LAN, le pare-feu, les variables ci-dessus et la présence de nœuds ROS sur le Pi. Ne pas changer le domaine sur une seule machine.

## 4. Paramètres à conserver

Les fichiers de configuration versionnés dans le dépôt sont la référence. Avant toute modification, sauvegarder les changements dans Git et noter la date, la raison, ainsi que les résultats des tests.

Fichiers importants :
- `ros2_ws/src/isimm_robot/config/robot_params.yaml` — paramètres robot, sécurité et multiplexeur.
- `ros2_ws/src/isimm_robot/config/nav2_params.yaml` — paramètres Nav2.
- `ros2_ws/src/isimm_robot/config/slam_localization_params.yaml` — localisation SLAM Toolbox.
- `ros2_ws/src/isimm_robot/launch/robot.launch.py` — pile de base.
- `ros2_ws/src/isimm_robot/launch/navigation.launch.py` — navigation et chaîne de commande.
- `operator_ui/config/teleop.yaml` — vitesse et seuils de l'interface manuelle.
- `scripts/check_robot.sh` — diagnostic en lecture seule.
- `maps/isimm_main.posegraph` et `maps/isimm_main.data` — carte de localisation.

Les fichiers sous `ros2_ws/install/` sont générés par compilation : ne pas les modifier manuellement. Après une modification du code source ou d'un fichier de paramètres installé par le paquet, reconstruire puis re-sourcer le workspace sur la machine concernée.

Sur le Pi, après une modification ROS :

```bash
cd ~/isimm/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-up-to isimm_robot
source install/setup.bash
```

Ne reconstruire que lorsque les sources ont changé. Si le build échoue, ne pas supprimer l'installation qui fonctionne : conserver le journal d'erreur et revenir à la dernière version Git connue comme fonctionnelle.

## 5. Créer un point de sauvegarde avant une modification

Sur **chaque machine où le dépôt est utilisé**, exécuter :

```bash
cd ~/isimm
git status --short --branch
git diff
git log -5 --oneline
```

Si les fichiers suivis ont changé et que les changements sont voulus :

```bash
git add ros2_ws/src/isimm_robot operator_ui scripts maps
git commit -m "Backup: known-good robot configuration"
git push origin feature/operator-ui
```

Vérifier les chemins avant `git add` et ne jamais ajouter de secrets, mots de passe, jetons, journaux volumineux, environnement virtuel `.venv`, ou fichiers générés. Si des changements ne concernent qu'une machine, ne pas les committer aveuglément sur l'autre.

Pour sauvegarder aussi l'état exact du dépôt sans inclure les fichiers ignorés :

```bash
cd ~
tar --exclude='isimm/.git' --exclude='isimm/**/.venv' \
    -czf "isimm-source-$(date +%F-%H%M).tar.gz" isimm
```

Cette archive copie le contenu du dossier, y compris les fichiers locaux non suivis ; vérifier son contenu et son emplacement avant de la partager. Faire une copie séparée des paramètres locaux ou fichiers ignorés s'ils sont nécessaires.

Pour noter un état Git stable, récupérer le SHA du commit :

```bash
git rev-parse HEAD
git status --short
```

Conserver le SHA dans un carnet de maintenance. Un commit Git protège le code versionné, mais ne sauvegarde pas automatiquement les périphériques, les paquets système, le firmware Arduino, les variables d'environnement ou les fichiers ignorés.

## 6. Redémarrage après coupure de courant ou arrêt

1. Démarrer le Pi et le PC, connecter les deux au même réseau.
2. Sur le Pi, lancer les commandes de la section 2, une seule fois.
3. Attendre la fin de l'initialisation et exécuter `bash scripts/check_robot.sh navigation`.
4. Sur le PC, activer l'environnement de la section 3 et exécuter `python -m operator_ui --check-gamepad`, puis `python -m operator_ui`.
5. Vérifier le statut du robot dans l'interface, la manette et les états de sécurité.
6. Faire un essai à faible vitesse avec la zone dégagée et un opérateur prêt à arrêter le robot.

Ne pas forcer le pilotage manuel si les composants requis sont en défaut ou si le statut du robot est périmé. Le timeout de la manette est volontaire : l'absence de nouvelles données doit provoquer l'arrêt de la commande.

## 7. Dépannage rapide

- **`ros2: command not found`** : exécuter `source /opt/ros/jazzy/setup.bash`.
- **Package ou exécutable introuvable** : sourcer aussi `~/isimm/ros2_ws/install/setup.bash`, puis vérifier `ros2 pkg prefix isimm_robot`.
- **Port série absent** : contrôler USB, `ls -l /dev/isimm_*`, règles udev et permissions. Ne pas remplacer un port par un autre sans l'identifier.
- **Carte introuvable** : vérifier les deux fichiers `.posegraph` et `.data` au chemin passé à `map_file`.
- **Nœuds dupliqués / CPU très élevé** : rechercher les anciens processus et arrêter proprement l'ancien launch avant tout redémarrage. Ne pas tuer des processus au hasard.
- **PC ne voit pas les topics du Pi** : vérifier le réseau, le domaine DDS, `ROS_LOCALHOST_ONLY` et les règles de pare-feu.
- **Manette non reconnue** : lancer `python -m operator_ui --check-gamepad` et vérifier le périphérique avant d'activer le mode manuel.
- **Échec du diagnostic** : conserver la sortie complète, `git status --short --branch`, `git rev-parse HEAD` et les journaux du launch pour analyser le problème.

## 8. Avant de déclarer le robot prêt

- [ ] Une seule pile de navigation est lancée.
- [ ] Aucun nom de nœud dupliqué.
- [ ] `/scan`, `/odom`, TF et carte de localisation sont disponibles.
- [ ] Les nœuds Nav2 requis sont actifs.
- [ ] `/motor_status = OK`.
- [ ] La chaîne de commande a les publishers/subscribers attendus.
- [ ] Le PC et le Pi échangent les topics ROS 2.
- [ ] L'interface détecte la manette et affiche un statut récent.
- [ ] Essai moteur à faible vitesse réussi dans une zone dégagée.
- [ ] Le SHA Git de l'état testé est noté et les modifications sont sauvegardées.

---
Dernière mise à jour documentaire : 2026-10-21.
