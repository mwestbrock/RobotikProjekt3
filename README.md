# RobotikProjekt3



## Installation
Bevor sie das package installieren stellen Sie sicher, dass ROS 2 richtig installiert wurde und Sie eine funktionierende RO2 2 Umgebung konfiguriert haben.
Zur Installation und Konfiguration wird auf die ROS 2 Dokumentation verwiesen: https://docs.ros.org/en/humble/index.html

Um das package zu installieren, clonen sie den src Ordner in ihr ros2-workspace-Verzeichnis
Falls Sie schon einen existierenden src Ordner haben, können Sie den Inhalt des *RobotikProjekt3/src* Ordners in diesen kopieren.

Wechseln Sie im Terminal in ihr ros2-workspace Verzeichnis und führen Sie dort den folgenden Befehl aus:
```colcon build ```

## Anleitung zum Starten des Systems
### 1. Installation vorbereiten:
- Stellen Sie sicher, dass alle erforderlichen Komponenten und Software vorhanden sind.
### 2. USB-Slots freigeben:
 - Öffnen Sie ein Terminal und führen Sie den Befehl : ```sudo usermod –a –G dialout $USER```aus.
### 3. Anschließen der USB-Verbidungen
 - Schließen Sie die zwei USB-Kabel (ESP, Arduino) der Sortiermaschine an
 - Stellen Sie sicher dass der Arduino und ESP verbunden und verfügbar sind.
   <br> Im Terminal eingeben: ```ls/dev/``` in dieser Liste müssten die Namen ```ttyACM0``` und ```ttyUSB0```erscheinen.
   <br> Falls die Portnamen unterschiedlich sind müssen Sie die Datei: ```install/ro45_ros2_pickrobot_serial/share/ro45_ros2_pickrobot_serial/config/ro45_params.yaml``` anpassen oder den Parameter manuell überschreiben.
### 4. Nullpunkt der Sortiermaschine einstellen
-  Öffnen Sie ein Terminal und wechseln sie in ihr ros2-workspace-Verzeichnis
-  Sourcen Sie ihr ROS2-Overlay:```source install/setup.bash```
-  Starten sie die Serial Knoten mit dem Befehl: ```ros2 launch ro45_ros2_pickrobot_serial launch_nodes.py```
-  Öffnen Sie ein weiteres Terminal, wechseln sie in ihr ros2-workspace-Verzeichnis und sourcen Sie ihr Overlay erneut. Führen sie dann den Befehl ```rqt```aus.
-  Im rqt Fenster erscheinen nun die topics ```/robot_command``` ```/robot_position```
-  Erstellen sie auf der Rechten Seite im Fenster unter **Message Publisher** einen neuen Publisher mit der Topic ```/robot_command``` vom Type ```ro45_portalrobot_interfaces/msg/RobotCmd``` mit einer Frequenz (Freq.) von ```10```Hz und klicken Sie auf das ```+```
-  Klappen Sie nun ihren Publisher auf (Pfeil klicken) und setzen sie die Parameter von ```vel_x``` ```vel_y``` ```vel_z``` auf ```-0.01``` (In der Spalte *expression*)
-  Setzen Sie nun im Freien Kästchen neben ihrer ```/robot_command``` Topic einen Haken. Die Maschine fährt nun auf ihren Nullpunkt.
-  Wenn sich die Maschine nicht mehr bewegt und auf alle Endschalter gefahren ist, stoppen Sie den Serial Knoten ```STRG + C ``` im Terminal und nehmen sie den Haken wieder aus dem Kästchen
-  Schalten Sie die Sortiermaschine ab und trennen Sie die USB-Verbindungen. Warten Sie ein paar Sekunden
-  Schließen Sie die USB-Verbindungen wieder an und schalten Sie die Maschine ein
-  Starten Sie nun den Serial Knoten erneut und
   wählen Sie im rqt Fenster unter **Topic Monitor** ```/robot_position``` aus (Haken im Kästchen) und klappen sie diese auf (Pfeil klicken)
   ```pos_x``` ```pos_y``` ```pos_z``` haben nun den *Value* ```0.0``` Der Nullpunkt ist nun gesetzt.
   <br> Falls die Werte nicht 0.0 sind, versuchen sie den Vorgang erneut und starten Sie auch das rqt Fenster neu!
### 5. AruCo Marker positionieren:
- Platzieren sie die AruCo Marker gemäß dem beigefügten Bild:
- Hier noch Bild einfügen
### 6. Kamera ausrichten
- Schließen sie die Kamera an ihren Computer an
  #### Für die folgenden Schritte können Sie die bereitgestellte Kamera Anwendung ihres Computers verwenden: 
- Überprüfen sie ob die Kamera scharf gestellt ist. falls nicht -> Kamera scharf stellen
- Richten Sie die Kamera so aus dass sich die AruCo Marker in der Mitte vom Bild befinden und vollständig zu sehen sind.

### 7. Starten des Gesamtsystems
- Öffnen Sie ein neues terminal und wechseln sie in ihr ros2-workspace-Verzeichnis
- Sourcen Sie ihr ROS2-Overlay ```source install/setup.bash```
- Starten Sie die Knoten über die launch datei mit folgendem Befehl: ```ros2 launch robot_startup robot_launch.py```
  <br> Die Maschine fährt nun auf die Idle-Position und es kann sortiert werden




 
