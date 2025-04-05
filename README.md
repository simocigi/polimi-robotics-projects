 # Istruzioni per il primo utilizzo
- Scaricare GitHub Desktop e clonare la repo in una nuova cartella
- Copiare e incollare i file di avvio per ROS distribuiti a lezione dentro la cartella
  - Utenti Windows/Mac: creare all'interno una nuova cartella "start" dove incollare i file utilizzati per avviare ROS
  - Utenti Linux: incollare i file .sh per avviare ROS dentro la cartella direttamente, la sottocartella "start" non è necessaria
- Avviare il terminale ed entrare su ROS, cd catkin_ws, catkin_make

# Come runnare la bag
- Avviare ROS da terminale
- Creare una sessione con tmux e splittare
- In una finestra, runnare roscore
- Nell'altra, cd data e poi usare il comando: rosbag play --clock project.bag