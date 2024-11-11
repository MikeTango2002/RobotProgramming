In che ordine leggere questo codice sorgente?

L'eseguibile si chiama "rp_sim", pertanto vado subito a vedere il file 
"rp_sim.cpp".

La prima libreria locale (inclusa cioè tra le "") è "world.h". Vado a vedere tale
file.

Ora, in "world.h" vengono incluse altre due librerie locali: "linalg.h" e "canvas.h".
Iniziamo a controllare "linalg.h".

In "linalg.h" noto con piacere che sono importate solo librerie standard (cioè comprese
tra <>) e posso dunque iniziare a leggere questo file.

"canvas.h" include "linalg.h" e quindi posso leggere questo file.

Posso ora leggere "world.h".

Ritorno in "rp_sim.cpp" e vedo che viene importata anche "grid_map.h".

Questo file include, tra gli header locali non ancora analizzati, "grid.h" e 
"opencv2/opencv.hpp" (questo non va analizzato).

"grid.h" non include header locali e quindi sto apposto.

Ora posso leggere "grid_map.h".

Torniamo in "rp_sim.cpp" e vedo che mancano ancora due header locali:
"differential_drive_robot.h" e "lidar.h".

Andiamo in "differential_drive_robot.h", il quale non include header locali non ancora analizzati.

Andiamo allora in "lidar.h" e anche qui non devo andare oltre.

Posso tornare in "rp_sim.cpp" e finire di leggere il codice.


ORDINE DI LETTURA:
1) "linalg.h"
2) "canvas.h" e "canvas.cpp"
3) "world.h" e "world.cpp"
4) "grid.h"
5) "grid_map.h"
6) "differential_drive_robot.h" e "differential_drive_robot.cpp"
7) "lidar.h" e "lidar.cpp"
8) "rp_sim.cpp" e "sim_in_class.cpp"