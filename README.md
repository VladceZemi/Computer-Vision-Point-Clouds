# Computer-Vision-Point-Clouds

Repozitář obsahuje dva projekty, které byly součástí školního předmětu.

---

Složka **gestures** obsahuje rozpoznání několika různých gest.

<p align="center">
  <img src="https://user-images.githubusercontent.com/93316166/189916700-f884f650-a905-411c-b84a-2542e21d3b8e.gif" />
</p>

---

Složka **point_cloud** obsahuje zpracování mračna bodů. Konkrétně se jedná o vesnici, a cílem je kontrukce kostry střech domů.

<p align="center">
<img src="https://user-images.githubusercontent.com/93316166/189917123-684c8d20-210f-43bd-9c27-d88458acf666.png" />
</p>

---

Návod na spuštění projektu na **OS Linux**

---

Pro sestavení a spuštění projektu je nutné mít naistalovanou knihovnu **OpenCV** a nástroj **CMake**.

1. V kořenovém adresáři vytvořit složku build a přidat do ní složku videos
2. V adresáři v terminálu spustit příkaz cmake s přepínačem -S, a cestou k CMakeLists.txt. Tzv. **cmake -S /home/repo/lvr/gestures/CMakeLists.txt**
3. Ve stejném adresáři v terminálu spustit příkaz **make**
4. Ve stejném adresáři v terminálu spustit příkaz **./lvr**
