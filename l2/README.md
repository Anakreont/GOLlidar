# Unitree L2: Point-LIO i rzut z chmury 3D

1. Instalacja na Dellu, jednorazowo. Instalator stawia ROS 1 Noetic z robostack w `~/ros1_l2` i kompiluje `point_lio_unilidar` od Unitree w `~/catkin_l2`. Nie potrzebuje sudo ani Dockera.

       bash ~/GOLlidar/l2/instaluj_l2.sh

2. Nagranie L2 (.bag, np. przykładowe z https://www.unitree.com/download/L2):

       ~/bin/l2_mapa "~/L2 Indoor Point Cloud Data.bag" ~/l2_indoor

   Program odtwarza nagranie w czasie rzeczywistym (Point-LIO), zapisuje `chmura.pcd`, a potem uruchamia `l2_rzut.py`.

3. `l2_rzut.py`: podłogę i sufit znajduje z histogramu wysokości. Daje:
   - **SCIANY**: pas 15–45 cm pod sufitem,
   - **CIECIE_1M**: pas 1,0–1,2 m nad podłogą,
   - wysokość w świetle.

   Pliki wynikowe: `rzut.png`, `rzut.dxf` (w metrach) i `wysokosci.txt`.

Stan na 26.11.2026: `l2_rzut.py` sprawdzony na sztucznej chmurze. Instalator i Point-LIO nie były uruchamiane, pierwsza próba odbywa się na Dellu.
