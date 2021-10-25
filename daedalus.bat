@echo off
set USER=ec765
START ssh -L 8081:idpcam2.eng.cam.ac.uk:8080 %USER%@gate.eng.cam.ac.uk
python Daedalus\main.py 2>nul