#!/bin/bash

cd src
g++ -o pi.exe pi.cpp -lwiringPi -lpthread
python3 master.py
cd ..


