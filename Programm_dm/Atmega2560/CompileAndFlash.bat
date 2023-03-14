@echo off

cd C:\Users\morit\Documents\Microcontroller\Robocup\Robocup_2023\Programm_dm\Atmega2560
make

cd C:\Program Files (x86)\Arduino\hardware\tools\avr\bin
avrdude -D -patmega2560 -cwiring -PCOM4 -b115200 -Uflash:w:"$(ProjectDir)Debug\$(TargetName).hex":i -C"C:\Program Files (x86)\Arduino\hardware\tools\avr\etc\avrdude.conf"