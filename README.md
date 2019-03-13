# MEng_Project_Arm
The purpose of this code is to control (and monitor) Dynamixel MX-64AR smart servos. It is designed to work with Arduino Mega (ATmega2560) and MAX485 TTL to RS485 module. The code is based on work by Mark Khordi-Moodi as a part of his dissertation completed in 2018.
# Atmel Studio 7 Setup:
In [Tools] => [External Tools]:
Titile: Arduino MEGA COM8
Command: C:\Program Files (x86)\Arduino\hardware\tools\avr\bin\avrdude.exe
Arguments: -C "C:\Program Files (x86)\Arduino\hardware\tools\avr\etc\avrdude.conf" -v -p atmega2560 -c wiring -P COM8 -b 115200 -D -U flash:w:"$(ProjectDir)Debug\$(TargetName).hex":i
Initial directory: 
# Version 0.2:
Implemented 3 buttons that control the move of the robotic arm. 1st button toggle between two motors and the other 2 buttons adjust the angle down and up of the connected segment (link): [motorSelect][down][up].
