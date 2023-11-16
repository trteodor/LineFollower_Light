# Light Follower Light

Demonstration videos:

Jsumo wheels vs solarbotics:
https://www.youtube.com/watch?v=5hosZAXFKjE

X_Challange 2023, Rzeszow:
https://www.youtube.com/watch?v=OcwFzTCTTZQ

New JSumo Tires test:
https://youtu.be/Sj1vxMwn7Ys (achived average speed 1.8m/s!)

Real word prototypes:    ( Jsumo and Solarbotics wheels )

![FristPrototypePhoto](https://github.com/trteodor/LineFollower_Light/blob/master/60_Pictures/BothRobotsIzoView.png)

Simulated assembly izometric view:

![Izometric3D_View](https://github.com/trteodor/LineFollower_Light/blob/master/60_Pictures/IzometricAssemblySimu.jpg)

  * You will need to spend around 200$ if you want recreate this design (PCB order included)
  * most important parts list: 10_PCB_Hardware_Altium\Readme.md


In Embedded source code is implemented many algorithms, PID Controller is used to follow for the black line on white theme. 

Nevertheless many things still can be done much better - Let's be honest - the prepared Control Code in my opinion is somewhere between a draft and a prototype

# You can also check my previous Line Follower Project:
* https://github.com/trteodor/Line_Follower_STM32H7

# In this project:

PCB and 3D Assemblyis prepared, real world prototype prepared

TODO:
* Add support for MPU-6050
* Add support for VL53l0
* Development of QT service application..
* Generally much things..

The repository contains all the files used to create this project (PCB, Mechanic models, Embedded source Code, QT serv. App code)

QT service app overview screen shot:

![QtApp](https://github.com/trteodor/LineFollower_Light/blob/master/60_Pictures/QT_servApplScreenShotEx.jpg)


Installer wizard:  (created using Inno Compiler)
https://github.com/trteodor/LineFollower_Light/blob/master/40_ServiceApplication_QT/LF_ServiceApp_1.35_Installer.exe

After installation associated "*.lfp" extension should be opened by default by LF_ServiceApp"

QT_AppIco

![QtAppIco](https://github.com/trteodor/LineFollower_Light/blob/master/60_Pictures/LF_SimuAsseblyBackSideRightIco.png)

# References
* https://github.com/trteodor/Line_Follower_STM32H7
* https://forbot.pl/blog/algorytm-linefollowera-c-poczatkujacych-id2722
* https://kcir.pwr.edu.pl/~mucha/Pracki/Jedrzej_Stolarz_praca_inzynierska.pdf
* https://kcir.pwr.edu.pl/~mucha/Pracki/Witek_Lipieta_praca_magisterska.pdf
* https://kcir.pwr.edu.pl/~mucha/Pracki/Rafal_Cyminski_praca_magisterska.pdf
* https://forbot.pl/forum/topic/13552-cukiereczek/

**_If you have question please open issue to this repository, or write comment under video on youtube._**
