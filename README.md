# Upravljanjem mobilnim robotom na diferencijalni pogon - završni rad

## SAŽETAK

Završni rad opisuje modifikaciju mobilnog robota Pioneer 2DX koji koristi diferencijalni sustav pogona. Robot je nadograđen s novim upravljačkim i pogonskim komponentama uz koje je izrađen upravljački algoritam niske razine s mogućnošću dvosmjerne komunikacije s nadređenim upravljačkim računalima. U sklopu zadatka projektiran je regulator brzine vrtnje elektromotora čije su performanse ispitane odzivima na različite pobudne funkcije. Nadalje, primjenom jednadžbi direktne kinematike podređeni upravljački sustav vrši izračun referenci brzine vrtnje kotača u odnosu na željene translacijske i rotacijske brzine, dok se jednadžbama direktne kinematike tada preko enkodera estimiraju realne brzine robota. Serijska komunikacija s nadređenim računalom izvedena je pomoću robotskog operativnog sustava (eng. Robot Operating System – ROS) primjenom paketa „rosserial“. Dodatno, rad obuhvaća konstruiranje i izradu adaptera novih pogonskih motora tehnologijom 3D ispisa, uz koje je također prikazana shema spajanja upravljačkih komponenti te je izrađena prototipna pločica.

Sva dokumentacija koja se tiče upravljačkog programa s modoficiranim knjižnicama, nalazi se u završnom radu (Završni rad_Luka Grden.pdf). Uz to na repozitoriju su dodane
step. datoteke modeliranih dijelovima zajedno s upravljačkim kodom mobilnog robota.

![image](https://user-images.githubusercontent.com/38221332/191283752-9b0b1979-6e6c-41b1-b66d-9a9960cacd47.png) 

# Control of a differential drive mobile robot - final paper

## SUMMARY

The final paper describes the modification of the Pioneer 2DX mobile robot that uses a differential drive system. The robot was upgraded with new control and drive components, along with which a low-level control algorithm was created with the possibility of two-way communication with  a high-level controller. As part of the task, an electric motor rotation speed regulator was designed, whose performance was tested by responses to different input signals. Furthermore, by applying the equations of direct kinematics, the low-level control system calculates the reference speed of the wheels in relation to the desired translational and rotational speeds, while the equations of direct kinematics are then used to estimate the real speeds of the robot through the motor encoder. Serial communication with the high-level controller is performed with the robot operating system (Robot Operating System - ROS) using the "rosserial" package. In addition, the work includes constructing and manufacturing of adapters for new drive motors using 3D printing technology. Along with that, with  given connection scheme, the prototype circuit board is crated.

All documentation regarding control algorithm with modified libaries is included in the final paper (Završni rad_Luka Grden.pdf). In addition, step. files of modeled parts together with the control code of the mobile robot is added to the repository.

![image](https://user-images.githubusercontent.com/38221332/191283878-94891de8-01d4-44f4-9aaf-74372840a42a.png)

