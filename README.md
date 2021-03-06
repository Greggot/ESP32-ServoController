# Servo controller

![Project](/img/ProjPhotoBeta.jpg)

## Components

- 1 x ESP32-D0WD chip on WiFi Kit 32 board
- 2 x MG996R servomotors
- 1 x Nextion display NX8048P070-011C
- nRF connection application for mobile platform

## Application in a nutshell

This scheme illustrates how does PWM signal turns rotor of MG996R

![PWM signal](https://arduinomaster.ru/wp-content/uploads/2018/02/1.png)

In practice that means that `duty = 3% <=> 0 degree, duty = 12% <=> 180 degree`. We need to setup 2 independent PWM signals that will drive servo depending on `Bluetooth Low Energy` or `UART` signal.

## Communication

### **Bluetooth Low Energy(BLE)**

This wireless communication protocol has been added *in prior*. No application is ready to send data that corresponds with user input, **BUT** you can control every servo's angle sending 1 byte of data throug **nRF Connect** mobile application. One of the characteristics write event will happen and change servo's duty.

### **UART**

UART is used because of Nextion display that is fully controlled via UART. User can even send script commands through COM port to control screens, variables' values and widgets. Display can output data depending on sensor input as well.

Display project for NX8048P070-011C is made via **Nextion Studio**. Two sliders send their values when of them is touched or moved. From ESP's side a thread listens for UART messages generated by display. PWM duty changes depending on slider's value.
