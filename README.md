# Proyecto Final Integrador: Robot de 2 Grados de Libertad para Captura de Multimedia

## Ingeniería en Mecatrónica  
## Facultad de Ingeniería - Universidad Nacional de Cuyo  

---

## Imágenes

![Robot](Micro Overleaf/robotito.jpg)
![Esquema de conexiones](Micro Overleaf/esquema_conexiones.png)
![Joystick](Micro Overleaf/joystick.png)

---

## Descripción

Este proyecto consiste en el diseño, control y programación de un robot de **2 grados de libertad (GDL)**, diseñado específicamente para facilitar la captura de fotografías y filmaciones de objetos desde diversas perspectivas.

El sistema permite un control preciso tanto de la posición como de la velocidad de los movimientos.

---

## Características Principales

- **Arquitectura de 2 GDL**  
  Posee un movimiento traslacional y uno rotacional (ángulo *yaw*), este último con un soporte integrado para cámara o smartphone.

- **Modos de Operación**
  - **Modo Manual:** Controlado por el usuario a través de un joystick inalámbrico de fabricación propia.
  - **Modo Automático:** Ejecuta trayectorias predefinidas tras realizar un proceso de *homing* para reconocer su posición absoluta.

- **Movimientos Suaves**  
  Implementación de perfiles de velocidad trapezoidales con rampas de aceleración y desaceleración para evitar movimientos bruscos.

- **Comunicación Inalámbrica**  
  Enlace por radiofrecuencia (2.4 GHz) entre el control y el robot.

---

## Hardware Utilizado

El sistema se divide en dos unidades principales que se comunican de forma inalámbrica:

### 1. Robot (Receptor)

- **Microcontrolador:** Arduino Uno  
- **Actuadores:** 2 motores paso a paso controlados por drivers DRV8825  
- **Sensores:**  
  - Fines de carrera mecánicos (eje traslacional)  
  - Optoacoplador óptico (eje rotacional)  
- **Comunicación:** Módulo NRF24L01  

### 2. Joystick (Transmisor)

- **Microcontrolador:** Arduino Pro Mini  
- **Entradas:**  
  - 2 analógicas  
  - 4 botones  
  - 2 switches  
  - 2 potenciómetros  
- **Comunicación:** Módulo NRF24L01  

---

## Detalles de Implementación (Software)

El software ha sido desarrollado priorizando la eficiencia y el uso de recursos del microcontrolador:

- **Máquina de Estados**  
  El funcionamiento global se rige por una máquina de estados finitos que gestiona la inicialización, el *homing* y los modos de operación.

- **Librería NRF24L01 Propia**  
  Se desarrolló una librería personalizada para la comunicación inalámbrica basada en el protocolo SPI y el protocolo Enhanced ShockBurst.

- **Control por Interrupciones**  
  Se utilizan:
  - Interrupciones externas y de cambio de pin para sensores  
  - Interrupciones para recepción de datos  
  - Timers para generación de pulsos de los motores  

- **Manejo de Bits**  
  Uso de macros para optimizar la manipulación de registros y configuración de pines.

---

## Ensayos y Resultados

Se determinó experimentalmente la relación de pasos por unidad de medida para ambos ejes:

- **Eje Traslacional:** 322 pasos/cm  
- **Eje Rotacional:** 7200 pasos por vuelta completa (360°)  
