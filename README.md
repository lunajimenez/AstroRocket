# AstroRocket - HMI para Sistema de Control de Cohete a Escala

Este repositorio contiene el desarrollo del sistema HMI (Interfaz Humano-Máquina) para un cohete a escala controlado por un microcontrolador STM32 NUCLEO F411RE. El objetivo es ofrecer una interfaz de monitoreo y control amigable e intuitiva, tanto a nivel embebido como a través de un SMP (Sistema Mecatrónico Programable), con visualización mediante herramientas como Grafana.

---

## 🔊 Descripción del Proyecto

El sistema consiste en un cohete a escala que despega mediante combustible sólido tipo "candy". Cuenta con un sistema de control basado en un microcontrolador STM32 (NUCLEO-F411RE), 5 servomotores para estabilización y control de trayectoria, y sensores que permiten monitorear el comportamiento del cohete durante el vuelo. La estructura ha sido diseñada en OpenRocket, las piezas impresas en PETG y recubiertas en fibra de vidrio.

Posee un mecanismo de recuperación basado en la detección de velocidad cero (altura máxima), en donde se activa un servo para desplegar la ojiva y liberar un paracaídas. Se planea la implementación de un sistema PID para el control de las aletas en función de la trayectoria.

---

## ⚙️ Tecnologías Utilizadas

* **Microcontrolador:** STM32F411RE (programado en STM32CubeIDE)
* **Backend:** Python
* **Visualización:** Grafana
* **Base de datos:** InfluxDB
* **Diseño estructural:** OpenRocket
* **Diseño electrónico:** KiCAD
* **Material de fabricación:** PETG y fibra de vidrio

---

## 📊 Requerimientos del HMI

### 🏛️ Para Embebidos

* Arquitectura **orientada a objetos**.
* Indicadores individuales por sensor (temperatura, presión, acelerómetro, giroscopio, etc).
* Comunicación con el microcontrolador mediante **puerto serial**.
* Gráficas del comportamiento de sensores clave (aceleración, velocidad, ángulo de vuelo, etc).
* Registro de datos en logs para análisis posterior (**opcional pero recomendado**).
* Interfaz **intuitiva, amigable visualmente y de fácil uso**.

### 📂 Para el SMP (Sistema Mecatrónico Programable)

* Integración de al menos un **controlador** ejecutando el firmware.
* Capacidad de **almacenamiento y transmisión de datos** operativos (vía LAN/WAN).
* Centro de mando HMI con **controles físicos** (interruptores, sliders, joystick) y luces indicadoras.
* **Pantalla de estado** (táctil o no) para visualización de variables y alertas.
* HMI principal para interacción remota/local del usuario.
* Puerto serial adicional para programación y actualización del firmware.
* Módulo protegido con **contraseña para configuración avanzada** del sistema.

---

## 📚 Estructura del Repositorio

```
.
├── /firmware/                # Código del STM32 (STM32CubeIDE)
├── /backend/                 # Backend Python para comunicación y recolección de datos
├── /grafana_dashboards/      # Dashboards y configuraciones para Grafana
├── /pcb/                     # Archivos KiCAD para el diseño de la PCB
├── /rocket_design/           # Archivos OpenRocket para el diseño del cohete
├── /docs/                    # Documentación técnica y manuales
└── README.md
```

---

## 🚀 Contribuciones Futuras

* Implementación del control PID para las aletas en tiempo real
* Agregado de autenticación por roles en la HMI principal
* Optimizar visualizaciones para pantallas táctiles
* Integración de módulo de predicción de trayectoria

---

## ✉️ Contacto

Si tienes preguntas, sugerencias o deseas colaborar, por favor contáctanos:

* Luna Katalina Quintero Jiménez [Github aquí](https://github.com/lunajimenez)

* Juan Sebastián Pérez Muñoz [Github aquí](https://github.com/JuanPe1204)

* Leonardo Esteban González Castillo [Github aquí](https://github.com/leogoca00)

---

> Este proyecto ha sido desarrollado como una investigación del semillero de Astronomía y ciencia de datos, la asignatura de Sistemas Embebidos y la de Sistemas Mecatrónicos Programables. Todas estos espacios de aprendizaje brindados e impartidos por la Universidad Tecnológica de Bolívar. Su desarrollo se centra en el área de sistemas embebidos, electrónica y control aplicado al lanzamiento de cohetes experimentales.
