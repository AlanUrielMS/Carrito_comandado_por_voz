# Robot-Carrito  
El robot-carrito es un **sistema robótico autónomo y modular**, diseñado como plataforma educativa y experimental para implementar tecnologías de **control por voz, procesamiento de lenguaje natural y robótica móvil**. Está construido sobre una base mecánica con ruedas, integrando componentes electrónicos de bajo costo y la tarjeta **Jetson Nano** como unidad central de procesamiento. Su diseño prioriza:  
- **Accesibilidad**: Pensado para makers, estudiantes y entusiastas.  
- **Modularidad**: Permite añadir/remover sensores y actuadores.  
- **Interacción intuitiva**: Control mediante comandos de voz en español.  
![]()
---

## **¿Qué hace el Robot-Carrito?**  

### **1. Reconocimiento y Ejecución de Comandos de Voz**  
- **Procesamiento en tiempo real**:  
  - Captura audio mediante micrófono (I2S o USB).  
  - Interpreta órdenes como *"avanza"*, *"gira a la izquierda"*, *"detente"* usando:  
    - **SpeechRecognition** (offline, con PocketSphinx).  
    - **APIs en la nube** (Google/AWS para mayor precisión).  
  - Mapea los comandos a acciones mediante lógica programada en Python.  

### **2. Navegación y Control de Movimiento**  
- **Sistema de tracción**:  
  - Motores DC controlados por **PWM** (modulación por ancho de pulso) para ajustar velocidad.  
  - Driver **L298N** para manejo de dirección (avance/retroceso/giros).  
- **Parámetros optimizados**:  
  - Frecuencia PWM: **1 kHz** para evitar vibraciones.  
  - Duty cycle típico: **60-70%** para movimiento estable.  

### **3. Sensores Integrados (Opcionales)**  
- **Ultrasónicos (HC-SR04)**: Detección de obstáculos (evasión automática).  
- **IMU (MPU6050)**: Medición de inclinación para terrenos irregulares.  
- **Cámara (Pi Camera)**: Futura implementación de visión por computadora.  

### **4. Adaptabilidad a Entornos Reales**  
- **Filtrado de ruido**:  
  - Hardware: Micrófono direccional con supresión de eco.  
  - Software: Eliminación de frecuencias no vocales (>4 kHz) con `librosa`.  
- **Modos de operación**:  
  - **Offline**: Ideal para zonas sin internet (precisión media).  
  - **Online**: Máxima precisión en interiores con conexión estable.  

---

## **Aplicaciones Prácticas**  
1. **Educación en robótica**: Enseña conceptos de IA, procesamiento de voz y control embebido.  
2. **Prototipado rápido**: Base para proyectos de domótica o logística autónoma.  
3. **Asistencia**: Potencial uso para personas con movilidad reducida (control por voz).  

---
