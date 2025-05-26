import RPi.GPIO as GPIO
import time
import speech_recognition as sr
from threading import Lock, Thread, Event
from enum import Enum, auto

# Configuración global
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BOARD)
SENSOR_PIN = 11  # Pin global del sensor

# Enumeración para direcciones
class Direccion(Enum):
    ADELANTE = auto()
    ATRAS = auto()
    IZQUIERDA = auto()
    DERECHA = auto()
    ALTO = auto()

class ControlMotor:
    def __init__(self, en_pin, in1_pin, in2_pin):
        self.en_pin = en_pin
        self.in1_pin = in1_pin
        self.in2_pin = in2_pin
        self.lock = Lock()
        
        GPIO.setup(self.en_pin, GPIO.OUT)
        GPIO.setup(self.in1_pin, GPIO.OUT)
        GPIO.setup(self.in2_pin, GPIO.OUT)
        
        self.pwm = GPIO.PWM(self.en_pin, 1000)
        self.pwm.start(0)
    
    def set_velocidad(self, velocidad):
        with self.lock:
            self.pwm.ChangeDutyCycle(velocidad)
    
    def set_Direccion(self, direccion):
        with self.lock:
            if direccion == Direccion.ADELANTE:
                GPIO.output(self.in1_pin, GPIO.LOW)
                GPIO.output(self.in2_pin, GPIO.HIGH)
            elif direccion == Direccion.ATRAS:
                GPIO.output(self.in1_pin, GPIO.HIGH)
                GPIO.output(self.in2_pin, GPIO.LOW)
            else:  # ALTO
                GPIO.output(self.in1_pin, GPIO.LOW)
                GPIO.output(self.in2_pin, GPIO.LOW)
    
    def ALTO(self):
        with self.lock:
            self.pwm.ChangeDutyCycle(0)
            GPIO.output(self.in1_pin, GPIO.LOW)
            GPIO.output(self.in2_pin, GPIO.LOW)
    
    def cleanup(self):
        self.ALTO()
        self.pwm.stop()

class SensorVelocidad:
    def __init__(self):
        self.pulsos = 0
        self.distancia = 0.0
        self.velocidad = 0.0
        self.pulsos_por_rev = 19
        self.cm_por_rev = 20
        self.ultimo_tiempo = time.time()
        self.lock = Lock()
        self.distancia_objetivo = None
        self.evento_distancia = Event()
        
        GPIO.setup(SENSOR_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.add_event_detect(SENSOR_PIN, GPIO.RISING, 
                            callback=self._callback, bouncetime=10)
    
    def _callback(self, channel):
        with self.lock:
            self.pulsos += 1
            self.distancia = (self.pulsos / self.pulsos_por_rev) * self.cm_por_rev
            
            if self.distancia_objetivo and self.distancia >= self.distancia_objetivo:
                self.evento_distancia.set()
                self.distancia_objetivo = None
            
            ahora = time.time()
            periodo = ahora - self.ultimo_tiempo
            if periodo > 0:
                self.velocidad = (self.cm_por_rev / self.pulsos_por_rev) / periodo
            self.ultimo_tiempo = ahora
    
    def establecer_distancia_objetivo(self, distancia_cm):
        with self.lock:
            self.distancia_objetivo = distancia_cm
            self.evento_distancia.clear()
    
    def esperar_distancia(self, timeout=None):
        return self.evento_distancia.wait(timeout)
    
    def obtener_datos(self):
        with self.lock:
            return {
                'pulsos': self.pulsos,
                'distancia': self.distancia,
                'velocidad_cm_s': self.velocidad,
                'velocidad_km_h': self.velocidad * 0.036,
                'objetivo_alcanzado': self.evento_distancia.is_set()
            }
    
    def reset(self):
        with self.lock:
            self.pulsos = 0
            self.distancia = 0.0
            self.velocidad = 0.0
            self.ultimo_tiempo = time.time()
            self.distancia_objetivo = None
            self.evento_distancia.clear()
    
    def cleanup(self):
        GPIO.remove_event_detect(SENSOR_PIN)

class CarritoRobot:
    def __init__(self):
        # Configuración de pines
        self.ENA = 33
        self.IN1 = 35
        self.IN2 = 37
        self.ENB = 32
        self.IN3 = 36
        self.IN4 = 40
        
        # Configuración de velocidad
        self.velocidad_min = 30
        self.velocidad_max = 100
        self.velocidad = self.velocidad_min
        self.MOVIMIENTO_CONTINUO = False
        
        # Inicialización de motores
        self.motor_a = ControlMotor(self.ENA, self.IN1, self.IN2)
        self.motor_b = ControlMotor(self.ENB, self.IN3, self.IN4)
        
        # Sensor de velocidad
        self.sensor = SensorVelocidad()
        
        # Reconocimiento de voz
        self.recognizer = sr.Recognizer()
        print("Carrito robot inicializado correctamente")
    
    def set_velocidad(self, velocidad):
        if self.velocidad_min <= velocidad <= self.velocidad_max:
            self.velocidad = velocidad
            self.motor_a.set_velocidad(velocidad)
            self.motor_b.set_velocidad(velocidad)
    
    def aceleracion_progresiva(self, target_velocidad=None):
        target_velocidad = target_velocidad or self.velocidad
        self.motor_a.set_velocidad(target_velocidad)
        self.motor_b.set_velocidad(target_velocidad)
        
        # def _execute():
        #     for velocidad in range(20, target_velocidad + 1, 10):
        #         self.motor_a.set_velocidad(velocidad)
        #         self.motor_b.set_velocidad(velocidad)
        #         time.sleep(0.3)
        
        # Thread(target=_execute, daemon=True).start()
    
    def desaceleracion_progresiva(self):
        def _execute():
            current_velocidad = self.velocidad
            for velocidad in range(current_velocidad, 20, -10):
                self.motor_a.set_velocidad(velocidad)
                self.motor_b.set_velocidad(velocidad)
                time.sleep(0.3)
            self.ALTO()
        
        Thread(target=_execute, daemon=True).start()
    
    def MOVIMIENTO(self, direccion, distancia_cm=None):
        if direccion == Direccion.ADELANTE:
            self.motor_a.set_Direccion(Direccion.ADELANTE)
            self.motor_b.set_Direccion(Direccion.ADELANTE)
            self.aceleracion_progresiva()
            
            if distancia_cm is not None:
                self.sensor.establecer_distancia_objetivo(distancia_cm)
                Thread(target=self._monitorear_distancia, args=(distancia_cm,), daemon=True).start()
        
        elif direccion == Direccion.ATRAS:
            self.motor_a.set_Direccion(Direccion.ATRAS)
            self.motor_b.set_Direccion(Direccion.ATRAS)
            self.aceleracion_progresiva()
        
        elif direccion == Direccion.IZQUIERDA:
            if self.MOVIMIENTO_CONTINUO:
                self.desaceleracion_progresiva()
                self.motor_a.set_Direccion(Direccion.ADELANTE)
                self.motor_b.set_Direccion(Direccion.ADELANTE)
                self.motor_a.set_velocidad(self.velocidad_min)
                self.motor_b.set_velocidad(0)
                time.sleep(1.5)
                self.aceleracion_progresiva()
            else:
                self.motor_a.set_Direccion(Direccion.ADELANTE)
                self.motor_b.set_Direccion(Direccion.ADELANTE)
                self.motor_a.set_velocidad(self.velocidad_min)
                self.motor_b.set_velocidad(0)
                time.sleep(1.5)
                self.ALTO()
        
        elif direccion == Direccion.DERECHA:
            if self.MOVIMIENTO_CONTINUO:
                self.desaceleracion_progresiva()
                self.motor_a.set_Direccion(Direccion.ADELANTE)
                self.motor_b.set_Direccion(Direccion.ADELANTE)
                self.motor_a.set_velocidad(0)
                self.motor_b.set_velocidad(self.velocidad_min)
                time.sleep(1.5)
                self.aceleracion_progresiva()
            else:
                self.motor_a.set_Direccion(Direccion.ADELANTE)
                self.motor_b.set_Direccion(Direccion.ADELANTE)
                self.motor_a.set_velocidad(0)
                self.motor_b.set_velocidad(self.velocidad_min)
                time.sleep(1.5)
                self.ALTO()
    
    def _monitorear_distancia(self, distancia_cm):
        if self.sensor.esperar_distancia(timeout=20):
            print(f"\n¡OBJETIVO ALCANZADO! {distancia_cm} cm completados")
            self.ALTO()
            self.sensor.reset()
            self.sensor.distancia=None
        else:
            print("\nAdvertencia: No se alcanzó la distancia en el tiempo esperado")
            self.sensor.distancia_objetivo = None
    
    def GIRO(self, direccion):
        if direccion == Direccion.DERECHA:
            if self.MOVIMIENTO_CONTINUO:
                self.desaceleracion_progresiva()
                self.motor_a.set_Direccion(Direccion.ATRAS)
                self.motor_b.set_Direccion(Direccion.ADELANTE)
                self.motor_a.set_velocidad(self.velocidad_min)
                self.motor_b.set_velocidad(self.velocidad_min)
                time.sleep(1.5)
                self.aceleracion_progresiva()
            else:
                self.motor_a.set_Direccion(Direccion.ATRAS)
                self.motor_b.set_Direccion(Direccion.ADELANTE)
                self.motor_a.set_velocidad(self.velocidad_min)
                self.motor_b.set_velocidad(self.velocidad_min)
                time.sleep(1.5)
                self.ALTO()
        else:  # DERECHA
            if self.MOVIMIENTO_CONTINUO:
                self.desaceleracion_progresiva()
                self.motor_a.set_Direccion(Direccion.ADELANTE)
                self.motor_b.set_Direccion(Direccion.ATRAS)
                self.motor_a.set_velocidad(self.velocidad_min)
                self.motor_b.set_velocidad(self.velocidad_min)
                time.sleep(1.5)
                self.ALTO()
                self.aceleracion_progresiva()
            else:
                self.motor_a.set_Direccion(Direccion.ADELANTE)
                self.motor_b.set_Direccion(Direccion.ATRAS)
                self.motor_a.set_velocidad(self.velocidad_min)
                self.motor_b.set_velocidad(self.velocidad_min)
                time.sleep(1.5)
                self.ALTO()
    
    def ALTO(self):
        self.motor_a.ALTO()
        self.motor_b.ALTO()
        self.MOVIMIENTO_CONTINUO = False
    
    def procesar_comando_voz(self, texto):
        texto = texto.lower()
        
        if (not self.MOVIMIENTO_CONTINUO and 
            self.sensor.distancia_objetivo is not None and
            not self.sensor.evento_distancia.is_set()):
            print("ADVERTENCIA: Comando ignorado. En progreso movimiento a distancia fija")
            return
        
        if "avanza" in texto:
            if len(texto.split()) == 1: 
                print("COMANDO: Avanzando en modo continuo")
                self.MOVIMIENTO_CONTINUO = True
                self.MOVIMIENTO(Direccion.ADELANTE)
            else:
                distancia = None
                if "un" in texto:
                    distancia = 100
                elif any(word in texto for word in ["2", "3", "4", "5", "6", "7", "8", "9"]):
                    try:
                        distancia = int(texto.split()[1])
                        if distancia < 10:
                            distancia *= 100
                    except:
                        pass
                
                if distancia:
                    print(f"COMANDO: Avanzando {distancia} cm")
                    self.MOVIMIENTO(Direccion.ADELANTE, distancia)
        
        elif "atrás" in texto or "retrocede" in texto:
            print("COMANDO: Movimiento hacia atrás")
            self.MOVIMIENTO(Direccion.ATRAS)        

        elif "giro" in texto or "gira" in texto:
            if "derecha" in texto:
                print("COMANDO: Giro Derecha")
                self.GIRO(Direccion.DERECHA)
            elif "izquierda" in texto:
                print("COMANDO: Giro Derecha")
                self.GIRO(Direccion.IZQUIERDA)
        
        elif "izquierda" in texto:
            print("COMANDO: Giro a la izquierda")
            self.MOVIMIENTO(Direccion.IZQUIERDA)
        
        elif "derecha" in texto:
            print("COMANDO: Giro a la derecha")
            self.MOVIMIENTO(Direccion.DERECHA)
        
        elif "velocidad" in texto:
            try:
                vel = int(texto.split()[1])
                if self.velocidad_min <= vel <= self.velocidad_max:
                    print(f"COMANDO: Ajustando velocidad a {vel}%")
                    self.set_velocidad(vel)
                else:
                    print("COMANDO: Velocidad fuera de rango (30-100%)")
            except:
                print("COMANDO: Ajuste de velocidad no válido")
        
        elif any(palabra in texto for palabra in ["alto", "detente", "para"]):
            print("COMANDO: Deteniendo movimiento")
            self.ALTO()
            self.sensor.reset()
        
        elif any(palabra in texto for palabra in ["termina", "salir", "finalizar"]):
            print("COMANDO: Terminando programa")
            return False
        
        else:
            print("COMANDO no reconocido")
        
        # Mostrar datos del sensor
        datos = self.sensor.obtener_datos()
        print(f"\n[DATOS SENSOR] Distancia: {datos['distancia']:.1f} cm | Velocidad: {datos['velocidad_km_h']:.1f} km/h\n")
        return True
    
    def ejecutar(self):
        try:
            print("=== CONTROL POR VOZ DEL CARRITO ROBOT ===")
            print("Comandos disponibles: avanza [distancia], atrás, izquierda, derecha, velocidad [X], alto, termina\n")
            
            with sr.Microphone(device_index=11) as source:
                self.recognizer.adjust_for_ambient_noise(source)
                print("Sistema listo para recibir comandos...")
                
                while True:
                    try:
                        print("\nDi un comando...")
                        audio = self.recognizer.listen(source, timeout=3)
                        texto = self.recognizer.recognize_google(audio, language='es-ES')
                        print(f"Has dicho: {texto}")
                        
                        if not self.procesar_comando_voz(texto):
                            break
                    
                    except sr.UnknownValueError:
                        print("No se entendió el comando")
                    except sr.RequestError:
                        print("Error en el servicio de reconocimiento")
                    except sr.WaitTimeoutError:
                        continue
        
        except KeyboardInterrupt:
            print("\nInterrupción por teclado")
        finally:
            self.cleanup()
            GPIO.cleanup()
            print("Sistema terminado correctamente")
    
    def cleanup(self):
        self.ALTO()
        self.motor_a.cleanup()
        self.motor_b.cleanup()
        self.sensor.cleanup()

# Programa principal
if __name__ == "__main__":
    carrito = CarritoRobot()
    carrito.ejecutar()
