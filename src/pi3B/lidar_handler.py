import sys
import time
from rplidar import RPLidar, RPLidarException

# Configuración del puerto según tu entorno
PUERTO = '/dev/ttyUSB0'
BAUDRATE = 460800  

def procesar_datos():
    lidar = RPLidar(PUERTO, baudrate=BAUDRATE, timeout=2000)
   
    print("[LIDAR] Conectando e inicializando el motor...")
    time.sleep(1)
   
    print("\n=== ESCUCHANDO DISTANCIAS EN TIEMPO REAL ===")
    print("Rangos definidos: Frente (±15°), Atrás (±15°), Izquierda (±15°), Derecha (±15°)")
    print("Presiona Ctrl+C para detener el script.\n")
   
    try:
        # CORRECCIÓN: El método correcto es iter_measures()
        for measurement in lidar.iter_measures():
            # iter_measures devuelve: (bool_nueva_vuelta, calidad, ángulo_grados, distancia_mm)
            _, _, angle, distance = measurement
           
            if distance > 0:
                # 1. FRENTE (Zona alrededor de los 0° / 360°)
                if angle <= 15 or angle >= 345:
                    print(f"[FRENTE]    Ángulo: {angle:6.1f}° | Distancia: {distance:6.1f} mm")
               
                # 2. DERECHA (Zona alrededor de los 90°)
                elif 75 <= angle <= 105:
                    print(f"[DERECHA]   Ángulo: {angle:6.1f}° | Distancia: {distance:6.1f} mm")
               
                # 3. ATRÁS (Zona alrededor de los 180°)
                elif 165 <= angle <= 195:
                    print(f"[ATRÁS]     Ángulo: {angle:6.1f}° | Distancia: {distance:6.1f} mm")
               
                # 4. IZQUIERDA (Zona alrededor de los 270°)
                elif 255 <= angle <= 285:
                    print(f"[IZQUIERDA] Ángulo: {angle:6.1f}° | Distancia: {distance:6.1f} mm")
                   
    except KeyboardInterrupt:
        print("\n[LIDAR] Deteniendo escaneo por el usuario...")
    except RPLidarException as re:
        print(f"\n[LIDAR ERROR DE PROTOCOLO]: {re}")
    except Exception as e:
        print(f"\n[LIDAR ERROR GENERAL]: {e}")
    finally:
        print("[LIDAR] Apagando de forma segura...")
        try:
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
            print("[LIDAR] Desconectado correctamente.")
        except:
            print("[LIDAR] Error al cerrar los puertos, forzando salida.")

if __name__ == '__main__':
    procesar_datos()
