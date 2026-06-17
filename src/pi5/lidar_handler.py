import serial
import time

PUERTO = '/dev/ttyUSB0'
BAUDRATE = 460800  

try:
    print(f"[LIDAR] Conectando a {PUERTO} a {BAUDRATE} baudios...")
    
    ser = serial.Serial(PUERTO, BAUDRATE, timeout=2.0, write_timeout=1.0)
    
    print("[LIDAR] Estableciendo estados de pines DTR/RTS...")
    ser.dtr = False
    ser.rts = True
    time.sleep(0.2)
    
    print("[LIDAR] Limpiando canales seriales...")
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(0.1)
    
    print("[LIDAR] Enviando comando de inicio nativo del C1 (Start Scan)...")
    ser.write(b'\xa5\x82\x05\x00\x00\x00\x00\x00\x22')
    ser.flush()
    
    print("[LIDAR] Esperando cabecera de confirmación (7 bytes)...")
    time.sleep(0.5)
    
    if ser.in_waiting >= 7:
        cabecera = ser.read(7)
        print(f"[LIDAR] ¡Cabecera recibida con éxito!: {cabecera.hex()}")
    else:
        print("[LIDAR]  Advertencia: No se detectó la cabecera inicial, forzando lectura de ráfaga...")

    print("\n ESCUCHANDO FLUJO PERPETUO DE DATOS")
    print("Presiona Ctrl+C para detener el robot.\n")
    
    ser.timeout = 0.1
    
    while True:
        if ser.in_waiting > 0:
            datos = ser.read(ser.in_waiting)
            
            print(f"Telemetría C1: {datos.hex()[:70]}...")
        else:
            time.sleep(0.01)
            
except KeyboardInterrupt:
    print("\n\n[LIDAR] Deteniendo escaneo por el usuario...")
    try:
        ser.write(b'\xa5\x25')
        ser.flush()
        ser.close()
        print("[LIDAR]  Motor apagado y puerto cerrado de forma segura.")
    except:
        pass
except Exception as e:
    print(f"\n[LIDAR ERROR]: {e}")
