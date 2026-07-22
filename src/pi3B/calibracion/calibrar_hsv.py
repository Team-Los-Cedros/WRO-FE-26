import cv2
import numpy as np
import socket
import time

# Nombre de las ventanas nativas en tu laptop
VENTANA_CONTROLES = "Controles-HSV"
VENTANA_FEED = "Calibrar-HSV"

cv2.namedWindow(VENTANA_CONTROLES, cv2.WINDOW_NORMAL)
cv2.namedWindow(VENTANA_FEED, cv2.WINDOW_AUTOSIZE)

def nada(x): pass

# Barras deslizantes interactivas en tu laptop
cv2.createTrackbar("MODO (0:Verde|1:Rojo1|2:Rojo2|3:Final)", VENTANA_CONTROLES, 0, 3, nada)
cv2.createTrackbar("H Min", VENTANA_CONTROLES, 0, 179, nada)
cv2.createTrackbar("S Min", VENTANA_CONTROLES, 0, 255, nada)
cv2.createTrackbar("V Min", VENTANA_CONTROLES, 0, 255, nada)
cv2.createTrackbar("H Max", VENTANA_CONTROLES, 179, 179, nada)
cv2.createTrackbar("S Max", VENTANA_CONTROLES, 255, 255, nada)
cv2.createTrackbar("V Max", VENTANA_CONTROLES, 255, 255, nada)

# Rangos HSV iniciales
calib = {
    0: {'h_min': 35,  's_min': 50,  'v_min': 50,  'h_max': 85,  's_max': 255, 'v_max': 255},
    1: {'h_min': 0,   's_min': 70,  'v_min': 50,  'h_max': 10,  's_max': 255, 'v_max': 255},
    2: {'h_min': 170, 's_min': 70,  'v_min': 50,  'h_max': 180, 's_max': 255, 'v_max': 255}
}

modo_anterior = 0

def detectar_forma(frame, mascara, color, etiqueta):
    contornos, _ = cv2.findContours(mascara, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in contornos:
        if cv2.contourArea(c) > 500:
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.05 * peri, True)  # tolerancia de borde
            if 4 <= len(approx) <= 6:
                x, y, w, h = cv2.boundingRect(approx)
                cv2.rectangle(frame, (x, y), (x+w, y+h), color, 2)
                cv2.putText(frame, etiqueta, (x, y-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

def main():
    global modo_anterior
    
    # Configurar el socket para escuchar la transmisión de la Pi
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind(('0.0.0.0', 5000))
    server_socket.listen(1)
    
    print(" Esperando Respuesta del puerto 5000")
    conexion, direccion = server_socket.accept()
    print(f"Conectado desde {direccion[0]}!")
    
    buffer_bytes = b''
    
    # Sincronizar todos los sliders al arrancar con el modo Verde (0)
    cv2.setTrackbarPos("H Min", VENTANA_CONTROLES, calib[0]['h_min'])
    cv2.setTrackbarPos("S Min", VENTANA_CONTROLES, calib[0]['s_min'])
    cv2.setTrackbarPos("V Min", VENTANA_CONTROLES, calib[0]['v_min'])
    cv2.setTrackbarPos("H Max", VENTANA_CONTROLES, calib[0]['h_max'])
    cv2.setTrackbarPos("S Max", VENTANA_CONTROLES, calib[0]['s_max'])
    cv2.setTrackbarPos("V Max", VENTANA_CONTROLES, calib[0]['v_max'])

    # Kernel para la limpieza morfológica (matriz de 5x5)
    kernel = np.ones((5, 5), np.uint8)

    try:
        while True:
            datos = conexion.recv(4096)
            if not datos: break
            buffer_bytes += datos
            
            inicio_jpeg = buffer_bytes.find(b'\xff\xd8')
            fin_jpeg = buffer_bytes.find(b'\xff\xd9')
            
            if inicio_jpeg != -1 and fin_jpeg != -1:
                jpg = buffer_bytes[inicio_jpeg:fin_jpeg+2]
                buffer_bytes = buffer_bytes[fin_jpeg+2:]
                
                frame = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is None: continue

                # Interfaz de control dinámico
                modo_actual = cv2.getTrackbarPos("MODO (0:Verde|1:Rojo1|2:Rojo2|3:Final)", VENTANA_CONTROLES)
                
                if modo_actual != modo_anterior and modo_actual in calib:
                    cv2.setTrackbarPos("H Min", VENTANA_CONTROLES, calib[modo_actual]['h_min'])
                    cv2.setTrackbarPos("S Min", VENTANA_CONTROLES, calib[modo_actual]['s_min'])
                    cv2.setTrackbarPos("V Min", VENTANA_CONTROLES, calib[modo_actual]['v_min'])
                    cv2.setTrackbarPos("H Max", VENTANA_CONTROLES, calib[modo_actual]['h_max'])
                    cv2.setTrackbarPos("S Max", VENTANA_CONTROLES, calib[modo_actual]['s_max'])
                    cv2.setTrackbarPos("V Max", VENTANA_CONTROLES, calib[modo_actual]['v_max'])
                    modo_anterior = modo_actual

                if modo_actual in calib:
                    calib[modo_actual]['h_min'] = cv2.getTrackbarPos("H Min", VENTANA_CONTROLES)
                    calib[modo_actual]['s_min'] = cv2.getTrackbarPos("S Min", VENTANA_CONTROLES)
                    calib[modo_actual]['v_min'] = cv2.getTrackbarPos("V Min", VENTANA_CONTROLES)
                    calib[modo_actual]['h_max'] = cv2.getTrackbarPos("H Max", VENTANA_CONTROLES)
                    calib[modo_actual]['s_max'] = cv2.getTrackbarPos("S Max", VENTANA_CONTROLES)
                    calib[modo_actual]['v_max'] = cv2.getTrackbarPos("V Max", VENTANA_CONTROLES)

                # Procesar algoritmos de visión en la laptop
                hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
                
                lower_v = np.array([calib[0]['h_min'], calib[0]['s_min'], calib[0]['v_min']])
                upper_v = np.array([calib[0]['h_max'], calib[0]['s_max'], calib[0]['v_max']])
                lower_r1 = np.array([calib[1]['h_min'], calib[1]['s_min'], calib[1]['v_min']])
                upper_r1 = np.array([calib[1]['h_max'], calib[1]['s_max'], calib[1]['v_max']])
                lower_r2 = np.array([calib[2]['h_min'], calib[2]['s_min'], calib[2]['v_min']])
                upper_r2 = np.array([calib[2]['h_max'], calib[2]['s_max'], calib[2]['v_max']])

                mask_green = cv2.inRange(hsv, lower_v, upper_v)
                mask_red1 = cv2.inRange(hsv, lower_r1, upper_r1)
                mask_red2 = cv2.inRange(hsv, lower_r2, upper_r2)
                mask_red = cv2.bitwise_or(mask_red1, mask_red2)

                # Limpieza morfologica: OPEN quita ruido, CLOSE une bloques partidos
                mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_OPEN, kernel)
                mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)
                mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
                mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_CLOSE, kernel)

                # Detección de formas sobre las máscaras ya limpias
                detectar_forma(frame, mask_green, (0, 255, 0), "VERDE")
                detectar_forma(frame, mask_red, (0, 0, 255), "ROJO")

                # Renderizado de ventanas según el modo seleccionado
                if modo_actual == 0:    
                    render = cv2.cvtColor(mask_green, cv2.COLOR_GRAY2BGR)
                elif modo_actual == 1:  
                    render = cv2.cvtColor(mask_red1, cv2.COLOR_GRAY2BGR)
                elif modo_actual == 2:  
                    render = cv2.cvtColor(mask_red2, cv2.COLOR_GRAY2BGR)
                else:                   
                    render = frame  # En modo 3 muestra la cámara real con los recuadros dibujados

                cv2.imshow(VENTANA_FEED, render)
                if cv2.waitKey(1) & 0xFF == ord('q'): break
    finally:
        conexion.close()
        server_socket.close()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()