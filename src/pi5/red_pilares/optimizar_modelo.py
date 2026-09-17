import numpy as np
from hailo_sdk_client import ClientRunner

print("< Hailo > Cargando el archivo HAR recortado...")
runner = ClientRunner()
# Carga el modelo limpio que parseamos con las capas nativas de YOLOv8
runner.load_har('pilares_v0.har')

print("< Hailo > Creando datos de calibración en memoria RAM...")
# Generamos el bloque de 32 matrices aleatorias (640x640x3) para cumplir el requisito de INT8
fake_images = np.random.rand(32, 640, 640, 3).astype(np.float32)
input_data = {'pilares_v0/input_layer1': fake_images}

print("< Hailo > Iniciando la optimización y cuantización a INT8...")
# El DFC calculará de forma automática los rangos óptimos para el chip
runner.optimize(input_data)

print("< Hailo > Guardando el archivo HAR optimizado...")
runner.save_har('pilares_v0_quant.har')
print("¡Listo! El archivo 'pilares_v0_quant.har' se ha generado con éxito.")
