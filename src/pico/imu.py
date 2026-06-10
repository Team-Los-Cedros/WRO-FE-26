# src/pico/imu.py
from machine import I2C, Pin
import time

class ControlIMU:
    def __init__(self):
        # I2C0 en pines GP16/GP17 segun cableado fisico actual en chasis
        self.i2c = I2C(0, sda=Pin(16, Pin.PULL_UP), scl=Pin(17, Pin.PULL_UP), freq=400000)
        self.addr = 0x68
        
        # Power management 1: desactivar modo sleep del mpu
        self.i2c.writeto_mem(self.addr, 0x6B, b'\x00')
        
        # Gyro config: FS_SEL=0 (Scale range +/- 250 deg/s -> FS=131.0 LSB/deg/s)
        self.i2c.writeto_mem(self.addr, 0x1B, b'\x00')
        
        self.yaw = 0.0
        self.gz_offset = 0.0
        self.t_last = time.ticks_us()
        
        self.init_calibracion()

    def init_calibracion(self):
        print("[IMU] Calibrando giroscopio... Mantener chasis quieto.")
        raw_sum = 0
        samples = 200
        for _ in range(samples):
            raw_sum += self._read_raw_gz()
            time.sleep_ms(5)
        self.gz_offset = raw_sum / samples
        print(f"[IMU] Calibracion OK. Offset: {self.gz_offset:.2f}")
        self.t_last = time.ticks_us()

    def _read_raw_gz(self):
        # Registro 0x47: GYRO_ZOUT_H y GYRO_ZOUT_L
        reg_data = self.i2c.readfrom_mem(self.addr, 0x47, 2)
        val = (reg_data[0] << 8) | reg_data[1]
        return val - 65536 if val >= 32768 else val

    def actualizar_yaw(self):
        t_now = time.ticks_us()
        dt = time.ticks_diff(t_now, self.t_last) / 1000000.0
        self.t_last = t_now
        
        # Remocion de offset de ruido estatico
        gz_filtered = self._read_raw_gz() - self.gz_offset
        gyro_z_dps = gz_filtered / 131.0
        
        # Filtro de zona muerta para evitar drift por vibraciones del chasis
        if abs(gyro_z_dps) > 0.45:
            self.yaw += gyro_z_dps * dt
            
        return self.yaw

    def reset_yaw(self):
        self.yaw = 0.0

# DEBUG LOCAL
if __name__ == '__main__':
    imu = ControlIMU()
    print("[DEBUG] Inicializando loop de telemetria basica...")
    try:
        while True:
            y = imu.actualizar_yaw()
            print(f"YAW: {y:+.2f} deg")
            time.sleep_ms(40)
    except KeyboardInterrupt:
        print("\n[DEBUG] Test detenido por usuario.")
