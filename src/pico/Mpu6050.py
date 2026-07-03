import machine

class MPU6050:
    def __init__(self, i2c, addr=0x68):
        self.i2c = i2c
        self.addr = addr
        # Encender Mpu6050 (Por defecto esta apagado)
        self.i2c.writeto_mem(self.addr, 0x6B, b'\x00')

    def _read_word(self, reg):
        buf = self.i2c.readfrom_mem(self.addr, reg, 2)
        val = (buf[0] << 8) | buf[1]
        if val >= 0x8000:
            return -((65535 - val) + 1)
        return val

    def get_values(self):
        # Leer Acelerómetro (escalado por defecto a +/-2g)
        ax = self._read_word(0x3B) / 16384.0
        ay = self._read_word(0x3D) / 16384.0
        az = self._read_word(0x3F) / 16384.0
        
        # Leer Temperatura
        temp = (self._read_word(0x41) / 340.0) + 36.53
        
        # Leer Giroscopio (escalado por defecto a +/-250 deg/s)
        gx = self._read_word(0x43) / 131.0
        gy = self._read_word(0x45) / 131.0
        gz = self._read_word(0x47) / 131.0
        
        return {
            "acel": {"x": ax, "y": ay, "z": az},
            "temp": temp,
            "giro": {"x": gx, "y": gy, "z": gz}
        }
