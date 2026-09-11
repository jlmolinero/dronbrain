# dronbrain
The necesary program to fly a dron with raspberry pi

## SPL06-001 altímetro SPI

Programa C++ para leer el sensor de presión/temperatura SPL06-001 conectado al
bus SPI de una Raspberry Pi y usarlo como altímetro.

### Compilar

```bash
make
```

### Ejecutar

```bash
sudo ./spl06_altimeter --samples 10
```

Por defecto usa `/dev/spidev0.0`, SPI mode 0, 1 MHz, referencia estándar de
nivel del mar `1013.25 hPa` y calcula una altura relativa con las primeras 10
muestras. Opciones útiles:

```bash
./spl06_altimeter --help
./spl06_altimeter --device /dev/spidev0.1 --samples 20
./spl06_altimeter --sea-level-hpa 1018.4 --baseline-samples 20
```

La salida incluye ID del chip, registros de configuración, coeficientes de
calibración, presión sin compensar, temperatura sin compensar, temperatura,
presión en varias unidades y altura estimada en metros/pies.
