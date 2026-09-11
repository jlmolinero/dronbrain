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

## MMA845x inclinómetro

Programa C++ para leer un acelerómetro MMA8451Q/MMA8452Q/MMA8453Q y mostrar
inclinación continuamente con valores crudos, aceleración en `g`, roll/pitch y
un gráfico ASCII de actitud.

Aunque pueda estar montado junto a sensores SPI, la familia MMA845x se comunica
por I2C. El programa autodetecta el sensor en `/dev/i2c-1` y `/dev/i2c-2`,
direcciones `0x1c`/`0x1d`.

### Compilar

```bash
make
```

### Ejecutar

```bash
./mma845x_inclinometer
./mma845x_inclinometer --samples 20 --interval-ms 200
./mma845x_inclinometer --bus /dev/i2c-1 --address 0x1d
```

Si se ejecuta en una terminal normal limpia la pantalla entre muestras. Para
guardar logs sin secuencias de limpieza:

```bash
./mma845x_inclinometer --no-clear --samples 10
```
