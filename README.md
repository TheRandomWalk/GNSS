# SoftGPS para Ettus B200

Receptor GPS L1 C/A por software en C++ para linea de comando. Esta version
inicial, basada en la estructura de `..\sdrtv`, usa UHD para capturar IQ desde
un Ettus B200 y ejecuta adquisicion GPS C/A para PRN 1-32.

## Estado

Implementado:

- Configuracion directa del B200 con UHD.
- Sintonia GPS L1 en `1575.42 MHz`.
- Generacion de codigos GPS C/A PRN 1-32.
- Busqueda por Doppler y fase de codigo.
- Tabla de satelites detectados y exportacion CSV.

Pendiente para una solucion PVT completa:

- Tracking DLL/PLL/FLL continuo.
- Decodificacion de mensaje NAV y efemerides.
- Calculo de pseudorangos y solucion WGS84.

## Build

Requiere Visual Studio C++ tools, Ninja, CMake y UHD. Si ya usas el entorno de
`sdrtv`, el mismo UHD/Radioconda deberia servir.

```powershell
.\scripts\build.ps1
```

Si UHD esta en otra ruta:

```powershell
cmake -S . -B build -G Ninja -DSOFTGPS_UHD_ROOT="C:/Program Files/UHD"
cmake --build build
```

## Uso

Con una antena GPS activa conectada al B200:

```powershell
.\build\softgps.exe --antenna RX2 --gain 45 --seconds 0.08
```

Opciones utiles:

```text
--args STR             argumentos UHD, ej. "type=b200"
--antenna RX2|TX/RX    entrada RF del B200
--gain DB              ganancia RX
--rate SAMPLES         tasa de muestreo, default 2046000
--bandwidth HZ         ancho de banda RX, default 2400000
--seconds N            segundos a capturar
--prn A-B              rango PRN, default 1-32
--doppler A:B:C        min:max:paso en Hz
--threshold N          umbral peak/promedio
--csv FILE             escribe resultados CSV
```

Ejemplo con salida CSV:

```powershell
.\build\softgps.exe --args "type=b200" --antenna RX2 --gain 50 --csv gps-acq.csv
```

## Notas RF

GPS L1 llega muy debil. Usa una antena GPS activa con alimentacion adecuada
externa o bias-tee compatible, vista clara al cielo y evita saturar el B200.
Si no aparecen locks, prueba ganancias entre 35 y 60 dB, revisa `uhd_find_devices`
y confirma que no haya overflows.
