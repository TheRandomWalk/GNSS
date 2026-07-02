# GNSS para Ettus B200

Receptor GPS L1 C/A por software en C++20 para linea de comando. El proyecto permite capturar IQ desde un Ettus B200 con UHD o trabajar offline con archivos IQ. La app ejecuta adquisicion GPS C/A, exporta resultados, puede hacer tracking basico por PRN y resuelve PVT por minimos cuadrados cuando se entrega un CSV de observaciones con posiciones ECEF de satelites y pseudorangos.

## Estado

Implementado:

- Configuracion directa del B200 con UHD cuando UHD esta disponible.
- Build offline sin UHD para pruebas, archivos IQ y solver PVT.
- Sintonia GPS L1 en `1575.42 MHz`.
- Lectura de archivos IQ `fc32`, `sc16` y `cs8`.
- Generacion de codigos GPS C/A PRN 1-32.
- Busqueda por Doppler y fase de codigo.
- Tabla de satelites detectados y exportacion CSV.
- Tracking basico early/prompt/late por epochs de 1 ms.
- Solver PVT ECEF/WGS84 por minimos cuadrados con GDOP, PDOP, HDOP, VDOP, TDOP y residuos.
- Tests unitarios para C/A, adquisicion, tracking y PVT.

Limitacion importante:

- La decodificacion NAV en vivo desde el bitstream GPS todavia no esta implementada. Para calcular posicion, usa `--pvt` con observaciones externas o generadas por otro pipeline: `prn,x_m,y_m,z_m,pseudorange_m[,sigma_m]`.

## Build

Requiere CMake, Ninja y un compilador C++20. UHD es opcional: si no se encuentra, el binario se compila igual en modo IQ/PVT offline.

```powershell
.\scripts\build.ps1
```

Si UHD esta en otra ruta:

```powershell
cmake -S . -B build -G Ninja -DGNSS_UHD_ROOT="C:/Program Files/UHD"
cmake --build build
ctest --test-dir build --output-on-failure
```

Build sin UHD:

```bash
cmake -S . -B build -G Ninja -DGNSS_ENABLE_UHD=OFF
cmake --build build
ctest --test-dir build --output-on-failure
```

## Uso con B200

Con una antena GPS activa conectada al B200:

```powershell
.\build\gnss.exe --antenna RX2 --gain 45 --seconds 0.08
```

Ejemplo con salida CSV:

```powershell
.\build\gnss.exe --args "type=b200" --antenna RX2 --gain 50 --csv gnss-acq.csv
```

## Uso con IQ offline

Archivo `fc32` intercalado I,Q como `float32` little-endian:

```bash
./build/gnss --iq capture.fc32 --iq-format fc32 --rate 2046000 --prn 1-32 --csv acq.csv
```

Archivo `sc16` intercalado I,Q como `int16`:

```bash
./build/gnss --iq capture.sc16 --iq-format sc16 --rate 2046000 --doppler -7000:7000:250
```

## Tracking

Primero corre adquisicion y luego sigue el mejor lock o un PRN especifico:

```bash
./build/gnss --iq capture.fc32 --track-ms 200 --track-csv tracking.csv
./build/gnss --iq capture.fc32 --track-prn 7 --track-ms 200 --track-csv prn7-tracking.csv
```

El CSV de tracking incluye `prompt_i`, `prompt_q`, magnitudes early/prompt/late, discriminador DLL y una estimacion C/N0.

## PVT

Formato de entrada:

```csv
prn,x_m,y_m,z_m,pseudorange_m,sigma_m
1,15600000,0,20180000,21482000,3
2,18760000,13400000,0,23211000,3
3,17610000,-13480000,13400000,22144000,3
4,19170000,610000,-18390000,24590000,3
```

Ejecutar:

```bash
./build/gnss --pvt observations.csv --pvt-out solution.csv
```

Salida principal:

- Latitud, longitud y altura WGS84.
- Coordenadas ECEF.
- Sesgo de reloj en metros y segundos.
- RMS de residuos.
- GDOP, PDOP, HDOP, VDOP y TDOP.

## Opciones principales

```text
--args STR             argumentos UHD, ej. "type=b200"
--antenna RX2|TX/RX    entrada RF del B200
--gain DB              ganancia RX
--frequency HZ         frecuencia RX, default 1575420000
--rate SAMPLES         tasa de muestreo, default 2046000
--bandwidth HZ         ancho de banda RX, default 2400000
--seconds N            segundos a capturar
--prn A-B              rango PRN, default 1-32
--doppler A:B:C        min:max:paso en Hz
--threshold N          umbral peak/promedio
--csv FILE             resultados de adquisicion
--iq FILE              archivo IQ offline
--iq-format fc32|sc16|cs8
--track-ms N           cantidad de epochs de 1 ms
--track-prn N          PRN a seguir
--track-csv FILE       salida tracking
--pvt FILE             CSV de observaciones
--pvt-out FILE         salida PVT
```

## Notas RF

GPS L1 llega muy debil. Usa una antena GPS activa con alimentacion adecuada externa o bias-tee compatible, vista clara al cielo y evita saturar el B200. Si no aparecen locks, prueba ganancias entre 35 y 60 dB, revisa `uhd_find_devices`, confirma que no haya overflows y usa capturas de al menos 80 ms para inspeccion inicial.
