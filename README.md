# Simulador de control PID · Motor DC

Plataforma para estudiar el control PID de velocidad de un motor de corriente continua, en simulación y sobre hardware real, desde una misma aplicación de escritorio.

El repositorio contiene dos programas que trabajan juntos:

- **Aplicación en Python (PyQt5)** — simula el lazo cerrado resolviendo las ecuaciones eléctrica y mecánica del motor con Runge-Kutta de cuarto orden, y además se conecta por el puerto serie a un Arduino para gobernar el motor físico.
- **Firmware en Arduino Uno (C++/PlatformIO)** — mide la velocidad del eje con un sensor óptico de ranura y regula el PWM del motor con un PID discreto.

Proyecto de la asignatura **Control Inteligente**, Ingeniería Mecatrónica, Universidad Tecnológica de Bolívar.

---

## Tabla de contenido

- [Características](#características)
- [Estructura del repositorio](#estructura-del-repositorio)
- [Hardware](#hardware)
- [Puesta en marcha](#puesta-en-marcha)
- [Uso de la aplicación](#uso-de-la-aplicación)
- [Protocolo serie](#protocolo-serie)
- [Cómo funciona](#cómo-funciona)
- [Parámetros por defecto](#parámetros-por-defecto)
- [Resultados obtenidos](#resultados-obtenidos)
- [Limitaciones conocidas](#limitaciones-conocidas)
- [Trabajo futuro](#trabajo-futuro)
- [Informe](#informe)
- [Autor](#autor)

---

## Características

**Simulador**

- Modelo de motor DC controlado por armadura, con corriente de armadura y velocidad angular como variables de estado.
- Integración por Runge-Kutta de cuarto orden con subpasos automáticos y retenedor de orden cero, para que el paso de integración se adapte a la constante de tiempo eléctrica.
- PID en paralelo con anti-windup por integración condicional, derivada sobre la medición y filtro pasa-bajas en el término derivativo. Las tres opciones se activan y desactivan desde la interfaz.
- Referencia configurable: escalón, onda cuadrada o senoidal.
- Diagrama de bloques del lazo y diagrama del modelo físico, ambos con el valor vivo de cada señal.
- Modelo 3D del motor girando, dibujado por software con QPainter (no requiere OpenGL).
- Historial del ensayo completo con memoria acotada: el transitorio inicial nunca se pierde, aunque la simulación se alargue.

**Control del motor físico**

- Conexión por puerto serie con selección de puerto y baudios desde la interfaz.
- Lectura en vivo de RPM, PWM aplicado, pulsos, consigna, error, modo y sentido de giro.
- Gráfica de RPM contra tiempo con la consigna dibujada en escalones y diezmado que conserva picos.
- Ajuste de la consigna y de las tres ganancias sin recompilar ni recargar el firmware.
- Monitor del puerto serie con envío manual de órdenes y ficha del dispositivo USB.
- Parada de emergencia; al cerrar la ventana se envía `s` antes de soltar el puerto.

**Firmware**

- Medición de velocidad de flanco a flanco, sin error de cuantización.
- Doble filtrado del ruido del PWM: rechazo de picos dentro de la interrupción y mediana de las cinco últimas medidas.
- PID con prealimentación que compensa la zona muerta del conjunto motor + puente H.
- Error normalizado al fondo de escala, de modo que las ganancias quedan adimensionales.
- Lazo no bloqueante: sin `delay()` en el bucle principal.

---

## Estructura del repositorio

```
.
├── Simulador_motor/
│   ├── simulador.py          # aplicación completa (interfaz, modelo, PID, enlace serie)
│   ├── requirements.txt      # PyQt5, pyqtgraph, numpy
│   └── ejecutar.bat          # crea el entorno virtual si no existe y lanza la app
│
├── Motor_encoder/
│   └── motor_encoder/
│       ├── src/main.cpp      # firmware del Arduino Uno
│       ├── platformio.ini    # entorno uno / atmelavr / arduino
│       └── .vscode/tasks.json
│
├── docs/
│   ├── informe.pdf           # informe del proyecto en formato IEEE
│   └── capturas/             # evidencias de las pruebas
│
└── README.md
```

---

## Hardware

| Componente | Detalle |
|---|---|
| Motorreductor DC amarillo tipo TT | Relación 1:48, 3 a 6 V |
| Arduino UNO R3 | ATmega328P a 16 MHz |
| Puente H L298N | Etapa de potencia |
| Sensor óptico de ranura FC-03 | Comparador LM393, salida digital `D0` |
| Disco encoder | 20 ranuras |
| Convertidor DC-DC LM2596 | Reductor ajustable |
| Baterías 18650 | Dos en serie, unos 6,5 V |

### Conexiones

| Señal | Pin del Arduino | Función |
|---|---|---|
| L298N `ENA` | `D3` | PWM de velocidad (~490 Hz, Timer2) |
| L298N `IN1` | `D7` | Sentido de giro |
| L298N `IN2` | `D8` | Sentido de giro |
| FC-03 `D0` | `D2` | Pulsos del encoder (interrupción INT0) |
| FC-03 `VCC` / `GND` | `5V` / `GND` | Alimentación del sensor |
| Motor | L298N `OUT1` / `OUT2` | Salida del canal A |
| Baterías | L298N entrada de 12 V y `GND` | Alimentación de potencia |

> **Importante**
> - Retira el jumper de `ENA`. Si lo dejas puesto, el motor gira siempre a máxima velocidad y el PWM del Arduino no tiene efecto.
> - Une todas las tierras (Arduino, L298N y batería) en un solo punto. Sin referencia común, el puente no interpreta las señales lógicas.
> - Lleva el `GND` del sensor directamente a un pin `GND` del Arduino, no a la bornera del L298N, para que la corriente del motor no circule por la tierra del sensor.
> - `D2` no es intercambiable: en el Uno solo los pines 2 y 3 admiten interrupciones externas, y el 3 ya lo ocupa `ENA`. El firmware incluye un `static_assert` que impide compilar si se cambia.

### Ajuste del sensor

El disco debe atravesar la herradura por el centro, sin rozar las paredes. Con el motor girando despacio (un 15 % de PWM), gira el potenciómetro del módulo hasta que el LED de `D0` parpadee claramente con cada ranura. Si el LED queda fijo, encendido o apagado, el umbral está mal ajustado y no se cuenta ningún pulso.

---

## Puesta en marcha

### Aplicación

Requiere Python 3.10 o superior.

```bash
cd Simulador_motor
python -m venv .venv
.venv\Scripts\activate        # Windows
# source .venv/bin/activate   # Linux / macOS
pip install -r requirements.txt
python simulador.py
```

En Windows basta con ejecutar `ejecutar.bat`, que crea el entorno virtual e instala las dependencias la primera vez.

Versiones verificadas: Python 3.14.3, PyQt5 5.15.11 sobre Qt 5.15.2, pyqtgraph 0.14.0 y numpy 2.5.2.

La comunicación serie usa `PyQt5.QtSerialPort`, incluido en la distribución oficial de PyQt5, así que **no hace falta `pyserial`**. Si ese módulo faltara, la vista del motor físico lo indica y el resto del simulador sigue funcionando.

### Firmware

Con [PlatformIO](https://platformio.org/) instalado:

```bash
cd Motor_encoder/motor_encoder
pio run                       # compilar
pio run --target upload       # cargar
pio device monitor            # monitor serie a 9600 baudios
```

También sirve el IDE de Arduino: copia `src/main.cpp` a un sketch y selecciona placa *Arduino Uno*. En el monitor debes activar **«Nueva línea»** o **«Ambos NL & CR»**, porque el firmware solo procesa una orden cuando recibe el fin de línea.

> Si la carga falla con `ser_open(): can't set com-state`, PlatformIO eligió un puerto virtual equivocado. Identifica el puerto correcto con `pio device list` y fíjalo en `platformio.ini` con `upload_port` y `monitor_port`.

---

## Uso de la aplicación

La ventana tiene tres vistas seleccionables desde la cabecera.

**Control PID** — diagrama de bloques del lazo cerrado con el valor vivo de cada señal, y gráficas del error `e(t)`, de la salida del controlador `u(t)` con sus líneas de saturación, y de la salida del proceso `y(t)` junto a la referencia. En la columna izquierda se ajustan la planta, las condiciones iniciales, la referencia y los parámetros de simulación; a la derecha, las ganancias y las opciones del controlador.

**Dinámica del Motor** — malla eléctrica de armadura, equilibrio mecánico del rotor, ecuaciones del modelo, gráficas de corriente y de velocidad angular, y balance de pares.

**Control de Motor Físico** — elige el puerto COM del Arduino y pulsa *Conectar*. Al conectar aparecen la vista 3D, la gráfica de RPM contra tiempo, la medición del encoder, los campos de consigna y los deslizadores de las ganancias.

> Los campos que definen el modelo se bloquean mientras la simulación está en marcha. Las ganancias, la referencia y el par de carga se ajustan en caliente.

> Al abrir el puerto se activa la línea DTR, lo que reinicia la placa. Por eso la aplicación espera 2,5 s antes de enviar las tres ganancias. El botón *Reenviar las tres ganancias* sirve tras un reinicio manual.

> Los deslizadores arrancan en `Kp 3` y `Ki 40`, que son los valores por defecto del firmware con el fondo de escala antiguo de 4000 rpm. Con `rpmMax = 600` esos valores resultan muy agresivos: **muévelos a algo del orden de `Kp 1`, `Ki 3` antes de enviar el primer setpoint.**

---

## Protocolo serie

9600 baudios, 8N1. Toda orden termina con fin de línea. El intérprete ignora los espacios iniciales, no distingue mayúsculas de minúsculas y acepta espacio, `=` o `:` entre la orden y el valor, de modo que `rpm 800`, `rpm800` y `rpm=800` son equivalentes.

| Orden | Rango | Efecto |
|---|---|---|
| `rpm <v>` | −rpmMax a +rpmMax | Fija el setpoint y **activa** el PID |
| `pwm <v>` | −100 a 100 | Fija el PWM y **desactiva** el PID |
| `<v>` | −100 a 100 | Atajo de `pwm <v>` |
| `kp <v>` | 1 a 10 | Constante proporcional |
| `ki <v>` | 0 a 200 | Constante integral (reinicia el integrador) |
| `kd <v>` | 0 a 2 | Constante derivativa |
| `max <v>` | 10 a 4000 | RPM del motor con PWM al 100 % (reinicia el PID) |
| `zm <v>` | 0 a 90 | Zona muerta: PWM mínimo de la prealimentación |
| `ff` | — | Activa o desactiva la prealimentación |
| `r <v>` | 1 a 1000 | Ranuras por vuelta del disco |
| `m` | — | Activa o desactiva el reporte periódico |
| `s` | — | Detiene el motor y desactiva el PID |
| `?` | — | Muestra la ayuda |

El signo indica el sentido de giro, tanto en `rpm` como en `pwm`.

### Reporte periódico

Se imprime cada 250 ms, en dos formatos según el modo:

```
[PID] SP: 300  RPM: 298.7  PWM: 58.1 %  err: 1.3  pulsos: 25
[MAN] RPM: 245.3  PWM: 50.0 %  pulsos: 20
```

El setpoint y el PWM aparecen en valor absoluto, `err` va en RPM y `pulsos` cuenta los pulsos desde el reporte anterior. Cada orden de consigna produce además una línea de confirmación que incluye el sentido (`horario`, `antihorario` o `detenido`).

> **No cambies el formato de estas líneas.** La aplicación las interpreta con expresiones regulares y depende de ellas.

---

## Cómo funciona

### Medición de velocidad

Cuatro filtros sucesivos protegen la medida:

1. **Rechazo de picos (en la ISR).** Un pico inducido por la conmutación del PWM dura pocos microsegundos, mientras que una ranura real mantiene la línea en bajo cientos de microsegundos. Al detectar un flanco de bajada, la interrupción espera 8 µs y vuelve a leer el pin; si ya volvió a alto, descarta el flanco.
2. **Rechazo de rebotes.** Se ignora cualquier flanco que llegue antes de 250 µs del anterior, lo que filtra la oscilación del LM393 en el borde de cada ranura y admite hasta 12 000 rpm con un disco de 20 ranuras.
3. **Medición de flanco a flanco.** En vez de contar pulsos en una ventana fija, se mide el intervalo exacto entre el último flanco de la ventana actual y el de la anterior:

   ```
   RPM = (60e6 · pulsos) / (Δt_µs · ranuras)
   ```

4. **Mediana de cinco.** La velocidad que llega al PID es la mediana de las cinco últimas medidas crudas, lo que rechaza hasta dos valores atípicos a cambio de unos 75 ms de retardo.

Si pasan 300 ms sin pulsos, el motor se da por detenido, la velocidad pasa a cero y el historial se vacía.

### Ley de control

Con `|SP| < 1` rpm la salida es 0 % y el integrador se limpia. En otro caso:

```
ff   = sat[0,100]( pwmMin + |SP| · (100 − pwmMin) / rpmMax )
e%   = (|SP| − RPM) · 100 / rpmMax
P    = Kp · e%
I    = I + Ki · e% · Δt            (solo si la salida no está saturada)
D    = −Kd · Δ(RPM · 100 / rpmMax) / Δt
u    = sat[0,100]( ff + P + I + D )
```

Tres decisiones de diseño valen la pena de resaltar:

- **Error normalizado.** Dividir entre el fondo de escala deja las ganancias adimensionales: `Kp = 3` significa «3 % de PWM por cada 1 % de error de fondo de escala». Sin esto, las constantes tendrían que ser del orden de 0,01.
- **Prealimentación.** El término `ff` aporta de antemano el PWM que un modelo lineal con zona muerta estima necesario para el setpoint, de modo que el PID solo corrige la diferencia en vez de construir todo el esfuerzo desde el integrador.
- **Anti-windup y derivada sobre la medición.** El integrador solo acumula si la salida no está saturada, o si el error empuja de vuelta al rango útil; la derivada se toma sobre la medición para eliminar el pico de salida al cambiar la consigna.

El sensor tiene un solo canal, así que **no detecta el sentido de giro**: el PID regula la magnitud de la velocidad y el signo del setpoint decide el sentido directamente sobre `IN1` e `IN2`.

---

## Parámetros por defecto

### Firmware

| Parámetro | Valor | Descripción |
|---|---|---|
| `kp`, `ki`, `kd` | 3 · 40 · 0 | PI puro al arrancar |
| `rpmMax` | 600 rpm | Fondo de escala (velocidad con PWM al 100 %) |
| `pwmMin` | 15 % | Zona muerta usada por la prealimentación |
| `ffActivo` | activada | Estado de la prealimentación |
| `ranurasPorVuelta` | 20 | Ranuras del disco encoder |
| `PERIODO_PID_MS` | 25 ms | Medición y control (40 Hz) |
| `PERIODO_REPORTE_MS` | 250 ms | Reporte por serie (4 Hz) |
| `MIN_PULSO_US` | 250 µs | Separación mínima entre flancos válidos |
| `TIEMPO_SIN_PULSOS_US` | 300 ms | Tiempo sin pulsos para dar el motor por detenido |
| `BAUDIOS` | 9600 | Velocidad del puerto serie |

Uso de recursos de la versión final: **17,6 % de la RAM** (361 B) y **33,7 % de la flash** (10 866 B) del ATmega328P.

### Motor equivalente del simulador

Parámetros ajustados para que la simulación se parezca al motor real:

| Parámetro | Valor | Origen |
|---|---|---|
| `Va,max` | 6 V | Supuesto (voltaje efectivo con PWM al 100 %) |
| `Ra` | 5 Ω | Típico de un motor pequeño |
| `La` | 2 mH | Típico de un motor pequeño |
| `Kt = Ke` | 0,08 | Ajustados para ~600 rpm a 6 V |
| `J` | 1,3 × 10⁻⁴ kg·m² | Ajustado a τ ≈ 0,1 s (supuesto) |
| `B` | 1 × 10⁻⁵ N·m·s | Casi despreciable |
| `TL` | 0,0144 N·m | Representa la zona muerta del 15 % |

Con estos valores el motor simulado llega a 604 rpm con 6 V, no gira con el 15 % de PWM y gira a 28,4 rpm con el 19 %, igual que el real.

### Equivalencia de ganancias

El firmware trabaja en «% de PWM por % de fondo de escala» y el simulador en «voltios por rad/s». La conversión es:

```
K_sim = K_arduino · Va,max / ω_max = 0.0949 · K_arduino
```

Vale igual para `Kp`, `Ki` y `Kd`, porque ambos controladores acumulan la integral como `Ki · e · Δt`.

---

## Resultados obtenidos

| Aspecto | Resultado |
|---|---|
| Rango útil del controlador | ~25 a 150 rpm |
| Velocidad máxima medida (PWM 100 %) | ~600 rpm |
| Zona muerta | Arranca con 19 %, se mantiene hasta 15 % |
| Error en estado estacionario | Prácticamente nulo dentro del rango útil |
| Variación de la lectura a velocidad máxima | ±40 % con PWM constante |
| Arranque de la aplicación | 1,9 s |

Dentro del rango útil, la respuesta al escalón muestra la forma clásica de un lazo PI: sobreimpulso, un segundo pico menor y asentamiento sobre la consigna sin error permanente.

El límite superior lo impone **la medición, no el actuador**. Con PWM constante la lectura varía cerca de un ±40 %, algo imposible para la inercia real de un motorreductor. De hecho, las ~600 rpm medidas triplican las 200 rpm que la hoja de datos del TT 1:48 especifica en vacío a 6 V, lo que confirma que a alta velocidad el encoder cuenta pulsos espurios.

---

## Limitaciones conocidas

- **Resolución del encoder.** 20 ranuras en el eje de salida dan muy poca resolución a baja velocidad y la lectura se degrada al subir la velocidad. Es la mayor limitación del sistema.
- **Sentido de giro no medido.** Con un solo canal óptico, el firmware supone que el sentido real coincide con el signo del setpoint.
- **Límite inferior.** Por debajo de unas 10-15 rpm los pulsos llegan cada 300 ms, que es justo el tiempo tras el cual el firmware da el motor por detenido.
- **Caída del L298N.** Los transistores bipolares del puente pierden de 2 a 3 V, lo que reduce la velocidad máxima alcanzable con la batería de 6,5 V.
- **Retardo de la mediana.** Un escalón de velocidad aparece tres muestras después, unos 75 ms; a baja velocidad el retardo crece.
- **El simulador idealiza la planta.** No incluye el ruido ni el retardo de la medición, no modela la fricción estática del arranque, puede aplicar voltaje negativo (el Arduino solo entrega de 0 a 100 %) y no incorpora la prealimentación del firmware.
- **Zumbido audible.** El PWM a 490 Hz produce un silbido en el motor. Se puede subir a ~31 kHz cambiando el preescalador del Timer2, a costa de que `tone()` deje de funcionar en el pin 11.

---

## Trabajo futuro

- Mejorar la resolución del transductor: disco en el eje del motor, antes de la reductora, o encoder Hall de cuadratura.
- Sustituir el L298N por un TB6612FNG, con una caída de unos 0,5 V frente a los 2-3 V del L298N.
- Medir la constante de tiempo mecánica real con un escalón de PWM y actualizar la inercia equivalente.
- Añadir la curva de PWM en un segundo eje de la gráfica del motor físico, para diagnosticar la saturación de un vistazo.
- Marcar automáticamente sobre las gráficas el sobreimpulso, el tiempo de subida y el de establecimiento.
- Incorporar al simulador el retardo de la medición y un actuador limitado de 0 a 100 %.
- Exportar los ensayos reales a un archivo de datos.
- Subir el puerto serie a 115 200 baudios si hace falta más información por segundo.

---

## Informe

El informe completo del proyecto, en formato IEEE, está en [`docs/informe.pdf`](docs/informe.pdf). Incluye el marco teórico, la metodología, los diagramas de bloques de ambos programas, las evidencias de los ensayos y el análisis de resultados.

---

## Autor

**Carlos Andrés Batista Figueroa** — T00078355
Ingeniería Mecatrónica, Universidad Tecnológica de Bolívar
Asignatura: Control Inteligente · Docente: Ing. Ph.D. Efraín Rodríguez
Cartagena de Indias, Colombia · 2026
