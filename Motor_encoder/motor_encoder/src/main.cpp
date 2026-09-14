#include <Arduino.h>
#include <ctype.h>

/*
 * Control de velocidad de un motor DC: PWM manual + PID de RPM
 * Arduino Uno + puente H L298N + sensor optico de ranura (FC-03 / MH-Sensor)
 *
 * Conexiones:
 *   L298N  ENA -> D3   (salida PWM, ~490 Hz por Timer2)
 *   L298N  IN1 -> D7
 *   L298N  IN2 -> D8
 *   FC-03  D0  -> D2   (INT0: la cuenta de pulsos se hace por interrupcion)
 *   FC-03  VCC -> 5V del Arduino
 *   FC-03  GND -> GND del Arduino
 *   Todos los GND (Arduino, L298N y bateria) unidos entre si.
 *
 * Comandos del monitor serial (9600 baudios), siempre con Enter al final:
 *   rpm 300    -> setpoint de 300 RPM y ACTIVA el PID (-max a max)
 *   pwm 60     -> fija 60 % de PWM y DESACTIVA el PID (-100 a 100)
 *   60         -> atajo equivalente a "pwm 60"
 *   kp 3.5     -> constante proporcional (1 a 10)
 *   ki 40      -> constante integral (0 a 200)
 *   kd 0.5     -> constante derivativa (0 a 2)
 *   max 600    -> RPM reales del motor con PWM al 100 % (10 a 4000)
 *   zm 15      -> zona muerta: PWM minimo de la prealimentacion (0 a 90)
 *   ff         -> activar / desactivar la prealimentacion (arranca activada)
 *   s          -> detener el motor (tambien desactiva el PID)
 *   r 20       -> ranuras por vuelta del disco encoder
 *   m          -> activar / desactivar el reporte periodico
 *   ?          -> mostrar la ayuda
 *
 * El signo indica el sentido de giro, tanto en "rpm" como en "pwm". El sensor
 * de ranura no distingue el sentido, asi que el PID regula la MAGNITUD de la
 * velocidad y el sentido lo impone el signo del setpoint.
 */

const uint8_t PIN_ENA = 3;
const uint8_t PIN_IN1 = 7;
const uint8_t PIN_IN2 = 8;
const uint8_t PIN_SENSOR = 2;
// contarPulso() lee el pin directo como PD2 por velocidad: si mueves el
// sensor de pin, hay que ajustar tambien esa lectura.
static_assert(PIN_SENSOR == 2, "contarPulso() lee PD2 directamente");

const unsigned long BAUDIOS = 9600;

// Velocidad real del motor con el PWM al 100 %. Es el fondo de escala del
// sistema: normaliza el error del PID (asi las constantes quedan en los rangos
// kp 1-10, ki 0-200, kd 0-2), limita el setpoint y escala la prealimentacion.
// Se cambia en caliente con el comando max.
float rpmMax = 600.0f;
const float RPMMAX_MIN = 10.0f, RPMMAX_MAX = 4000.0f;

// Prealimentacion (feedforward): ff = pwmMin + |SP| * (100 - pwmMin) / rpmMax.
// pwmMin es la zona muerta medida: el motor arranca con 19 % y se sigue
// moviendo hasta 15 %. Se cambia con el comando zm; ff la activa/desactiva.
float pwmMin = 15.0f;
const float ZM_MIN = 0.0f, ZM_MAX = 90.0f;
bool ffActivo = true;

const unsigned long PERIODO_PID_MS = 25;       // lectura del sensor y PID: 40 Hz
const unsigned long PERIODO_REPORTE_MS = 250;  // impresion por serial: 4 Hz

// Ranuras (o aspas) del disco encoder. Los discos tipicos traen 20; cambialo
// aqui o en caliente con el comando r.
uint16_t ranurasPorVuelta = 20;

// Ancho minimo entre flancos validos. Filtra los rebotes del comparador LM393
// en el borde de cada ranura. 250 us admiten hasta 4000 pulsos/s (12000 RPM
// con un disco de 20 ranuras), de sobra para este motor.
const unsigned long MIN_PULSO_US = 250;

// Sin pulsos durante este tiempo se considera el motor detenido.
const unsigned long TIEMPO_SIN_PULSOS_US = 300000UL;

// La linea se arma caracter por caracter y se procesa unicamente al recibir el
// fin de linea: el monitor serial envia cada tecla por separado, asi que
// cualquier procesado por tiempo partiria "10" en un 1 y luego un 0.
const uint8_t LARGO_BUFFER = 24;
char buffer[LARGO_BUFFER];
uint8_t indiceBuffer = 0;

// --- Estado del control ---
bool pidActivo = false;
float setpointRpm = 0.0f;       // con signo: negativo = sentido inverso
float salidaPorcentaje = 0.0f;  // PWM aplicado, con signo
float rpmMedida = 0.0f;

// Constantes del PID. Valores de arranque conservadores: PI puro, sin
// derivativo, que es lo mas estable para empezar a sintonizar.
float kp = 3.0f;
float ki = 40.0f;
float kd = 0.0f;

const float KP_MIN = 1.0f, KP_MAX = 10.0f;
const float KI_MIN = 0.0f, KI_MAX = 200.0f;
const float KD_MIN = 0.0f, KD_MAX = 2.0f;

float integral = 0.0f;
float rpmPrevia = 0.0f;
float errorActualPct = 0.0f;

bool reporteActivo = true;
unsigned long ultimoControlMs = 0;
unsigned long ultimoReporteMs = 0;
unsigned long pulsosAcumulados = 0;

// Referencia temporal de la medicion: instante del ultimo flanco procesado.
unsigned long anclaUs = 0;
bool anclaValida = false;

// Ultimas 5 medidas crudas, para la mediana que filtra picos aislados.
const uint8_t N_MEDIANA = 5;
float hist[N_MEDIANA] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

// Compartidas con la ISR: volatile para que el compilador no las cachee en un
// registro y las relea de memoria en cada acceso.
volatile unsigned long contadorPulsos = 0;
volatile unsigned long tUltimoPulsoUs = 0;

void mostrarAyuda();
void procesarLinea(char *linea);
void aplicarSalida(float porcentaje);
void actualizarMedicion();
float medianaHist();
void controlPid(float dtS);
void reiniciarPid();
void imprimirEstado(const __FlashStringHelper *prefijo);
void reportar();
void contarPulso();

void setup() {
  pinMode(PIN_ENA, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_SENSOR, INPUT);  // el modulo ya entrega un nivel digital firme

  Serial.begin(BAUDIOS);
  aplicarSalida(0.0f);  // arrancar siempre con el motor detenido

  attachInterrupt(digitalPinToInterrupt(PIN_SENSOR), contarPulso, FALLING);
  ultimoControlMs = millis();
  ultimoReporteMs = ultimoControlMs;

  mostrarAyuda();
}

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (indiceBuffer > 0) {  // ignora el segundo caracter de un CR+LF
        buffer[indiceBuffer] = '\0';
        procesarLinea(buffer);
        indiceBuffer = 0;
      }
    } else if (indiceBuffer < LARGO_BUFFER - 1) {
      buffer[indiceBuffer++] = c;
    }
    // Si la trama excede el buffer se ignoran los caracteres sobrantes.
  }

  unsigned long ahoraMs = millis();

  if (ahoraMs - ultimoControlMs >= PERIODO_PID_MS) {
    // dt real, no el nominal: si el loop se retrasa, el integral y el
    // derivativo se calculan sobre el tiempo que de verdad transcurrio.
    float dtS = (float)(ahoraMs - ultimoControlMs) / 1000.0f;
    ultimoControlMs = ahoraMs;

    actualizarMedicion();
    if (pidActivo) {
      controlPid(dtS);
    }
  }

  if (ahoraMs - ultimoReporteMs >= PERIODO_REPORTE_MS) {
    ultimoReporteMs = ahoraMs;
    reportar();
  }
}

// ISR: lo mas corta posible. Descarta picos de ruido y rebotes, y suma.
void contarPulso() {
  // Un pico inducido por el PWM dura pocos microsegundos; una ranura real
  // mantiene la linea en bajo cientos. Si tras unos us volvio a alto, era ruido.
  delayMicroseconds(8);
  if (PIND & _BV(PD2)) {  // D2 = PD2 en el Uno
    return;
  }
  unsigned long ahoraUs = micros();
  if (ahoraUs - tUltimoPulsoUs < MIN_PULSO_US) {
    return;
  }
  tUltimoPulsoUs = ahoraUs;
  contadorPulsos++;
}

void actualizarMedicion() {
  unsigned long pulsos;
  unsigned long tUltimo;

  // Lectura atomica: son variables de 4 bytes en un micro de 8 bits, asi que
  // sin esto una interrupcion podria colarse a mitad de la copia.
  noInterrupts();
  pulsos = contadorPulsos;
  contadorPulsos = 0;
  tUltimo = tUltimoPulsoUs;
  interrupts();

  pulsosAcumulados += pulsos;

  if (pulsos > 0) {
    if (!anclaValida) {
      // Primer pulso tras un arranque: solo se fija la referencia, todavia no
      // hay un intervalo valido que medir.
      anclaUs = tUltimo;
      anclaValida = true;
    } else {
      unsigned long dtUs = tUltimo - anclaUs;
      anclaUs = tUltimo;
      if (dtUs > 0) {
        // Se mide de flanco a flanco, no de borde a borde de la ventana: asi
        // no hay error de cuantizacion por contar pulsos en un intervalo fijo,
        // que a 25 ms serian saltos de 120 RPM.
        float cruda = (60000000.0f * (float)pulsos) /
                      ((float)dtUs * (float)ranurasPorVuelta);
        // Desplaza el historial y agrega la medida nueva al final.
        for (uint8_t i = 0; i < N_MEDIANA - 1; i++) {
          hist[i] = hist[i + 1];
        }
        hist[N_MEDIANA - 1] = cruda;
        // Mediana de 5: hasta dos picos entre las ultimas 5 nunca pasan al PID.
        rpmMedida = medianaHist();
      }
    }
  } else if (micros() - anclaUs > TIEMPO_SIN_PULSOS_US) {
    rpmMedida = 0.0f;
    for (uint8_t i = 0; i < N_MEDIANA; i++) {
      hist[i] = 0.0f;  // sin historia vieja al rearrancar
    }
    anclaValida = false;
  }
}

// Mediana del historial: ordena una copia por insercion (son solo 5 valores)
// y devuelve el valor central.
float medianaHist() {
  float orden[N_MEDIANA];
  for (uint8_t i = 0; i < N_MEDIANA; i++) {
    float v = hist[i];
    int8_t j = (int8_t)i - 1;
    while (j >= 0 && orden[j] > v) {
      orden[j + 1] = orden[j];
      j--;
    }
    orden[j + 1] = v;
  }
  return orden[N_MEDIANA / 2];
}

void controlPid(float dtS) {
  float spAbs = fabs(setpointRpm);

  if (spAbs < 1.0f) {  // setpoint cero: parar y limpiar el integrador
    integral = 0.0f;
    errorActualPct = 0.0f;
    rpmPrevia = rpmMedida;
    aplicarSalida(0.0f);
    return;
  }

  // Prealimentacion: el PWM que, segun la recta entre la zona muerta y
  // rpmMax, ya deberia dar la velocidad pedida. Asi el PID solo corrige lo que
  // el modelo no acierta, en vez de construir todo el esfuerzo desde el
  // integrador y atravesar la zona muerta a base de acumular error.
  float ff = 0.0f;
  if (ffActivo) {
    ff = constrain(pwmMin + spAbs * (100.0f - pwmMin) / rpmMax, 0.0f, 100.0f);
  }

  // Error normalizado a puntos porcentuales del fondo de escala: con esto las
  // constantes viven en los rangos pedidos en vez de tener que ser diminutas
  // para compensar errores de cientos de RPM.
  float errorPct = (spAbs - rpmMedida) * 100.0f / rpmMax;
  errorActualPct = errorPct;

  float p = kp * errorPct;

  // Derivativo sobre la medicion, no sobre el error: evita el salto brusco de
  // salida cada vez que se cambia el setpoint.
  float d = -kd * ((rpmMedida - rpmPrevia) * 100.0f / rpmMax) / dtS;

  float integralTent = integral + ki * errorPct * dtS;
  float salidaTent = ff + p + integralTent + d;

  // Anti-windup: el integrador solo acumula si la salida no esta saturada, o
  // si el error ya empuja de vuelta hacia el rango util. Sin esto, arrancar
  // con el motor frenado cargaria el integrador hasta valores enormes y el
  // motor no obedeceria al bajar el setpoint. La saturacion se evalua con el
  // ff incluido: el ff ya ocupa parte del rango, y sin contarlo el integrador
  // seguiria cargandose con la salida real ya clavada en 100 %.
  bool satAlta = (salidaTent > 100.0f);
  bool satBaja = (salidaTent < 0.0f);
  if ((!satAlta && !satBaja) ||
      (satAlta && errorPct < 0.0f) ||
      (satBaja && errorPct > 0.0f)) {
    integral = integralTent;
  }

  float salida = constrain(ff + p + integral + d, 0.0f, 100.0f);
  rpmPrevia = rpmMedida;

  // El PID regula la magnitud; el sentido lo decide el signo del setpoint.
  aplicarSalida(setpointRpm >= 0.0f ? salida : -salida);
}

void reiniciarPid() {
  integral = 0.0f;
  rpmPrevia = rpmMedida;
  errorActualPct = 0.0f;
}

// Escribe en el puente H sin imprimir nada: la llama el PID 40 veces por
// segundo y cualquier Serial aqui inundaria el monitor.
void aplicarSalida(float porcentaje) {
  porcentaje = constrain(porcentaje, -100.0f, 100.0f);
  salidaPorcentaje = porcentaje;

  if (porcentaje > 0.0f) {
    digitalWrite(PIN_IN1, HIGH);
    digitalWrite(PIN_IN2, LOW);
  } else if (porcentaje < 0.0f) {
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, HIGH);
  } else {
    // Ambas entradas en bajo: el motor queda libre (sin freno).
    digitalWrite(PIN_IN1, LOW);
    digitalWrite(PIN_IN2, LOW);
  }

  uint8_t pwm = (uint8_t)(fabs(porcentaje) * 255.0f / 100.0f + 0.5f);
  analogWrite(PIN_ENA, pwm);
}

void imprimirEstado(const __FlashStringHelper *prefijo) {
  Serial.print(prefijo);
  if (pidActivo) {
    Serial.print(F("  SP: "));
    Serial.print(setpointRpm, 0);
    Serial.print(F(" RPM"));
  }
  Serial.print(F("  PWM: "));
  Serial.print(salidaPorcentaje, 1);
  Serial.print(F(" %  sentido: "));
  if (salidaPorcentaje > 0.0f) {
    Serial.println(F("horario"));
  } else if (salidaPorcentaje < 0.0f) {
    Serial.println(F("antihorario"));
  } else {
    Serial.println(F("detenido"));
  }
}

void reportar() {
  unsigned long pulsos = pulsosAcumulados;
  pulsosAcumulados = 0;

  if (!reporteActivo) {
    return;
  }

  if (pidActivo) {
    Serial.print(F("[PID] SP: "));
    Serial.print(fabs(setpointRpm), 0);
    Serial.print(F("  RPM: "));
    Serial.print(rpmMedida, 1);
    Serial.print(F("  PWM: "));
    Serial.print(fabs(salidaPorcentaje), 1);
    Serial.print(F(" %  err: "));
    Serial.print(errorActualPct * rpmMax / 100.0f, 1);
    Serial.print(F("  pulsos: "));
    Serial.println(pulsos);
  } else {
    Serial.print(F("[MAN] RPM: "));
    Serial.print(rpmMedida, 1);
    Serial.print(F("  PWM: "));
    Serial.print(fabs(salidaPorcentaje), 1);
    Serial.print(F(" %  pulsos: "));
    Serial.println(pulsos);
  }
}

void mostrarAyuda() {
  Serial.println();
  Serial.println(F("=== Control de motor DC: PWM manual + PID de RPM ==="));
  Serial.println(F("  rpm <v>  -> setpoint en RPM y activa el PID (-max a max)"));
  Serial.println(F("  pwm <v>  -> PWM fijo y desactiva el PID (-100 a 100)"));
  Serial.println(F("  <v>      -> atajo de 'pwm <v>'"));
  Serial.println(F("  kp <v>   -> proporcional (1 a 10)"));
  Serial.println(F("  ki <v>   -> integral (0 a 200)"));
  Serial.println(F("  kd <v>   -> derivativo (0 a 2)"));
  Serial.println(F("  max <v>  -> RPM del motor con PWM al 100 % (10 a 4000)"));
  Serial.println(F("  zm <v>   -> zona muerta, PWM minimo del ff (0 a 90)"));
  Serial.println(F("  ff       -> activar/desactivar la prealimentacion"));
  Serial.println(F("  s        -> detener"));
  Serial.println(F("  r <v>    -> ranuras por vuelta del disco"));
  Serial.println(F("  m        -> activar/desactivar el reporte"));
  Serial.println(F("  ?        -> esta ayuda"));
  Serial.print(F("Kp="));
  Serial.print(kp, 2);
  Serial.print(F("  Ki="));
  Serial.print(ki, 2);
  Serial.print(F("  Kd="));
  Serial.print(kd, 2);
  Serial.print(F("  ranuras="));
  Serial.print(ranurasPorVuelta);
  Serial.print(F("  modo="));
  Serial.println(pidActivo ? F("PID") : F("MANUAL"));
  Serial.print(F("max="));
  Serial.print(rpmMax, 1);
  Serial.print(F("  zm="));
  Serial.print(pwmMin, 1);
  Serial.print(F("  ff="));
  Serial.println(ffActivo ? F("activada") : F("desactivada"));
  Serial.println();
}

// Informa un valor fuera de rango de forma uniforme.
static void errorRango(const __FlashStringHelper *que, float minimo, float maximo) {
  Serial.print(F("Fuera de rango: "));
  Serial.print(que);
  Serial.print(F(" debe ir de "));
  Serial.print(minimo, 2);
  Serial.print(F(" a "));
  Serial.println(maximo, 2);
}

void procesarLinea(char *linea) {
  while (*linea == ' ' || *linea == '\t') {
    linea++;
  }

  if (*linea == '\0') {
    return;
  }

  if (*linea == '?') {
    mostrarAyuda();
    return;
  }

  // Se separa el comando (letras) del argumento (numero), de modo que "rpm800",
  // "rpm 800" y "rpm=800" se acepten por igual.
  char comando[8];
  uint8_t n = 0;
  while (isalpha((unsigned char)*linea) && n < sizeof(comando) - 1) {
    comando[n++] = (char)tolower((unsigned char)*linea);
    linea++;
  }
  comando[n] = '\0';

  while (*linea == ' ' || *linea == '\t' || *linea == '=' || *linea == ':') {
    linea++;
  }

  char *fin = NULL;
  float valor = strtod(linea, &fin);
  bool hayNumero = (fin != linea);

  // Comandos sin argumento.
  if (strcmp(comando, "s") == 0 || strcmp(comando, "stop") == 0) {
    pidActivo = false;
    setpointRpm = 0.0f;
    reiniciarPid();
    aplicarSalida(0.0f);
    imprimirEstado(F("[MAN] detenido, PID desactivado."));
    return;
  }

  // strcmp exacto: "max" y "zm" no caen aqui aunque contengan una m.
  if (strcmp(comando, "m") == 0) {
    reporteActivo = !reporteActivo;
    Serial.print(F("Reporte: "));
    Serial.println(reporteActivo ? F("activado") : F("desactivado"));
    return;
  }

  if (strcmp(comando, "ff") == 0) {
    ffActivo = !ffActivo;
    Serial.print(F("Prealimentacion: "));
    Serial.println(ffActivo ? F("activada") : F("desactivada"));
    return;
  }

  // De aqui en adelante todos los comandos necesitan un valor.
  if (!hayNumero) {
    Serial.print(F("Falta el valor numerico en: "));
    Serial.println(buffer);
    return;
  }

  if (strcmp(comando, "rpm") == 0) {
    if (valor < -rpmMax || valor > rpmMax) {
      errorRango(F("el setpoint"), -rpmMax, rpmMax);
      return;
    }
    bool veniaDeManual = !pidActivo;
    setpointRpm = valor;
    pidActivo = true;
    if (veniaDeManual) {
      reiniciarPid();  // arranque limpio del integrador al entrar al PID
    }
    imprimirEstado(F("[PID] activado."));
    return;
  }

  // n == 0 significa que la linea era solo un numero: atajo de "pwm <v>".
  if (strcmp(comando, "pwm") == 0 || n == 0) {
    if (valor < -100.0f || valor > 100.0f) {
      errorRango(F("el PWM"), -100.0f, 100.0f);
      return;
    }
    pidActivo = false;
    setpointRpm = 0.0f;
    reiniciarPid();
    aplicarSalida(valor);
    imprimirEstado(F("[MAN] PID desactivado."));
    return;
  }

  if (strcmp(comando, "kp") == 0) {
    if (valor < KP_MIN || valor > KP_MAX) {
      errorRango(F("Kp"), KP_MIN, KP_MAX);
      return;
    }
    kp = valor;
    Serial.print(F("Kp = "));
    Serial.println(kp, 2);
    return;
  }

  if (strcmp(comando, "ki") == 0) {
    if (valor < KI_MIN || valor > KI_MAX) {
      errorRango(F("Ki"), KI_MIN, KI_MAX);
      return;
    }
    ki = valor;
    integral = 0.0f;  // lo acumulado ya no corresponde a la nueva Ki
    Serial.print(F("Ki = "));
    Serial.println(ki, 2);
    return;
  }

  if (strcmp(comando, "kd") == 0) {
    if (valor < KD_MIN || valor > KD_MAX) {
      errorRango(F("Kd"), KD_MIN, KD_MAX);
      return;
    }
    kd = valor;
    Serial.print(F("Kd = "));
    Serial.println(kd, 2);
    return;
  }

  if (strcmp(comando, "max") == 0) {
    if (valor < RPMMAX_MIN || valor > RPMMAX_MAX) {
      errorRango(F("max"), RPMMAX_MIN, RPMMAX_MAX);
      return;
    }
    rpmMax = valor;
    reiniciarPid();  // el error normalizado cambio de escala: integrador a cero
    Serial.print(F("max = "));
    Serial.println(rpmMax, 1);
    return;
  }

  if (strcmp(comando, "zm") == 0) {
    if (valor < ZM_MIN || valor > ZM_MAX) {
      errorRango(F("zm"), ZM_MIN, ZM_MAX);
      return;
    }
    pwmMin = valor;
    Serial.print(F("zm = "));
    Serial.println(pwmMin, 1);
    return;
  }

  if (strcmp(comando, "r") == 0) {
    if (valor < 1.0f || valor > 1000.0f) {
      errorRango(F("las ranuras"), 1.0f, 1000.0f);
      return;
    }
    ranurasPorVuelta = (uint16_t)valor;
    Serial.print(F("Ranuras por vuelta: "));
    Serial.println(ranurasPorVuelta);
    return;
  }

  Serial.print(F("Comando no reconocido: "));
  Serial.println(buffer);
}
