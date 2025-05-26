# PRACTICA-8
# Comunicación Serie Bidireccional entre UART0 y UART2

## Objetivo

Establecer una comunicación continua en ambas direcciones entre dos interfaces UART (UART0 y UART2) de un ESP32, permitiendo reenviar los datos entre un teclado conectado por terminal serie y otro puerto UART simulado.

---

## Componentes necesarios

- Placa **ESP32-S1**
- Monitor serie del ordenador
- Teclado (input desde el terminal serie)
- Cable de puente (para conectar TX2 ↔ RX2 en la placa)

---

## Implementación

### Código principal

```cpp
#include <Arduino.h>

void setup() {
  Serial.begin(115200);   // Inicializar Serial (UART0)
  Serial2.begin(115200);  // Inicializar Serial2 (UART2)
}

void loop() {
  // Leer datos de la UART0 y enviarlos a la UART2
  if (Serial.available()) {
    char c = Serial.read();
    Serial2.write(c);
  }
  
  // Leer datos de la UART2 y enviarlos a la UART0
  if (Serial2.available()) {
    char c = Serial2.read();
    Serial.write(c);
  }
}
```
## Conclusion

El programa establece una comunicación bidireccional entre los puertos UART0 y UART2 del ESP32. En la función setup(), ambos puertos se inicializan a 115200 baudios. Luego, en el loop(), se comprueba constantemente si hay datos disponibles en alguno de los dos puertos. Si se detecta información en UART0, se lee y se envía a UART2; si es en UART2, se reenvía a UART0. De este modo, se crea un bucle continuo de transmisión y recepción de datos entre ambos puertos.
