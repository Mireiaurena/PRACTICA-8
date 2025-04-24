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

#define UART2_RX 16  // Puedes cambiar estos pines si es necesario
#define UART2_TX 17

HardwareSerial UART2(2); // UART2 es el puerto serial número 2

void setup() {
  // Iniciar UART0 (Serial) para el monitor serie
  Serial.begin(115200);
  while (!Serial) {
    ; // Espera a que se conecte el terminal
  }

  // Iniciar UART2 en 115200 baudios
  UART2.begin(115200, SERIAL_8N1, UART2_RX, UART2_TX);
  Serial.println("Inicializado. Comunicando UART0 <-> UART2");
}

void loop() {
  // Leer de Serial (UART0) y enviar a UART2
  if (Serial.available()) {
    char data = Serial.read();
    UART2.write(data);
  }

  // Leer de UART2 y enviar a Serial (UART0)
  if (UART2.available()) {
    char data = UART2.read();
    Serial.write(data);
  }
}
