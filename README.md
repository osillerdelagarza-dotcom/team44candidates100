
# Robot Candidates 2026 - avance de software

Proyecto base para ESP32-WROOM-32 + PlatformIO.

## Pistas implementadas

### Pista A - MAZE
- Exploración dinámica de la cuadrícula.
- Lectura de paredes con 3 HC-SR04P + medición tras giro de 180°.
- Movimiento por celdas de 30 cm mediante tiempo calibrable.
- Detección de los cuatro colores del reglamento:
  - Cian
  - Amarillo
  - Naranja
  - Rosa
- Detección de verde (inicio) y rojo (checkpoint/final).
- Resultado del color mostrado en LCD.
- No incluye los bonus ArUco ni retorno por el mismo camino.

### Pista B - Niveles

Sección 1:
- Exploración de una cuadrícula 3x3.
- Búsqueda de una celda candidata con tres paredes.
- Aproximación mediante ultrasonido y cierre de garra.
- Búsqueda posterior del checkpoint rojo.

Sección 2:
- Evitación de líneas blancas usando el TCRT5000 de 3 canales.
- El robot intenta permanecer en el espacio libre.

Sección 3:
- Lectura de color del tile.
- Mapeo exacto del reglamento:
  - Cian -> derecha
  - Amarillo -> izquierda
  - Naranja -> arriba
  - Rosa -> abajo
- Las direcciones se tratan como absolutas respecto a la pista.
- Verde -> final.

## Selección de pista

En `src/main.cpp`:

#define ACTIVE_TRACK TRACK_A

o

#define ACTIVE_TRACK TRACK_B

## Pines usados

L298N:
- ENA GPIO14
- IN1 GPIO27
- IN2 GPIO25
- IN3 GPIO26
- IN4 GPIO33
- ENB GPIO32

HC-SR04P:
- Izquierdo TRIG18 ECHO34
- Frontal TRIG19 ECHO36
- Derecho TRIG21 ECHO35

I2C:
- SDA GPIO22
- SCL GPIO23

Servo:
- GPIO5

TCRT5000:
- Izquierdo GPIO4
- Centro GPIO16
- Derecho GPIO17

IR:
- GPIO15
- GPIO2

## Dependencias

- Adafruit TCS34725
- johnrickman/LiquidCrystal_I2C
- madhephaestus/ESP32Servo

## Antes de usarlo en competencia

1. Calibrar CELL_FORWARD_MS.
2. Calibrar TURN_90_MS.
3. Calibrar BASE_SPEED y TURN_SPEED.
4. Verificar la polaridad de los motores.
5. Verificar el nivel lógico del ECHO de los HC-SR04P.
6. Ejecutar CALIBRATION_MODE=1 para obtener RGB/HSV y TCRT.
7. Ajustar los umbrales de color a la iluminación real de la pista.
8. Ajustar ángulos del servo.
9. Verificar físicamente que la garra mantenga >50% de la pelota dentro del perímetro del robot.
10. Probar cada sección por separado y después el recorrido completo.

## Advertencia sobre el esquema eléctrico

Los motores TT indicados son de 3-6 V. No conectar la alimentación de motor del L298N a 12 V.

La alimentación de motores debe ser adecuada para esos TT. El L298N además introduce caída de tensión y disipación.

El LCD y el TCS34725 comparten el bus I2C:
SDA -> GPIO22
SCL -> GPIO23

No hay encoders en el esquemático actual. Por eso el control de distancia y giro es temporal y requiere calibración.

## Reglamento

Este proyecto es un punto de partida técnico. El reglamento indica que el código y las soluciones deben ser de autoría del equipo y que los integrantes deben comprender y poder explicar, modificar y justificar su programación.
