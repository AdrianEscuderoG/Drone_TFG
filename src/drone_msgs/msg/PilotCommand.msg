# Comando asíncrono para el piloto autónomo (pilot_node)
#
# Tipos de comando soportados:
#   TAKEOFF        — despegar hasta takeoff_altitude (0 = usar valor por defecto)
#   GOTO_RELATIVE  — moverse a [x, y, z, yaw] relativo al marco del dron
#                    (x: adelante, y: izquierda, z: arriba, yaw: giro relativo)
#   GOTO_GLOBAL    — moverse a [x, y, z, yaw] en el marco global ENU (map)
#   LAND           — aterrizar en la posición actual
#   EMERGENCY      — abortar inmediatamente y aterrizar
#
# Tras TAKEOFF, GOTO_RELATIVE o GOTO_GLOBAL, el dron se queda en HOVERING
# (estático) en el punto de destino hasta recibir el siguiente comando.

uint8 TAKEOFF = 0
uint8 GOTO_RELATIVE = 1
uint8 GOTO_GLOBAL = 2
uint8 LAND = 3
uint8 EMERGENCY = 4

std_msgs/Header header
uint8 command_type

# Punto objetivo [x, y, z, yaw] — interpretación según command_type
float32 x
float32 y
float32 z
float32 yaw

# Solo para TAKEOFF (metros). 0.0 = usar default_takeoff_alt del piloto.
float32 takeoff_altitude
