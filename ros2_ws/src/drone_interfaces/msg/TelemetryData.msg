# Úhly natočení (Euler angles)
int16 pitch       # [°] - náklon dopředu/dozadu (-90° až +90°)
int16 roll        # [°] - náklon doleva/doprava (-90° až +90°)
int16 yaw         # [°] - otočení kolem vertikální osy (-180° až +180°)

# Rychlosti v jednotlivých osách (ground speed)
int16 vgx         # [cm/s] - rychlost ve směru X (dopředu/dozadu)
int16 vgy         # [cm/s] - rychlost ve směru Y (doleva/doprava)
int16 vgz         # [cm/s] - rychlost ve směru Z (nahoru/dolů)

# Teploty senzorů
int8 templ        # [°C] - minimální teplota IMU
int8 temph        # [°C] - maximální teplota IMU

# Výška a vzdálenost
uint16 tof        # [cm] - vzdálenost od země (ToF senzor dolů, 10-1200cm)
int16 h           # [cm] - relativní výška od místa vzletu

# Stav baterie
uint8 bat         # [%] - úroveň baterie (0-100%)

# Barometrická výška
float32 baro      # [cm] - absolutní výška z barometru

# Doba letu
uint32 time       # [ms] - čas od zapnutí motoru

# Akcelerometry (IMU data)
float32 agx       # [0.001g] - zrychlení osy X (1g = 9.81 m/s²)
float32 agy       # [0.001g] - zrychlení osy Y
float32 agz       # [0.001g] - zrychlení osy Z
