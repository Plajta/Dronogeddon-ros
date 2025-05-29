# Úhly natočení
int16 pitch       # [°]
int16 roll        # [°]
int16 yaw         # [°]

# Rychlosti v jednotlivých osách
int16 vgx         # [cm/s]
int16 vgy         # [cm/s]
int16 vgz         # [cm/s]

# Teploty
int8 templ        # [°C] - minimální teplota
int8 temph        # [°C] - maximální teplota

# Výška a vzdálenost
uint16 tof        # [cm] - vzdálenost od země (Time-of-Flight)
int16 h           # [cm] - výška od vzletu

# Stav baterie
uint8 bat         # [%]

# Barometrická výška
float32 baro      # [cm]

# Doba letu
uint32 time       # [ms]

# Akcelerometry
float32 agx       # [0.001g]
float32 agy       # [0.001g]
float32 agz       # [0.001g]
