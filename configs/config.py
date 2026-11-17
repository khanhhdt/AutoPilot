import os
from dataclasses import dataclass

@dataclass(frozen=True)
class Config:
    host: str = os.environ.get("CARLA_HOST", "100.85.59.63")
    port: int = int(os.environ.get("CARLA_PORT", "3000"))
    timeout: float = 30.0

    fixed_delta: float = 0.05
    sensor_tick: float = 0.0

    rgb_width: int = 512
    rgb_height: int = 288
    aux_width: int = 192
    aux_height: int = 108
    fov: int = 90

    vehicle_filter: str = "vehicle.tesla.model3"


CFG = Config()
