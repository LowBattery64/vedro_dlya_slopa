import math, time
from quasar_client import QuasarClient

client = QuasarClient.from_environment()
client.wait_for_simulation_start()
client.reset()
client.configure_incidents([])        # отладка: без киберпрепятствий

R = "robot.1"

def info(step):
    s = client.get_state()
    r = s["robots"][0]
    g = next(g for g in s["gates"] if g["id"] == "gate.1")
    print(step, round(r["x"], 3), round(r["y"], 3), r["zones"], r["holding"],
          g["state"], s["reactor"]["state"], s["reactor"].get("gatesLockInMs"))
    return s

def go(x, y, v=0.3, tol=0.01):
    client.move_to(R, x, y, speed=v, tolerance=tol, timeout=30)

def open_gate():
    client.open_gate("gate.1")
    client.wait_until(lambda s: next(g for g in s["gates"]
                      if g["id"] == "gate.1")["angle"] > 1.5, timeout=5)

def grab(item):
    client.set_magnet(R, True)
    client.set_velocity(R, 0.03)          # медленно вперёд до захвата
    for _ in range(200):
        if client.get_state()["robots"][0]["holding"] == item:
            break
        time.sleep(0.05)
    client.stop(R)

# --- Задача 1: кнопка ---
info("старт")
go(0.6, 1.4)
client.rotate_to(R, math.pi / 2, angle_tolerance=0.01)   # носом на север
go(0.6, 2.2)
go(1.007, 2.2)
open_gate()
go(1.007, 2.75)
go(1.104, 2.75)
go(1.104, 2.815, v=0.05, tol=0.003)
for _ in range(4):                                       # дожать по 3 мм
    if client.get_state()["reactor"]["state"] == "off":
        break
    client.drive(R, 0.003, speed=0.02)
info("кнопка")
go(1.007, 2.75)
go(1.007, 2.2)
info("снаружи")

# --- Задача 2: катушка ---
client.rotate_to(R, 0, angle_tolerance=0.01)             # носом на восток
go(2.975, 2.2)
go(2.975, 2.65)
go(2.87, 2.65)
go(2.87, 2.957, v=0.1, tol=0.004)                        # напротив катушки
client.rotate_to(R, 0, angle_tolerance=0.01)
grab("spool")
info("катушка взята")
go(2.975, 0.6)                                           # вдоль восточного края
go(2.954, 0.21, v=0.1, tol=0.005)                        # катушка над приёмником
client.set_magnet(R, False)
time.sleep(1)
s = info("катушка")
print(s["spool"], s["charging"]["state"])