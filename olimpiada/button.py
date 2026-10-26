import math
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
    print(step, round(r["x"], 3), round(r["y"], 3), r["zones"],
          g["state"], round(g["angle"], 2),
          s["reactor"]["state"], s["reactor"]["gatesLockInMs"])
    return s

def go(x, y, v=0.3, tol=0.01):
    client.move_to(R, x, y, speed=v, tolerance=tol, timeout=30)

def open_gate():
    client.open_gate("gate.1")
    client.wait_until(lambda s: next(g for g in s["gates"]
                      if g["id"] == "gate.1")["angle"] > 1.5, timeout=5)

info("старт")
go(0.6, 1.4)
client.rotate_to(R, math.pi / 2, angle_tolerance=0.01)   # носом на север
go(0.6, 2.2)
go(1.007, 2.2)
info("у шлагбаума")
open_gate()
go(1.007, 2.75)
go(1.104, 2.75)
info("перед кнопкой")

go(1.104, 2.815, v=0.05, tol=0.003)                      # подъезд к кнопке
for _ in range(4):                                       # дожать по 3 мм
    if client.get_state()["reactor"]["state"] == "off":
        break
    client.drive(R, 0.003, speed=0.02)
info("кнопка")

go(1.007, 2.75)                                          # выезд, пока не заблокировало
go(1.007, 2.2)
info("снаружи")