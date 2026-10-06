#!/usr/bin/env python3
"""
Заготовка для робота на MaixCam Lite (4 датчика линии, лазерный дальномер,
4 независимых колеса, внешняя кнопка).

Что внутри:
  1. CONFIG        - все числа, которые придётся подбирать
  2. Robot         - ЕДИНСТВЕННОЕ место, где нужен API железа (ищите TODO)
  3. MockRobot     - модель для проверки логики на компьютере (python robot_main.py --sim)
  4. Обработка датчиков линии, ПИД, определение перекрёстка
  5. Главный цикл  - кнопка старт/стоп, следование по линии, объезд препятствия

Названий функций maixcam_bridge_2 я не знаю, поэтому там стоят заглушки.
"""
import sys
import time

# =====================================================================
# 1. CONFIG
# =====================================================================
LOOP_PERIOD_S = 0.02          # период цикла управления, 50 Гц

# --- датчики линии ---
ADC_MAX = 4095
LINE_IS_HIGH = True           # TODO: True, если линия даёт БОЛЬШОЕ значение АЦП; проверьте на роботе
SENSOR_WEIGHTS = [-3, -1, 1, 3]   # положения датчиков слева направо
LINE_THRESHOLD = 0.5          # после нормировки 0..1: выше = датчик на линии
LOST_SUM = 0.3                # сумма нормированных значений ниже = линия потеряна
JUNCTION_COUNT = 3            # столько датчиков одновременно на линии = перекрёсток

# --- ПИД ---
KP, KI, KD = 0.5, 0.0, 0.02  # подбирать на трассе, начните с одного KP
BASE_SPEED = 0.35             # 0..1
MAX_SPEED = 0.8
SEARCH_SPEED = 0.30           # скорость поиска потерянной линии

# --- дальномер ---
OBSTACLE_MM = 120             # ближе этого значения считаем препятствием
RANGE_MAX_MM = 1000           # эффективная дальность по описанию ~1 м

# --- повороты без энкодеров: по времени (подбирать!) ---
TURN_90_TIME_S = 0.45
TURN_180_TIME_S = 0.90
JUNCTION_CREEP_S = 0.12       # проехать вперёд, чтобы центр робота встал на перекрёсток
TURN_SPEED = 0.45


# =====================================================================
# 2. ЖЕЛЕЗО (единственное место, которое надо править под API)
# =====================================================================
class Robot:
    """Обёртка над maixcam_bridge_2. Верните в каждый метод нужные вызовы."""

    def start(self):
        # TODO: подключить библиотеку и включить датчики и моторы.
        # По описанию они по умолчанию ВЫКЛЮЧЕНЫ и запускаются через API.
        #   import maixcam_bridge_2 as bridge
        #   bridge.<включить_датчики>()
        #   bridge.<включить_моторы>()
        raise NotImplementedError("впишите инициализацию maixcam_bridge_2")

    def read_line_raw(self):
        """Вернуть список из 4 сырых значений АЦП (0..4095), слева направо."""
        # TODO: return [s0, s1, s2, s3]
        raise NotImplementedError

    def read_distance_mm(self):
        """Расстояние до препятствия впереди в мм."""
        # TODO: return distance_mm
        raise NotImplementedError

    def button_pressed(self):
        """True, пока нажата внешняя кнопка."""
        # TODO: return bool
        raise NotImplementedError

    def set_wheels(self, front_left, front_right, rear_left, rear_right):
        """Скорость каждого колеса, -1.0 .. 1.0 (минус = назад)."""
        # TODO: подать значения на 4 мотора. Проверьте знак для каждого
        # колеса отдельно: часто одна сторона стоит зеркально.
        raise NotImplementedError

    def stop(self):
        self.set_wheels(0, 0, 0, 0)

    # удобный интерфейс для остального кода: две стороны
    def drive(self, left, right):
        left = max(-1.0, min(1.0, left))
        right = max(-1.0, min(1.0, right))
        self.set_wheels(left, right, left, right)


# =====================================================================
# 3. МОДЕЛЬ ДЛЯ ПРОВЕРКИ НА КОМПЬЮТЕРЕ
# =====================================================================
class MockRobot(Robot):
    """Линия смещается относительно робота; проверяем, что ПИД её возвращает."""

    def __init__(self):
        self.offset = 2.0        # где линия относительно центра, в единицах датчиков
        self.t = 0
        self.last_cmd = (0, 0)

    def start(self):
        pass

    def read_line_raw(self):
        out = []
        for p in SENSOR_WEIGHTS:
            level = max(0.0, 1.0 - abs(p - self.offset) / 2.0)   # 1 над линией, спадает с расстоянием
            high, low = 3800, 300
            raw = low + (high - low) * level
            out.append(int(raw if LINE_IS_HIGH else high + low - raw))
        return out

    def read_distance_mm(self):
        return 800

    def button_pressed(self):
        self.calls = getattr(self, "calls", 0) + 1
        return self.calls == 2   # «нажали» кнопку на втором опросе

    def set_wheels(self, fl, fr, rl, rr):
        left, right = (fl + rl) / 2, (fr + rr) / 2
        self.offset += 0.15 * (right - left) + 0.01   # руление + небольшой снос
        self.t += 1
        self.last_cmd = (left, right)


# =====================================================================
# 4. ОБРАБОТКА ДАТЧИКОВ И РЕГУЛЯТОР
# =====================================================================
class LineSensors:
    def __init__(self):
        self.mn = [0] * 4
        self.mx = [ADC_MAX] * 4

    def calibrate(self, robot, seconds=3.0):
        """Покрутите робот над линией и фоном вручную (или он крутится сам)."""
        self.mn = [ADC_MAX] * 4
        self.mx = [0] * 4
        end = time.time() + seconds
        while time.time() < end:
            raw = robot.read_line_raw()
            for i, v in enumerate(raw):
                self.mn[i] = min(self.mn[i], v)
                self.mx[i] = max(self.mx[i], v)
            time.sleep(0.01)

    def normalized(self, raw):
        """0..1, где 1 = датчик над линией."""
        out = []
        for i, v in enumerate(raw):
            span = max(1, self.mx[i] - self.mn[i])
            x = (v - self.mn[i]) / span
            x = max(0.0, min(1.0, x))
            out.append(x if LINE_IS_HIGH else 1.0 - x)
        return out

    def error(self, vals):
        """Положение линии в диапазоне -1..1: отрицательное = слева, положительное = справа.
        None, если линия потеряна."""
        s = sum(vals)
        if s < LOST_SUM:
            return None
        return sum(w * v for w, v in zip(SENSOR_WEIGHTS, vals)) / s / max(abs(w) for w in SENSOR_WEIGHTS)

    def is_junction(self, vals):
        return sum(1 for v in vals if v > LINE_THRESHOLD) >= JUNCTION_COUNT


class PID:
    def __init__(self, kp, ki, kd, i_limit=1.0):
        self.kp, self.ki, self.kd, self.i_limit = kp, ki, kd, i_limit
        self.integral = 0.0
        self.prev = 0.0
        self.deriv = 0.0

    def reset(self):
        self.integral = 0.0
        self.prev = 0.0
        self.deriv = 0.0

    def update(self, err, dt):
        self.integral = max(-self.i_limit, min(self.i_limit, self.integral + err * dt))
        raw_d = (err - self.prev) / dt if dt > 0 else 0.0
        self.deriv = 0.7 * self.deriv + 0.3 * raw_d      # сглаживание, иначе D шумит
        deriv = self.deriv
        self.prev = err
        return self.kp * err + self.ki * self.integral + self.kd * deriv


# =====================================================================
# 5. ЛОГИКА НА ПЕРЕКРЁСТКАХ (плацдарм под лабиринт)
# =====================================================================
def decide_turn(robot, vals):
    """Что делать на перекрёстке. Сейчас - правило левой руки без карты.
    Возвращает 'L', 'S', 'R' или 'U' (разворот).
    TODO: когда станет известна задача, замените на свою логику
    (запомнить путь, flood fill, команда с метки и т.д.)."""
    return "L"


def turn(robot, direction):
    """Поворот на месте по времени (подберите константы под ваш пол и батарею)."""
    if direction == "S":
        return
    sign = {"L": -1, "R": 1, "U": 1}[direction]
    t = TURN_180_TIME_S if direction == "U" else TURN_90_TIME_S
    robot.drive(sign * TURN_SPEED, -sign * TURN_SPEED)
    time.sleep(t)
    robot.stop()


# =====================================================================
# 6. ГЛАВНЫЙ ЦИКЛ
# =====================================================================
def run(robot, max_steps=None):
    sensors = LineSensors()
    pid = PID(KP, KI, KD)
    running = False
    btn_was_down = False
    last_err = 0.0
    last_time = time.time()
    steps = 0

    robot.start()
    # калибровку лучше делать вручную перед стартом; в симуляции пропускаем
    if not isinstance(robot, MockRobot):
        sensors.calibrate(robot)

    while True:
        steps += 1
        if max_steps and steps > max_steps:
            robot.stop()
            return

        now = time.time()
        dt = LOOP_PERIOD_S if isinstance(robot, MockRobot) else max(1e-3, now - last_time)
        last_time = now

        # --- кнопка: переключает старт/стоп по нажатию ---
        down = robot.button_pressed()
        if down and not btn_was_down:
            running = not running
            pid.reset()
            if not running:
                robot.stop()
        btn_was_down = down

        if not running:
            time.sleep(LOOP_PERIOD_S)
            continue

        # --- препятствие по дальномеру ---
        dist = robot.read_distance_mm()
        if 0 < dist < OBSTACLE_MM:
            robot.stop()
            # TODO: объезд / разворот / ожидание, зависит от правил
            time.sleep(LOOP_PERIOD_S)
            continue

        # --- линия ---
        vals = sensors.normalized(robot.read_line_raw())

        if sensors.is_junction(vals):
            robot.drive(BASE_SPEED, BASE_SPEED)
            time.sleep(JUNCTION_CREEP_S)
            robot.stop()
            turn(robot, decide_turn(robot, vals))
            pid.reset()
            continue

        err = sensors.error(vals)
        if err is None:
            # линия потеряна: крутимся в сторону, где видели её последний раз
            d = 1 if last_err >= 0 else -1
            robot.drive(d * SEARCH_SPEED, -d * SEARCH_SPEED)
        else:
            last_err = err
            out = pid.update(err, dt)
            # положительная ошибка = линия справа -> поворот вправо
            left = BASE_SPEED + out
            right = BASE_SPEED - out
            left = max(-MAX_SPEED, min(MAX_SPEED, left))
            right = max(-MAX_SPEED, min(MAX_SPEED, right))
            robot.drive(left, right)

        if isinstance(robot, MockRobot):
            print(f"step {steps:3d}  offset={robot.offset:+.2f}  "
                  f"cmd=({robot.last_cmd[0]:+.2f},{robot.last_cmd[1]:+.2f})")
        else:
            time.sleep(LOOP_PERIOD_S)


if __name__ == "__main__":
    if "--sim" in sys.argv:
        run(MockRobot(), max_steps=60)
    else:
        run(Robot())
