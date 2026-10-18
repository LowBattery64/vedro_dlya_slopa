# -*- coding: utf-8 -*-
# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║  main_example.py — готовый каркас соревновательной программы                 ║
# ║                                                                              ║
# ║  Логика:                                                                     ║
# ║    IDLE ──(кнопка)──> RUNNING ──(препятствие/финиш)──> FINISHED ──(кнопка)   ║
# ║                                                                              ║
# ║  Переименуйте этот файл в main.py, если упаковываете проект-папку:           ║
# ║  в папке должны лежать  main.py  и  contest_kit.py                           ║
# ╚══════════════════════════════════════════════════════════════════════════════╝

from maix import app, time

import contest_kit as ck

# ─────────────────────────── НАСТРОЙКИ ЗАЕЗДА ──────────────────────────────────
BASE_SPEED   = 40        # базовая скорость моторов (единицы — см. мост)
OBSTACLE_MM  = 150       # дальномер ближе этого порога = стоп/финиш
LOOP_MS      = 20        # период цикла управления (~50 Гц)
USE_CAMERA   = True      # включить ли камеру (напр. для поиска меток)

# ─────────────────────────── ИНИЦИАЛИЗАЦИЯ ─────────────────────────────────────
hal = ck.RobotHAL()          # низ: датчики + моторы (есть режим заглушки)
hal.enable()                 # ОБЯЗАТЕЛЬНО: после включения всё выключено

pid = ck.LinePID(kp=20.0, ki=0.0, kd=5.0, lost_threshold=300)

vis = ck.Vision(320, 240, skip_frames=10) if USE_CAMERA else None

# ─────────────────────────── ОСНОВНОЙ ЦИКЛ ─────────────────────────────────────
state = "IDLE"
t_prev = time.ticks_ms()

print("Готов. Нажмите кнопку для старта.")

while not app.need_exit():
    now = time.ticks_ms()
    dt = time.ticks_diff(now, t_prev) / 1000.0
    t_prev = now

    if state == "IDLE":
        # Ждём нажатия внешней кнопки
        if hal.button_pressed():
            pid.reset()
            state = "RUNNING"
            print("СТАРТ")

    elif state == "RUNNING":
        # 1) Стоп по дальномеру — ПРАВИЛО С ВЫСШИМ ПРИОРИТЕТОМ
        if hal.distance_mm() < OBSTACLE_MM:
            hal.stop()
            state = "FINISHED"
            print("ФИНИШ/препятствие")
            continue

        # 2) Ошибка положения по 4 аналоговым датчикам линии
        err = pid.error_from_sensors(hal.line_raw())
        if err is None:
            # Линия потеряна. Варианты: стоп, разворот на месте,
            # либо продолжить движение с последним известным управлением.
            hal.stop()
        else:
            u = pid.update(err, dt)
            hal.set_motors(BASE_SPEED - u, BASE_SPEED + u)

        # 3) Параллельная работа камеры (пример: ищем красную метку)
        if vis is not None:
            img = vis.read()
            blob = vis.find_color("red", img=img)
            if blob:
                vis.draw_blob(blob, img)
            vis.show(img)

    elif state == "FINISHED":
        hal.stop()
        if hal.button_pressed():
            state = "IDLE"
            print("Сброс, ждём кнопку")

    time.sleep_ms(LOOP_MS)

# Выход по внешней кнопке/принудительно — глушим моторы в любом случае
hal.stop()
print("Программа завершена.")
