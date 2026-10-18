# -*- coding: utf-8 -*-
# ╔══════════════════════════════════════════════════════════════════════════════╗
# ║  contest_kit.py — набор для соревнований на роботе MaixCam Lite              ║
# ║                                                                              ║
# ║  Использование в вашем main.py:                                              ║
# ║      import contest_kit as ck                                                ║
# ║      vis = ck.Vision()                    # камера + экран                   ║
# ║      hal = ck.RobotHAL()                  # датчики и моторы                 ║
# ║      pid = ck.LinePID(kp=20, kd=5)        # регулятор                        ║
# ║                                                                              ║
# ║  Демо-режимы (запуск из терминала устройства):                               ║
# ║      python contest_kit.py blobs red      # поиск красных пятен              ║
# ║      python contest_kit.py line           # линия камерой                    ║
# ║      python contest_kit.py yolo           # детекция YOLO                    ║
# ║      python contest_kit.py photo          # сохранить кадр                   ║
# ║      python contest_kit.py sensors        # телеметрия датчиков              ║
# ║                                                                              ║
# ║  Все методы построены по официальной документации MaixPy v4.                 ║
# ║  Имена методов maixcam_bridge_2 сверьте с описанием организаторов —          ║
# ║  они собраны в одном месте, в классе RobotHAL (константы METHOD_*).          ║
# ╚══════════════════════════════════════════════════════════════════════════════╝

import os
import sys

from maix import camera, display, image, nn, app, time


# ══════════════════════════════════════════════════════════════════════════════
#  Цветовые пороги (пространство LAB: [Lmin,Lmax, Amin,Amax, Bmin,Bmax])
#  Три первых — официальные из документации; остальные — стартовые,
#  КАЛИБРУЙТЕ на площадке гистограммой в MaixVision!
# ══════════════════════════════════════════════════════════════════════════════
COLORS = {
    "red":        [0, 80,  40,   80,  10,   80],
    "green":      [0, 80, -120, -10,   0,   30],
    "blue":       [0, 80,  30,  100, -120, -60],
    "black_line": [0, 30, -128, 127, -128,  127],
    "white_line": [70, 100, -128, 127, -128, 127],
}

# Где искать модели нейросетей на устройстве
YOLO_MODELS = [
    "/root/models/yolo11n.mud",
    "/root/models/yolov5s.mud",
    "/root/models/yolov8n.mud",
    "/root/models/yolo26n.mud",
]
CLASSIFIER_MODEL = "/root/models/mobilenetv2.mud"


def clamp(v, lo, hi):
    """Ограничить значение диапазоном [lo, hi]."""
    return max(lo, min(hi, v))


# ══════════════════════════════════════════════════════════════════════════════
#  FpsMeter — меряем реальную скорость обработки (обязательно используйте!)
# ══════════════════════════════════════════════════════════════════════════════
class FpsMeter:
    def __init__(self):
        self._t0 = time.ticks_ms()
        self._frames = 0
        self.fps = 0.0

    def tick(self):
        """Вызывать на каждом кадре; обновляет значение примерно раз в секунду."""
        self._frames += 1
        now = time.ticks_ms()
        elapsed = time.ticks_diff(now, self._t0)
        if elapsed >= 1000:
            self.fps = self._frames * 1000.0 / elapsed
            self._frames = 0
            self._t0 = now
        return self.fps


# ══════════════════════════════════════════════════════════════════════════════
#  Vision — камера + классические алгоритмы + нейросети + фото
# ══════════════════════════════════════════════════════════════════════════════
class Vision:
    def __init__(self, width=320, height=240, fps=None, grayscale=False,
                 use_display=True, skip_frames=10, buff_num=None):
        kwargs = {}
        if fps is not None:
            kwargs["fps"] = fps
        if buff_num is not None:
            kwargs["buff_num"] = buff_num
        fmt = image.Format.FMT_GRAYSCALE if grayscale else image.Format.FMT_RGB888
        self.cam = camera.Camera(width, height, fmt, **kwargs)
        if skip_frames:
            self.cam.skip_frames(skip_frames)   # первые кадры нестабильны

        self.disp = None
        if use_display:
            try:
                self.disp = display.Display()   # на бесскринной версии
            except Exception as e:              # просто шлёт кадр в MaixVision
                print("[Vision] display недоступен:", e)

        self.fps = FpsMeter()
        self._last = None
        self._yolo = None
        self._classifier = None

    # ---- кадр ----
    def read(self):
        """Прочитать кадр из камеры (и запомнить его)."""
        self._last = self.cam.read()
        self.fps.tick()
        return self._last

    @property
    def last(self):
        return self._last

    def show(self, img=None):
        """Показать кадр (в предпросмотре MaixVision или на экране)."""
        if img is None:
            img = self._last
        if self.disp is not None and img is not None:
            self.disp.show(img)

    @property
    def width(self):
        return self.cam.width()

    @property
    def height(self):
        return self.cam.height()

    # ---- фото ----
    def snapshot(self, folder="/root"):
        """Сохранить текущий кадр в jpg; возвращает путь к файлу."""
        img = self._last if self._last is not None else self.read()
        path = "{}/photo_{}.jpg".format(folder, time.ticks_ms())
        img.save(path)
        print("[Vision] сохранено фото:", path)
        return path

    # ---- классические алгоритмы ----
    def threshold(self, name):
        """Взять порог по имени из таблицы COLORS или список порогов."""
        if isinstance(name, str):
            return COLORS[name]
        return name

    def find_color(self, color="red", img=None, pixels_threshold=300,
                   roi=None, biggest=True):
        """
        Найти цветные пятна.
          color — имя из COLORS либо готовый порог [..6 чисел..],
                  либо список порогов для нескольких цветов сразу.
        Возвращает самый большой blob (или список всех, если biggest=False).
        Blob индексируется: blob[0]=x, blob[1]=y, blob[2]=w, blob[3]=h.
        """
        if img is None:
            img = self.read()
        th = self.threshold(color)
        if isinstance(th[0], int):            # один порог -> список из одного
            th = [th]
        kwargs = {"pixels_threshold": pixels_threshold}
        if roi is not None:
            kwargs["roi"] = roi
        blobs = img.find_blobs(th, **kwargs)
        if biggest:
            if not blobs:
                return None
            return max(blobs, key=lambda b: b[2] * b[3])
        return blobs

    @staticmethod
    def blob_center(blob):
        """Центр пятна (в пикселях кадра)."""
        return (blob[0] + blob[2] // 2, blob[1] + blob[3] // 2)

    def color_error_x(self, blob, img=None):
        """
        Горизонтальная ошибка положения пятна: -1..+1
        (-1 = объект у левого края, +1 = у правого, 0 = по центру).
        Удобно подавать напрямую в регулятор наведения.
        """
        if blob is None:
            return None
        w = img.width() if img is not None else self.width
        cx = blob[0] + blob[2] / 2.0
        return (cx - w / 2.0) / (w / 2.0)

    def find_line(self, color="green", img=None, area_threshold=100, roi=None):
        """
        Найти линию по цвету (метод get_regression).
        Возвращает dict {x1,y1,x2,y2,theta,rho} или None.
        theta приведён к виду: 0 = линия перпендикулярна направлению движения.
        Для Ч/Б линии быстрее работать с серым кадром (см. Vision(..., grayscale=True)).
        """
        if img is None:
            img = self.read()
        th = self.threshold(color)
        if isinstance(th[0], int):
            th = [th]
        kwargs = {"area_threshold": area_threshold}
        if roi is not None:
            kwargs["roi"] = roi
        lines = img.get_regression(th, **kwargs)
        if not lines:
            return None
        a = lines[0]
        theta = a.theta()
        if theta > 90:
            theta = 270 - theta
        else:
            theta = 90 - theta
        return {
            "x1": a.x1(), "y1": a.y1(),
            "x2": a.x2(), "y2": a.y2(),
            "theta": theta, "rho": a.rho(),
        }

    def draw_line(self, info, img=None, color=None):
        """Нарисовать найденную линию на кадре."""
        if info is None:
            return
        if img is None:
            img = self._last
        if color is None:
            color = image.COLOR_GREEN
        img.draw_line(info["x1"], info["y1"], info["x2"], info["y2"], color, 2)

    def draw_blob(self, blob, img=None, color=None, label=True):
        """Нарисовать рамку и крестик центра найденного пятна."""
        if blob is None:
            return
        if img is None:
            img = self._last
        if color is None:
            color = image.COLOR_RED
        img.draw_rect(blob[0], blob[1], blob[2], blob[3], color)
        cx, cy = self.blob_center(blob)
        img.draw_cross(cx, cy, color)
        if label:
            img.draw_string(blob[0], blob[1], "{}x{}".format(blob[2], blob[3]),
                            color)

    # ---- нейросети ----
    def yolo(self, model=None, dual_buff=True):
        """
        Ленивая инициализация YOLO-детектора.
        Если путь не указан — ищется первый существующий из YOLO_MODELS.
        Тип сети выбирается по имени файла (yolo11/yolov5/yolov8/yolo26).
        """
        if self._yolo is not None and model is None:
            return self._yolo
        if model is None:
            for cand in YOLO_MODELS:
                if os.path.exists(cand):
                    model = cand
                    break
        if model is None or not os.path.exists(model):
            raise FileNotFoundError(
                "Модель YOLO не найдена. Проверьте:  ls /root/models/\n"
                "Скачать: maixhub.com/model/zoo/453 (YOLO11) и положить в /root/models/")

        name = os.path.basename(model).lower()
        if "yolo11" in name:
            self._yolo = nn.YOLO11(model=model, dual_buff=dual_buff)
        elif "yolov8" in name or "yolo8" in name:
            self._yolo = nn.YOLOv8(model=model, dual_buff=dual_buff)
        elif "yolo26" in name:
            self._yolo = nn.YOLO26(model=model, dual_buff=dual_buff)
        else:
            self._yolo = nn.YOLOv5(model=model, dual_buff=dual_buff)
        print("[Vision] YOLO загружена:", model)
        return self._yolo

    def detect(self, img=None, model=None, conf_th=0.5, iou_th=0.45):
        """
        Детекция объектов. Возвращает список объектов; у каждого доступны:
        obj.x, obj.y, obj.w, obj.h, obj.class_id, obj.score.
        Имя класса:  det.labels[obj.class_id]
        """
        det = self.yolo(model)
        if img is None:
            img = self.read()
        return det.detect(img, conf_th=conf_th, iou_th=iou_th)

    def draw_detections(self, objs, img=None, color=None):
        """Нарисовать рамки и подписи результатов YOLO."""
        if img is None:
            img = self._last
        if color is None:
            color = image.COLOR_RED
        det = self._yolo
        for obj in objs:
            img.draw_rect(obj.x, obj.y, obj.w, obj.h, color=color)
            msg = "{}: {:.2f}".format(det.labels[obj.class_id], obj.score)
            img.draw_string(obj.x, obj.y, msg, color=color)

    def classifier(self, model=CLASSIFIER_MODEL, dual_buff=True):
        """Ленивая инициализация классификатора (по умолчанию MobileNetV2)."""
        if self._classifier is None:
            if not os.path.exists(model):
                raise FileNotFoundError(
                    "Модель классификатора не найдена: " + model)
            self._classifier = nn.Classifier(model=model, dual_buff=dual_buff)
            print("[Vision] классификатор загружен:", model)
        return self._classifier

    def classify(self, img=None, model=CLASSIFIER_MODEL, top=1):
        """
        Классификация кадра. Возвращает список [(метка, вероятность), ...],
        отсортированный по убыванию вероятности (число = top).
        """
        clf = self.classifier(model)
        if img is None:
            img = self.read()
        res = clf.classify(img)
        scored = sorted(
            ((clf.labels[idx], prob) for idx, prob in res[0]),
            key=lambda p: p[1], reverse=True)
        return scored[:top]


# ══════════════════════════════════════════════════════════════════════════════
#  RobotHAL — нижний уровень робота через maixcam_bridge_2.
#
#  !!! ИМЕНА МЕТОДОВ БИБЛИОТЕКИ ОРГАНИЗАТОРОВ УТОЧНИТЕ И ВПИШИТЕ СЮДА !!!
#  Сделать это можно командой на роботе:
#      python3 -c "import maixcam_bridge_2 as m; print(dir(m)); help(m)"
# ══════════════════════════════════════════════════════════════════════════════
class RobotHAL:
    # ┌─────────────────────────────────────────────────────────────────────┐
    # │ Правьте только эти строки, когда узнаете реальные имена методов:    │
    # └─────────────────────────────────────────────────────────────────────┘
    METHOD_ENABLE   = "enable"
    METHOD_LINE     = "get_line_sensors"
    METHOD_DISTANCE = "get_distance"
    METHOD_BUTTON   = "get_button"
    METHOD_MOTORS   = "set_motors"

    def __init__(self, mock=None):
        """
        mock=None  — автоопределение: библиотека есть -> реальный режим,
                     нет -> заглушка (можно отлаживать верхний уровень без неё).
        """
        self._bridge = None
        self._mock_warned = False
        if mock is None:
            try:
                import maixcam_bridge_2
                self._bridge = maixcam_bridge_2.Bridge()
                print("[HAL] maixcam_bridge_2 подключён")
            except Exception as e:
                print("[HAL] библиотека не найдена ({}), режим заглушки".format(e))
        elif not mock:
            import maixcam_bridge_2
            self._bridge = maixcam_bridge_2.Bridge()

    def _call(self, method, *args, default=None):
        if self._bridge is not None and hasattr(self._bridge, method):
            return getattr(self._bridge, method)(*args)
        if not self._mock_warned:
            print("[HAL] метод '{}' недоступен — возвращаю заглушку. "
                  "Проверьте имена METHOD_* в contest_kit.py".format(method))
            self._mock_warned = True
        return default

    # ---- обязательный запуск периферии после включения! ----
    def enable(self):
        """Включить датчики и моторы (по умолчанию они выключены)."""
        self._call(self.METHOD_ENABLE)

    # ---- телеметрия ----
    def line_raw(self):
        """4 датчика линии: сырые коды АЦП 0..4095."""
        val = self._call(self.METHOD_LINE, default=[0, 0, 0, 0])
        return list(val) if val is not None else [0, 0, 0, 0]

    def distance_mm(self):
        """Лазерный дальномер, мм (до 1000). Заглушка = далеко."""
        val = self._call(self.METHOD_DISTANCE, default=9999)
        return val if val is not None else 9999

    def button_pressed(self):
        """Внешняя кнопка: нажата ли сейчас."""
        val = self._call(self.METHOD_BUTTON, default=False)
        return bool(val)

    # ---- моторы ----
    def set_motors(self, left, right):
        """
        Уставки моторам (диапазон зависит от библиотеки: проценты -100..100
        или мм/с — уточните у организаторов).
        """
        self._call(self.METHOD_MOTORS, left, right)

    def stop(self):
        self.set_motors(0, 0)


# ══════════════════════════════════════════════════════════════════════════════
#  LinePID — регулятор для езды по линии
# ══════════════════════════════════════════════════════════════════════════════
class LinePID:
    POSITIONS = (-3.0, -1.0, 1.0, 3.0)   # «веса» четырёх датчиков

    def __init__(self, kp=20.0, ki=0.0, kd=5.0,
                 integral_limit=50.0, out_limit=100.0, lost_threshold=300):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.integral_limit = integral_limit
        self.out_limit = out_limit
        self.lost_threshold = lost_threshold   # ниже — линия потеряна
        self._i = 0.0
        self._prev = None

    def reset(self):
        self._i = 0.0
        self._prev = None

    def error_from_sensors(self, values):
        """
        Ошибка положения по 4 датчикам АЦП: -3..+3
        (минус = линия левее центра робота).
        Возвращает None, если линия потеряна.
        """
        den = sum(values)
        if den < self.lost_threshold:
            return None
        num = sum(v * p for v, p in zip(values, self.POSITIONS))
        return num / den

    def update(self, err, dt):
        """Шаг регулятора; возвращает управляющее воздействие."""
        if dt <= 0:
            dt = 0.001
        self._i = clamp(self._i + err * dt, -self.integral_limit,
                        self.integral_limit)
        d = (err - self._prev) / dt if self._prev is not None else 0.0
        self._prev = err
        return clamp(self.kp * err + self.ki * self._i + self.kd * d,
                     -self.out_limit, self.out_limit)


# ══════════════════════════════════════════════════════════════════════════════
#  Демо-режимы (запуск:  python contest_kit.py <demo> [аргумент])
# ══════════════════════════════════════════════════════════════════════════════
def demo_blobs(color="red"):
    vis = Vision(320, 240)
    print("Ищем цвет:", color)
    while not app.need_exit():
        img = vis.read()
        blob = vis.find_color(color, img=img)
        if blob:
            vis.draw_blob(blob, img)
            err = vis.color_error_x(blob, img)
            img.draw_string(4, 4, "err: {:.2f}".format(err), image.COLOR_YELLOW)
        vis.show(img)


def demo_line(color="green"):
    vis = Vision(320, 240)
    print("Ищем линию цвета:", color)
    while not app.need_exit():
        img = vis.read()
        info = vis.find_line(color, img=img)
        vis.draw_line(info, img)
        if info:
            img.draw_string(4, 4, "theta:{:.0f} rho:{:.0f}".format(
                info["theta"], info["rho"]), image.COLOR_YELLOW)
        vis.show(img)


def demo_yolo():
    vis = Vision(320, 224)
    while not app.need_exit():
        img = vis.read()
        objs = vis.detect(img)
        vis.draw_detections(objs, img)
        img.draw_string(4, 4, "FPS: {:.0f}".format(vis.fps.fps),
                        image.COLOR_YELLOW)
        vis.show(img)


def demo_photo():
    vis = Vision(640, 480)
    img = vis.read()
    vis.show(img)
    print("Фото сохранено:", vis.snapshot())


def demo_sensors():
    hal = RobotHAL()
    hal.enable()
    print("Ctrl+C для выхода")
    while True:
        print("line:", hal.line_raw(),
              " dist_mm:", hal.distance_mm(),
              " btn:", hal.button_pressed())
        time.sleep_ms(200)


_USAGE = """
Демо-режимы:
  python contest_kit.py blobs [red|green|blue]
  python contest_kit.py line  [red|green|blue|black_line|white_line]
  python contest_kit.py yolo
  python contest_kit.py photo
  python contest_kit.py sensors
"""


if __name__ == "__main__":
    args = sys.argv[1:]
    if not args:
        print(_USAGE)
    elif args[0] == "blobs":
        demo_blobs(args[1] if len(args) > 1 else "red")
    elif args[0] == "line":
        demo_line(args[1] if len(args) > 1 else "green")
    elif args[0] == "yolo":
        demo_yolo()
    elif args[0] == "photo":
        demo_photo()
    elif args[0] == "sensors":
        demo_sensors()
    else:
        print(_USAGE)
