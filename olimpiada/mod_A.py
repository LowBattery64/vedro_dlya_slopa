import time, itertools
import numpy as np
import cv2
from quasar_client import QuasarClient, QuasarCommandError, QuasarTimeoutError, QuasarRunStoppedError

client = QuasarClient.from_environment(timeout=5)
client.wait_for_simulation_start()
client.start_attempt()
T0 = time.monotonic()
def ts(): return round(time.monotonic() - T0, 1)

def log(level, msg, **d):
    print(f'[{ts()}] {level}: {msg}', d if d else '')
    try: client.append_log(level, 'MOD_A', msg[:300], d or None)
    except Exception: pass

# ---------------- поезд: положение по цветовым меткам (фильтр по индексу блока и полярности) ----------------
def _gen(n=686, w=6):
    s, used, nxt = [], set(), [0]
    while len(s) < n:
        k, c, placed = len(s), nxt[-1], False
        while c < 4:
            if not s or c != s[-1]:
                if k >= w - 1:
                    win = tuple(s[k-w+1:]) + (c,)
                    if win in used: c += 1; continue
                    used.add(win)
                s.append(c); nxt[-1] = c + 1; nxt.append(0); placed = True; break
            c += 1
        if not placed:
            nxt.pop(); last = s.pop(); k = len(s)
            if k >= w - 1: used.discard(tuple(s[k-w+1:]) + (last,))
    return s
SEQ = np.array(_gen()); N = len(SEQ)
G = (-1, 1)          # полярность 0: S>0 уменьшает индекс; 1: увеличивает (старт поезда случайный)

def classify(rgb):
    ch = [rgb['r'], rgb['g'], rgb['b']]; v = max(ch)
    if v < 45: return 0
    if sorted(ch)[1] > 0.6 * v: return None
    return 1 + ch.index(v)

def _shift(a, k):
    out = np.zeros_like(a)
    if k == 0: return a.copy()
    if k > 0: out[k:] = a[:N-k]; out[N-1] += a[N-k:].sum()
    else: out[:N+k] = a[-k:]; out[0] += a[:-k].sum()
    return out

class Loc:
    def __init__(self):
        self.b = np.full((2, N), 0.5 / N); self.t = None; self.v = 0.0
    def predict(self, now):
        if self.t is not None and self.v != 0.0:
            for h in (0, 1):
                d = G[h] * self.v * (now - self.t) / 0.01
                n = int(np.floor(d)); f = d - n
                self.b[h] = (1 - f) * _shift(self.b[h], n) + f * _shift(self.b[h], n + 1)
        for h in (0, 1):
            x = self.b[h]; nb = 0.76 * x
            nb[1:] += 0.12 * x[:-1]; nb[:-1] += 0.12 * x[1:]; nb[0] += 0.12 * x[0]; nb[-1] += 0.12 * x[-1]
            self.b[h] = nb
        self.t = now
    def update(self, obs):
        if obs is None: return
        nb = self.b * np.where(SEQ == obs, 0.8, 0.05)[None, :]; s = nb.sum()
        self.b = nb / s if s > 1e-300 else np.full((2, N), 0.5 / N)
    def est(self):
        m = self.b.sum(axis=0); k = int(np.argmax(m))
        s0, s1 = self.b[0].sum(), self.b[1].sum()
        return k, float(m[max(0, k-2):k+3].sum()), (0 if s0 >= s1 else 1), float(max(s0, s1))

class Train:
    """Неблокирующий контроллер: step(rgb, now) вызывается на каждом опросе датчика."""
    def __init__(self, send, v=0.05, vf=0.02, fine=8):
        self.send, self.v, self.vf, self.fine = send, v, vf, fine
        self.f = Loc(); self.cmd = 0.0; self.mode = 'idle'; self.est = None; self.target = None
        self.t_mode = 0.0; self.blind = 0.0
    def _set(self, s, now):
        self.f.predict(now); self.f.v = s; self.cmd = s; self.send(s)
    def start(self, target, now):
        self.target = target; self.mode = 'run'; self.f.t = now; self.blind = now; self._set(self.v, now)
    def step(self, rgb, now):
        f = self.f; f.predict(now); f.update(classify(rgb)); k, c, h, ph = f.est(); self.est = k
        if self.mode in ('idle', 'done'): return
        loc = c >= 0.8 and ph >= 0.95
        if self.mode == 'settle':
            if now - self.t_mode >= 0.8: self.mode = 'run' if (loc and k != self.target) else 'done'
            return
        if self.mode == 'rev':
            if now - self.t_mode < 0.3: return
            self.mode = 'run'
        if loc:
            self.blind = now; err = self.target - k
            if err == 0:
                self._set(0.0, now); self.mode = 'settle'; self.t_mode = now; return
            need = 1 if err > 0 else -1
            want = need * G[h] * (self.v if abs(err) > self.fine else self.vf)
            if want != self.cmd:
                if self.cmd != 0 and (want > 0) != (self.cmd > 0):
                    self._set(0.0, now); self.mode = 'rev'; self.t_mode = now; return
                self._set(want, now)
        else:
            if self.cmd != 0 and now - self.blind > 8: self._set(0.0, now); self.blind = now
            elif self.cmd == 0 and now - self.blind > 1.0: self._set(self.v, now)

def send_speed(s):
    client.execute_gcode('train.1', 'M121' if s == 0 else f'M120 S{s:.2f}', wait=False)

TRAIN_HOME = 336                    # блок маршрута: центр вертикального участка (зона разгрузки)
tr = Train(send_speed)

LAST = {}
def pump():
    """Шаг опроса: цвет под составом -> контроллер поезда; возвращает состояние."""
    client.execute_gcode('train.1', 'M150', wait=False)
    st = client.get_state(); LAST['st'] = st
    if tr.mode == 'done' and not LAST.get('tdone'): LAST['tdone'] = True; log('info', 'поезд в домашнем положении (датчик)', block=tr.est, t=ts())
    try: tr.step(st['train']['gcode']['lastResponse']['rgb'], time.monotonic())
    except (KeyError, TypeError): pass
    return st

def nap(sec):
    t = time.monotonic()
    while time.monotonic() - t < sec: pump(); time.sleep(0.03)

# ---------------- кран ----------------
MVXY, FXY = 'G1', 9000          # режим и подача свободных перемещений по XY (подбираются автотюнингом в простое)
MVZ, FZU, FZD = 'G1', 7200, 7200   # подъём (свободный) и опускание (к контакту), мм/мин
ZT, ZG, ZB = 80, 180, 170        # высота перемещения, захват/посадка на вагон, сброс в буфер
BUF_X = 540                     # буфер: свободное место зоны разгрузки, между кораблём и путями
BUF_Y = (130, 250, 370, 60, 440)
TIMES = {}

def crane(prog, tmo=40, tag=''):
    client.execute_gcode('crane.1', prog, wait=False)
    t0 = time.monotonic()
    while True:
        st = pump(); g = st['crane']['gcode']
        if g.get('lastError'): raise QuasarCommandError(str(g['lastError']))
        if g.get('busy') is False:
            TIMES.setdefault(tag, []).append(time.monotonic() - t0); return st
        if time.monotonic() - t0 > tmo: raise QuasarTimeoutError('crane timeout')
        time.sleep(0.03)

def clamp(X, Y): return min(max(X, 0), 1106), min(max(Y, 0), 500)
def crane_xy(st): return st['crane']['portal'] * 1000, st['crane']['cart'] * 1000
def mxy(X, Y, Z=None):
    X, Y = clamp(X, Y)
    return f'{MVXY} X{X:.0f} Y{Y:.0f}' + ('' if Z is None else f' Z{Z}') + f' F{FXY}'
def zup(Z): return f'{MVZ} Z{Z} F{FZU}'
def zdn(Z): return f'G1 Z{Z} F{FZD}'

def autotune():
    """Пока корабль подходит: замер скоростей G1/G0 и подачи. Выбираем быстрее, только если выигрыш заметный."""
    global MVXY, FXY, MVZ, FZU
    res = {}
    def ready(): return (LAST.get('st', {}).get('moduleA') or {}).get('shipPhase') == 'ready'
    def run(tag, line, dist, check):
        if ready(): return
        try:
            t = time.monotonic(); st = crane(line, tag='tune'); dt = max(time.monotonic() - t, 0.05)
            ok = abs(check(st)) < 2.0
            res[tag] = (round(dist / dt), ok)
        except (QuasarCommandError, QuasarTimeoutError) as e:
            res[tag] = (0, False); log('warning', f'автотюнинг {tag}: {e}')
    cy = lambda y: (lambda st: st['crane']['cart'] * 1000 - y)
    cz = lambda z: (lambda st: st['crane']['zRod'] * 1000 - z)
    run('G1_9000', 'G1 X933 Y400 F9000', 200, cy(400))
    run('G0_9000', 'G0 X933 Y200 F9000', 200, cy(200))
    run('G0_18000', 'G0 X933 Y400 F18000', 200, cy(400))
    run('G1_18000', 'G1 X933 Y200 F18000', 200, cy(200))
    run('Z_G1_7200', 'G1 Z80 F7200', 80, cz(80))
    run('Z_G0_18000', 'G0 Z0 F18000', 80, cz(0))
    base = res.get('G1_9000', (0, False))[0] or 1
    best = max(((v, k) for k, (v, ok) in res.items() if ok and not k.startswith('Z')), default=(0, ''))
    if best[0] > 1.15 * base:
        MVXY, FXY = best[1].split('_')[0], int(best[1].split('_')[1])
    zb = res.get('Z_G1_7200', (0, False))[0] or 1
    if res.get('Z_G0_18000', (0, False))[1] and res['Z_G0_18000'][0] > 1.15 * zb: MVZ, FZU = 'G0', 18000
    log('info', f'автотюнинг мм/с {res} -> XY {MVXY} F{FXY}, Z-подъём {MVZ} F{FZU}'[:290])

ARUCO = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
IDS = {30, 32, 33, 34, 35, 36}
PX = 1.5                        # пикселей на мм при поднятом захвате (Z=0) на уровне корабля
def detect_markers():
    frame = client.get_crane_camera_frame()
    cs, ids, _ = cv2.aruco.detectMarkers(frame, ARUCO)
    out = {}
    if ids is not None:
        for cc, i in zip(cs, ids.ravel()):
            if int(i) in IDS: out[int(i)] = (float(cc[0][:, 0].mean()), float(cc[0][:, 1].mean()))
    return out

def scan_at(Y, X=933):
    """Камера на Z=0 над точкой (X, Y): {id: (X, Y)} в координатах крана."""
    st = crane(mxy(X, Y, 0), tag='scan')
    Xc, Yc = crane_xy(st)
    return {i: (Xc + (320 - cx) / PX, Yc + (240 - cy) / PX) for i, (cx, cy) in detect_markers().items()}

BIAS = [-5.0, 0.0]              # поправка захвата: магнит смещён от оси камеры (измерено: X-5 мм); уточняется по ходу
def try_grab(X, Y):
    X, Y = clamp(X, Y)
    try:
        crane([mxy(X, Y, ZT), zdn(ZG), 'M106 P1 S255'], tag='grab')
        return True
    except (QuasarCommandError, QuasarTimeoutError) as e:
        st = client.get_state()
        log('warning', f'захват не удался ({X:.0f},{Y:.0f}): {e}', cx=round(st['crane']['portal'] * 1000),
            cy=round(st['crane']['cart'] * 1000), cz=round(st['crane']['zRod'] * 1000))
        try: crane(['M106 P1 S0', zup(ZT)], tag='grab')
        except Exception: pass
        return False

def grab(i, pos):
    X, Y = pos[i]
    if try_grab(X + BIAS[0], Y + BIAS[1]): return True
    for dx, dy in ((-4, 0), (4, 0), (0, 5), (0, -5), (-8, 0), (8, 0)):
        if try_grab(X + BIAS[0] + dx, Y + BIAS[1] + dy):
            BIAS[0] += dx; BIAS[1] += dy; log('info', f'захват {i} со смещением', bx=BIAS[0], by=BIAS[1]); return True
    return False

def to_buffer(k):
    crane([zup(ZT), mxy(BUF_X, BUF_Y[k]), zdn(ZB), 'M106 P1 S0', 'G4 P150', zup(ZT)], tag='buf')

WAGON_X, BAY0_Y, BAY_PITCH = 110, 240, 91     # окно вагона: X 95..125, Y +-3 при поезде на блоке 336; отсеки Y~149/240/331
OFFS = (0, 3, -3, 6, -6, 9, -9, 12, -12, 16, -16, 20, -20)
def to_wagon(bay_y):
    """Опускаем в карман; блокировка = мимо кармана. Возвращает фактический Y или None."""
    for d in OFFS:
        y = bay_y + d
        try:
            crane([zup(ZT), mxy(WAGON_X, y), zdn(ZG)], tag='wag')
        except QuasarCommandError as e:
            if 'blocked' not in str(e): raise
            continue
        before = client.get_state()['containers']['onTrain']
        crane(['M106 P1 S0', 'G4 P200', zup(ZT)], tag='wag')
        after = client.get_state()['containers']['onTrain']
        log('info', f'контейнер на вагон: Y={y}', onTrain=after, was=before)
        return y
    crane(zup(ZT), tag='wag')
    return None

def agg(items):
    out = {}
    for it in items: out[it['nomenclature']] = out.get(it['nomenclature'], 0) + it['massKg']
    return out

# ---------------- основной сценарий ----------------
TLIM = 226                      # после этого времени новые операции не начинаем
completed = False
try:
    log('info', 'старт попытки модуля А')
    tr.start(TRAIN_HOME, time.monotonic())                 # поезд начинает ехать в домашнее положение сразу
    crane(['G90', mxy(933, 200, 0)], tag='pre')           # камера над колонкой, пока корабль подходит
    autotune()                                             # в простое замеряем скорости (если корабль ещё не готов)
    while True:                                            # ждём сигнал готовности корабля
        st = pump()
        if (st.get('moduleA') or {}).get('shipPhase') == 'ready': break
        time.sleep(0.03)
    log('info', 'корабль готов к разгрузке', t=ts())

    pos = {}
    for Y in (200, 420, 120, 480):                          # сканируем колонку, пока не найдём все шесть маркеров
        for i, p in scan_at(Y).items(): pos.setdefault(i, p)
        if len(pos) == 6: break
    log('info', 'маркеры: ' + ' '.join(f'{i}:({p[0]:.0f},{p[1]:.0f})' for i, p in sorted(pos.items())))
    if len(pos) < 6: raise RuntimeError('не найдены все шесть маркеров')

    request = agg(client.get_cargo_request()['items'])
    info = {}
    for i in sorted(pos):                                   # состав по маркеру из БД -> регистрация
        info[i] = client.get_container_cargo(i)['items']
        resp = client.register_container_cargo(i, info[i])
        if resp.get('matchesExpected') is not True: log('error', f'сверка контейнера {i} не прошла: {resp}'[:290])
    need = next((c for c in itertools.combinations(sorted(pos), 3)
                 if agg([it for i in c for it in info[i]]) == request), None)
    if need is None: raise RuntimeError('нет тройки контейнеров под заявку')
    rest = [i for i in sorted(pos) if i not in need]
    log('info', f'зарегистрированы все 6; требуются {list(need)}, в буфер {rest}', t=ts())

    done, nbuf, ys = set(), [0], []
    def buffer_pass():
        for i in sorted([j for j in rest if j not in done], key=lambda j: pos[j][1]):
            if time.monotonic() - T0 > TLIM: return
            if grab(i, pos):
                to_buffer(min(nbuf[0], 4)); nbuf[0] += 1; done.add(i)
    def wagon_pass():
        for k, i in enumerate(need):
            if i in done or time.monotonic() - T0 > TLIM: continue
            if k == 0 or not ys: guess = BAY0_Y
            elif k == 1: guess = ys[0] + BAY_PITCH
            else: guess = ys[0] - (ys[1] - ys[0] if len(ys) > 1 else BAY_PITCH)   # шаг отсеков из первых двух
            if not grab(i, pos): continue
            y = to_wagon(guess)
            if y is None:
                log('error', f'контейнер {i} не встал в карман, сбрасываем в буфер')
                to_buffer(min(nbuf[0], 4)); nbuf[0] += 1; continue
            done.add(i); ys.append(y)
    buffer_pass()                                           # лишние в буфер, пока поезд подъезжает
    while tr.mode != 'done' and time.monotonic() - T0 < TLIM:        # дожидаемся поезда
        pump(); time.sleep(0.03)
    wagon_pass()
    for _ in range(2):                                      # повторный проход по тем, что не удалось
        if len(done) == 6 or time.monotonic() - T0 > TLIM: break
        log('warning', f'повторный проход, осталось {sorted(set(pos) - done)}')
        buffer_pass(); wagon_pass()

    crane([zup(0), mxy(0, 0, 0)], tag='home')               # домашнее положение АРПУ: правый нижний угол, захват поднят
    cs = client.get_state()['containers']
    if cs['onTrain'] == 3 and cs['onShip'] == 0:
        client.complete_unloading(); completed = True
        log('info', 'разгрузка завершена, сигнал Диспетчеру отправлен', t=ts())
    else:
        log('error', 'разгрузка не завершена, сигнал не отправляем', onTrain=cs['onTrain'], onShip=cs['onShip'], free=cs['free'])
    log('info', 'время по этапам, с/число: ' + ' '.join(f'{k}:{sum(v):.0f}/{len(v)}' for k, v in TIMES.items()))
    t = time.monotonic()
    while time.monotonic() - t < 40:
        if client.get_state()['attemptState'] in ('success', 'failure', 'stopped'): break
        time.sleep(0.3)
    log('info', 'итог попытки: ' + str(client.get_state()['attemptState']), t=ts())
except QuasarRunStoppedError:
    log('info' if completed else 'error', 'прогон остановлен хостом' + (': попытка завершена после сигнала Диспетчеру' if completed else ' до завершения разгрузки'))
except Exception as e:
    log('error', f'сбой сценария: {e!r}')
    raise
finally:
    try: send_speed(0)
    except Exception: pass
