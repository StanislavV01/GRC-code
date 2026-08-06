# Стендовый тест: Link-Loss Backtrack (compass-free, IMU-only)

Инструкция по запуску и проверке ядра системы автономного возврата НРК на
стенде. Версия — **программный стенд**: всё ядро (оценка позиции + контроллер
возврата) гоняется на симуляции сенсоров, без реального железа. Это этап 1 ТЗ
(PoC / SITL-уровень) — проверяем алгоритм до выезда в поле.

> Реальные драйверы MAVLink/CAN и SITL — следующий этап (см. раздел 11).

---

## 1. Что проверяем на стенде

- Калибровка нуля гироскопа на старте (нет компаса → курс только с гиро).
- Запись трека (breadcrumbs) в GNSS-denied режиме.
- Срабатывание failsafe при потере связи.
- Автономный возврат по своему треку (forward и reverse-only).
- Метрика возврата ≤ 15 м / ~3 % (критерий приёмки ТЗ §7).
- Влияние датчика скорости (Hall vs энкодер AS5047P) на одометрию.
- Лог и blackbox для диагностики.

---

## 2. Что нужно для теста

**Минимум (программный стенд — достаточно для этой инструкции):**
- Любая Linux/macOS машина **или** Raspberry Pi CM4.
- Компилятор C++17: `g++` или `clang++`.
- (опц.) `cmake` ≥ 3.16. Без него работает `./build.sh`.
- (опц.) браузер или `qlmanage` (macOS) — посмотреть `route.svg`.

**Зависимостей нет** — только стандартная библиотека C++. Ни ROS2, ни Eigen,
ни сети. Это сознательно (требование ТЗ: всё on-board, без интернета).

---

## 3. Какие файлы нужны

Весь каталог `backtrack_cpp/`. Минимально необходимое для сборки и теста:

```
backtrack_cpp/
├── build.sh                     # сборка без cmake
├── CMakeLists.txt               # сборка через cmake (+ ctest)
├── include/backtrack/*.hpp      # заголовки (API ядра)        — НУЖНЫ
├── src/*.cpp                    # реализация ядра             — НУЖНЫ
├── apps/run_demo.cpp            # точка входа демо            — НУЖНА
├── tests/*                      # юнит/интеграционные тесты   — для проверки
└── deploy/                      # деплой на Pi (опц.)
    ├── deploy.sh
    └── backtrack.service
```

Артефакты сборки/прогона (`build/`, `data/`) создаются сами и в репозиторий не
коммитятся (см. `.gitignore`).

---

## 4. Установка зависимостей

```bash
# Raspberry Pi OS / Debian / Ubuntu:
sudo apt update && sudo apt install -y g++          # минимум
sudo apt install -y build-essential cmake           # полный набор

# macOS: компилятор идёт с Xcode Command Line Tools
xcode-select --install
```

---

## 5. Сборка

Вариант А — без cmake (быстрее всего):
```bash
cd backtrack_cpp
./build.sh
# => build/run_demo  и  build/unit_tests
```

Вариант Б — cmake + ctest:
```bash
cd backtrack_cpp
cmake -S . -B build && cmake --build build
```

Ожидаемо: сборка **без ошибок и предупреждений**.

---

## 6. Прогон тестов (обязательный шаг)

```bash
./build/unit_tests           # сборка через build.sh
# или
ctest --test-dir build --output-on-failure   # сборка через cmake
```

**Ожидаемый результат:** `30 passed, 0 failed, 30 total`.
Любой `FAIL` — стоп, без зелёных тестов в поле не выезжаем.

---

## 7. Прогон сценариев (демо)

Все команды пишут результаты в `data/` (CSV + SVG) и лог в stderr.

| # | Команда | Что показывает | Ожидаемая ошибка возврата (seed 7) |
|---|---------|----------------|-----------------|
| 1 | `./build/run_demo` | базовый: гиро + калибровка + ZUPT + энкодер | **~6 м** ✓ |
| 2 | `./build/run_demo --reverse` | откат задним ходом без разворота | **~4 м** ✓ |
| 3 | `./build/run_demo --speed-sensor hall` | только Hall — грубая одометрия | возврат ~5 м, но дист. ошибка ~65 м |
| 4 | `./build/run_demo --source odometry` | курс по гусеницам (плохо, проскальзывание) | ~66 м ✗ |
| 5 | `./build/run_demo --no-calib` | без калибровки гиро (видно дрейф) | ~150 м ✗ |

Сценарии 4–5 — **намеренно деградированные**, чтобы видеть, что именно держит
точность. На приёмке важны сценарии 1 и 2.

Прогон с логом в файл:
```bash
./build/run_demo --log-level info --log-file data/backtrack.log
```

---

## 8. Критерии успеха (стендовая приёмка)

Сценарий 1 (базовый), seed 7, должен дать примерно:

| Метрика | Норма | Смысл |
|---|---|---|
| `backtrack completed` | `true` | дошёл до дома |
| `final return error` | **≤ 15 м** (факт ~6 м) | критерий ТЗ §7 |
| `heading drift @ end` | ~1–6° | дрейф курса без компаса |
| `mean backtrack ERPM` | ~1200 | у порога бессенсорного FOC (нужен sensored) |
| тесты | 30/30 | регрессия |

Прогнать на нескольких seed (`--seed 11 22 99`) — разброс ошибки возврата
должен оставаться ≤ 15 м (компас-фри даёт вариативность, это нормально).

---

## 9. Где смотреть результаты

| Файл | Что внутри |
|---|---|
| `data/session_timeseries.csv` | blackbox 50 Гц: сенсоры, оценка, ground truth (FR-7) |
| `data/breadcrumbs.csv` | записанный трек (x, y, heading, дистанция) |
| `data/route.svg` | график: истинный путь vs оценка vs дом/финиш |
| `data/backtrack.log` | событийный лог (если задан `--log-file`) |

Посмотреть график:
```bash
# macOS:
qlmanage -t -s 1000 -o . data/route.svg && open data/route.svg.png
# Linux: открыть data/route.svg в браузере
```

---

## 10. Интерпретация логов и метрик

Нормальный жизненный цикл в логе:
```
INFO  CALIBRATING: holding still 15 s for gyro-bias estimation
INFO  calibration done: gyro bias est 0.0038 rad/s (true 0.004)
INFO  RECORDING: outbound 300 m
WARN  LINK LOST -> FAILSAFE_BACKTRACK; trail 577 crumbs, est end (...)
INFO  ARRIVED: final return error 6.07 m (2.0%)
```

Тревожные сообщения:
- `ERROR backtrack DID NOT COMPLETE ...` — контроллер не дошёл (увеличить
  `max_backtrack_steps` или искать застревание в логике pure-pursuit).
- `WARN mean backtrack ERPM ... below sensorless floor` — едем слишком медленно
  для бессенсорного FOC: нужен sensored (Hall/энкодер).

CSV-колонки для разбора дрейфа: `est_x/y/heading` (оценка) против `gt_x/y/heading`
(истина), `gyro_bias_est`, `stationary` (когда сработал ZUPT).

---

## 11. Troubleshooting

| Симптом | Причина / решение |
|---|---|
| `no such file or directory: ./build.sh` | вы не в каталоге `backtrack_cpp` — `cd backtrack_cpp` |
| ошибки линковки про `pthread` | старый компилятор; `build.sh` уже добавляет `-pthread` — обновите g++ |
| `error: cannot open for writing: data/...` | нет каталога вывода; `run_demo` создаёт `data/` сам, проверьте права |
| тесты падают на `returns_home` | алгоритм/параметры изменены — смотрите diff, не правьте тест под зелёный |
| на Pi не собирается | `sudo apt install -y g++`; проверьте версию: нужен C++17 |
| лог пустой | логгер opt-in: задайте `--log-level info` (в тестах он намеренно молчит) |

---

## 12. Чек-лист стендовой приёмки

- [ ] Сборка без предупреждений (`./build.sh`).
- [ ] `30/30` тестов зелёные.
- [ ] Сценарий 1: `final return error ≤ 15 м`, `backtrack completed = true`.
- [ ] Сценарий 2 (reverse): возврат ≤ 15 м, движение задним ходом.
- [ ] Разброс по seed 7/11/22/99 ≤ 15 м.
- [ ] `data/route.svg`: зелёные пути «туда/обратно» совпадают, финиш у дома.
- [ ] Лог содержит CALIBRATING → RECORDING → LINK LOST → ARRIVED.
- [ ] Сценарий `--speed-sensor hall` показывает рост ошибки одометрии (обоснование энкодера).

---

## 13. Чего на стенде ПОКА нет (следующие этапы)

- **Реальные драйверы** MAVLink (IMU с Pixhawk) и CAN/SocketCAN (ERPM/ток с VESC)
  — сейчас сенсоры синтетические. Это этап интеграции на CM4.
- **ArduPilot SITL + Gazebo** — прогон failsafe на «настоящем» протоколе без
  железа (этап 1 ТЗ).
- **Долгоживущий демон** — сейчас `run_demo` одноразовый; для systemd-сервиса
  нужен главный цикл (см. `deploy/backtrack.service`).
- **Калибровочные цифры платформы:** внешняя передача мотор→звезда, радиус
  звезды, наличие энкодера AS5047P (см. `docs/REFERENCE.md`, раздел «Параметры»).

См. также `README.md` (обзор) и `docs/REFERENCE.md` (техсправочник).
