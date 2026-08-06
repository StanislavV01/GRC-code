# Техсправочник: Link-Loss Backtrack core

Справочник по структуре, API, конфигурации, формату логов и привязке к ТЗ.
Операторская инструкция запуска — в `docs/STAND_TEST.md`.

---

## 1. Манифест файлов

| Файл | Назначение |
|---|---|
| `include/backtrack/geometry.hpp` | углы/векторы: `wrap_angle`, `angle_diff`, `distance`, `clamp` (header-only) |
| `include/backtrack/types.hpp` | `Pose`, `Point`, `Twist`, `WheelSpeeds` |
| `include/backtrack/random.hpp` | `GaussianSource` — сеедируемый источник шума |
| `kinematics.hpp/.cpp` | `DiffDriveModel`: twist↔гусеницы↔VESC ERPM, интегрирование позы |
| `sensors.hpp/.cpp` | `SensorModel`: синтез ERPM/тока/гиро/акселерометра, квантование одометрии |
| `estimator.hpp/.cpp` | `DeadReckoning`: dead reckoning + калибровка bias + ZUPT + breadcrumbs (FR-2/3) |
| `controller.hpp/.cpp` | `BacktrackController`: pure-pursuit возврата, forward/reverse (FR-4) |
| `route.hpp/.cpp` | определение outbound-маршрута 300 м |
| `simulation.hpp/.cpp` | `run_simulation`: цикл ground-truth, калибровка→выезд→потеря связи→возврат |
| `log.hpp/.cpp` | событийный логгер (уровни, timestamp, stderr+файл) |
| `logio.hpp/.cpp` | запись CSV + SVG (FR-7) |
| `apps/run_demo.cpp` | CLI точка входа |
| `tests/*` | zero-dependency харнесс + 30 тестов |
| `deploy/deploy.sh` | сборка/деплой на Pi |
| `deploy/backtrack.service` | systemd unit (watchdog) |

На вычислителе (CM4) в продакшене **без изменений** едут `estimator.*` и
`controller.*`; меняются только «края» — `sensors.*` → реальные MAVLink/CAN.

---

## 2. CLI `run_demo`

```
run_demo [опции]
  --source gyro|odometry         источник курса (по умолч. gyro)
  --no-calib                     выключить калибровку bias гироскопа
  --no-zupt                      выключить ZUPT
  --reverse                      откат задним ходом без разворота (FR-4)
  --speed-sensor hall|encoder|ideal   разрешение датчика скорости (по умолч. encoder)
  --seed N                       сид ГСЧ (по умолч. 7)
  --outdir DIR                   каталог вывода (по умолч. data)
  --log-level debug|info|warn|error   (по умолч. info)
  --log-file PATH                дублировать лог в файл
```

---

## 3. Формат blackbox CSV (`session_timeseries.csv`)

50 Гц, одна строка на тик. Колонки:

| Колонка | Смысл |
|---|---|
| `t` | время, с |
| `mode` | `CALIBRATING` / `RECORDING` / `FAILSAFE_BACKTRACK` / `ARRIVED` |
| `link_ok` | 1 = связь есть, 0 = потеряна |
| `erpm_left`, `erpm_right` | ERPM с VESC (измеренные, с квантованием датчика) |
| `motor_current` | ток мотора, А |
| `gyro_z` | угловая скорость рыскания, рад/с |
| `accel_x/y/z` | удельная сила в корпусной СК, м/с² (z≈g) |
| `gyro_bias_est` | текущая оценка смещения нуля гиро |
| `stationary` | 1 = сработал ZUPT (машина неподвижна) |
| `est_x/y/heading` | оценка позы (dead reckoning) |
| `gt_x/y/heading` | истинная поза (ground truth, для анализа; на железе недоступна) |

`breadcrumbs.csv`: `idx, x, y, heading, cumulative_dist`.

---

## 4. Параметры конфигурации

### DiffDriveModel — реальное железо (QS138 70H V3)
| Параметр | Дефолт | Статус |
|---|---|---|
| `pole_pairs` | **5** | ✓ из datasheet |
| `internal_gear` | **2.35** | ✓ встроенный редуктор |
| `external_gear` | 1.0 | **TBD — замерить** (передача мотор→звезда) |
| `wheel_radius` | 0.12 м | **TBD — замерить** (радиус звезды) |
| `track_width` | 0.5 м | **TBD — замерить** (колея) |

### SensorParams
| Параметр | Дефолт | Примечание |
|---|---|---|
| `odo_counts_per_motor_rev` | 16384 (энкодер) | Hall=30; **рекоменд. AS5047P** |
| `gyro_bias` | 0.004 рад/с | холодный bias, снимается калибровкой |
| `gyro_noise` | 0.003 рад/с | главный фактор точности без компаса |
| `accel_noise` | 0.15 м/с² | — |
| `current_max` | 40 А | для нормировки нагрузки |

### EstimatorConfig
| Параметр | Дефолт |
|---|---|
| `heading_source` | `gyro` |
| `calibrate_bias` | `true` |
| `zupt` | `true` |
| `breadcrumb_spacing` | 0.5 м |
| `v_still / gyro_still / accel_still` | 0.05 / 0.03 / 0.6 |

### SimConfig
| Параметр | Дефолт |
|---|---|
| `dt` | 0.02 с (50 Гц) |
| `cruise_speed` | 2.0 м/с (выезд) |
| `slip_long / slip_turn` | 0.02 / 0.10 (проскальзывание гусениц) |
| `calibration_seconds` | 15 с (boot-калибровка гиро) |
| `backtrack_stop_interval_m` | 75 м (BACKTRACK_PARTIAL) |
| `backtrack_stop_seconds` | 2.5 с |

### PursuitConfig
| Параметр | Дефолт |
|---|---|
| `cruise_speed` | 1.5 м/с (автономно, ниже ручного) |
| `lookahead` | 1.5 м |
| `max_yaw_rate` | 1.0 рад/с |
| `reverse` | `false` |

---

## 5. Метрики `SimSummary`

`final_return_error_m`, `final_error_pct`, `backtrack_completed`,
`est_heading_error_deg` (дрейф курса), `est_distance_error_m` (ошибка одометрии),
`mean_backtrack_erpm` (контроль бессенсорного порога), `final_gyro_bias_est` vs
`true_gyro_bias`, `est_outbound_error_m`, `breadcrumbs`, `ticks`.

---

## 6. Привязка к ТЗ

| Требование ТЗ | Где |
|---|---|
| FR-1 мониторинг связи / потеря | `simulation.cpp` (link_ok, переход в failsafe) |
| FR-2 запись трека (breadcrumbs) | `estimator.cpp` (`DeadReckoning::breadcrumbs`) |
| FR-3 GNSS-denied оценка состояния | `estimator.cpp` (гиро + одометрия + ZUPT + калибровка) |
| FR-4 выполнение backtrack | `controller.cpp` (pure pursuit, forward/reverse, BACKTRACK_PARTIAL) |
| FR-7 blackbox-лог | `logio.cpp` + `log.cpp` |
| НФТ watchdog/автрестарт | `deploy/backtrack.service` (`Restart=always`) |
| Риск: магнитометр от 84 В | отказ от компаса, курс с гиро (см. README) |
| §7 приёмка ≤15 м/3% | тест `returns_home_within_acceptance` + стенд-сценарий 1 |

---

## 7. Out of scope (этой версии)

Соответствует §10 ТЗ: нет объезда препятствий, нет SLAM/visual odometry, нет
реального транспорта MAVLink/CAN, склоны моделируются плоско (наклон по
акселерометру — будущее расширение). Это офлайн-песочница алгоритма.
