# Bring-up на реальному залізі (CM4 + RS485 CAN HAT + GY-9250)

Покрокова інструкція першого запуску `run_vehicle` на машині.
Алгоритмічна частина описана в `docs/REFERENCE.md`, стендові сценарії пісочниці — в `docs/STAND_TEST.md`.

## Залізо

| Компонент | Роль | Підключення |
|---|---|---|
| CM4 8GB/32GB eMMC (CM4008032, без WiFi) на адаптері 4B | обчислювач | — |
| Waveshare RS485 CAN HAT (MCP2515 + SP3485) | CAN до VESC, RS485 до пульта | 40-pin GPIO |
| GY-9250 (MPU-9250/9265) | гіроскоп/акселерометр | I2C: 3V3, GND, SDA→GPIO2, SCL→GPIO3 |
| 2× FlipSky FSESC 75450 | приводи гусениць | CAN_H/CAN_L шлейфом на обидва VESC |
| Пульт/радіомодем оператора | канал керування (FR-1) | RS485 A/B |

**Термінація CAN:** 120 Ом на обох кінцях шини (у HAT є перемичка; на дальньому VESC увімкнути свою).

**Магнітометр GY-9250 не використовується** — за ТЗ компас непридатний біля силової частини 84 В. Курс — тільки з гіроскопа (тому обов'язкові 15 с нерухомої калібрування після кожного старту сервісу).

## 1. Разове налаштування CM4

```bash
# на Pi, з каталогу backtrack_cpp:
./deploy/setup_cm4.sh     # I2C + SPI + mcp2515 overlay + UART + can0.service
sudo reboot
```

Якщо CAN мовчить — перевірте кварц на HAT: скрипт за замовчуванням ставить 12 МГц
(`OSC=8000000 ./deploy/setup_cm4.sh` для 8 МГц клонів).

Перевірка після ребуту:

```bash
i2cdetect -y 1                 # очікуємо 68 — GY-9250
ip -details link show can0     # state UP, bitrate 500000
candump can0                   # після налаштування VESC: кадри 0x0901/0x0902
```

## 2. Налаштування VESC Tool (обидва контролери)

- [ ] Motor wizard під QS138 70H V3 (5 pole pairs), **sensored FOC** (Hall,
      краще AS5047P) — крейсер повернення ~1200 ERPM нижче бессенсорного порога
- [ ] App Settings → General → **App to use: None** (керує тільки CAN)
- [ ] App Settings → CAN: baud **500k**, **VESC CAN ID: лівий = 1, правий = 2**
- [ ] CAN status messages: **Status 1 (ERPM/current/duty), rate ≥ 50 Hz**
- [ ] **Timeout**: App Settings → General → Timeout ~**500 мс**, brake current > 0
      — мотори самі зупиняться, якщо CM4 завис (страховка watchdog)

## 3. Деплой і запуск

```bash
# з ноутбука:
PI=pi@raspberrypi.local ./deploy/deploy.sh --vehicle
# логи:
ssh pi@raspberrypi.local journalctl -u vehicle -f
```

Або вручну на Pi:

```bash
./build.sh
./build/run_vehicle --log-level debug        # дефолти: can0, /dev/serial0, /dev/i2c-1
```

Ключові параметри (`run_vehicle --help` немає — дивись шапку `apps/run_vehicle.cpp`):
`--track-width`, `--wheel-radius`, `--external-gear` — **заміряти на платформі**
(зараз плейсхолдери 0.5 м / 0.12 м / 1.0); `--imu-axes` — орієнтація плати IMU
(наприклад `--imu-axes -y,x,z`, якщо плата повернута на 90°); `--reverse` —
повернення заднім ходом без розвороту; `--link-timeout` (0.5 с) — поріг FR-1.

## 4. Стендовий тест без радіо (op_console)

`op_console` імітує пульт: шле кадри оператора по serial (USB-RS485 свисток на
ноутбуці, або друга пара на самому Pi).

```bash
# ноутбук (USB-RS485 A/B на HAT A/B):
./build/op_console --uart /dev/ttyUSB0 --v 0.3 --omega 0.0
```

Сценарій приймання (повторює сценарій 1 зі STAND_TEST, але на залізі):

1. Запустити `vehicle` сервіс, **не рухати машину 15 с** (калібрування, лог
   `calibration done: gyro bias est ...`).
2. Запустити `op_console` з малими `--v/--omega` — стан `MANUAL`, гусениці
   слухаються пульта, breadcrumbs пишуться.
3. Проїхати контрольну дистанцію, **вбити op_console (Ctrl+C)** — через
   `--link-timeout` у лозі `LINK LOST -> FAILSAFE_BACKTRACK`, машина сама
   повертається по треку із зупинками кожні 75 м.
4. Запустити op_console знову — `link REGAINED -> MANUAL`: керування повернулося.
5. `--estop` — негайна зупинка при живому зв'язку.

Перші прогони — **на підставці (гусениці в повітрі)**, потім на майданчику.

## 5. Blackbox

Кожен запуск пише `data/vehicle_YYYYMMDD_HHMMSS.csv` (ті самі колонки, що в
пісочниці; `gt_*` = 0 на залізі). Тягнути на ноутбук для аналізу дрейфу/тюнінгу:

```bash
scp pi@raspberrypi.local:backtrack_cpp/data/vehicle_*.csv ./data/
```

## Відомі граблі

- **Немає кадрів у candump** → не той кварц в overlay (8 vs 12 МГц), нема
  термінації, або в VESC не ввімкнений status broadcast.
- **`i2cdetect` не бачить 0x68** → AD0 на GY-9250 підтягнутий вгору (адреса
  стане 0x69 → `--imu-addr 0x69`), або не залютовані піни.
- **Сміття на RS485** → переплутані A/B, або на порту досі висить login console
  (setup_cm4.sh прибирає, перевірити `cat /boot/firmware/cmdline.txt`).
- **BNO055 (коли доїде)** — друга реалізація `IImu` (I2C 0x28, режим сирих
  даних gyro+accel, БЕЗ вбудованого fusion — він вимагає магнітометр); решта
  коду не змінюється.
