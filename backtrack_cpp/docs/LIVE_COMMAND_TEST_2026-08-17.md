# Живий тест командного каналу (2026-08-17)

Мета: реально поїхати назад по backtrack. Підсумок: **командний канал доведено
робочим, але реальний рух заблоковано двома конкретними стінами** (лінк + GUIDED).

## Що ПРАЦЮЄ (доведено наживо)

- **USB MAVLink CM4↔Pixhawk:** кабель USB від Pixhawk у малину → BlueOS одразу
  бачить автопілот. `/dev/ttyACM0`, lsusb `Holybro Pixhawk6C-bdshot`, mavlink2rest
  показує HEARTBEAT (ArduPilot, GROUND_ROVER). НЕ треба ні DroneCAN, ні тунелю для
  тесту.
- **CM4 КЕРУЄ автопілотом:** POST MAVLink через mavlink2rest
  (`http://192.168.2.2:6040/mavlink`) — зміна режиму HOLD→GUIDED пройшла (status
  200, custom_mode став 15). Тобто малина реально командує політником.
- **Arm працює:** борт армиться (оператор — перемикачем на пульті, RC5_OPTION=153).
  Бачили `Throttle armed`, armed=True.
- **Backtrack-мозок** (listen-only) паралельно рахує трек і повернення — валідовано
  раніше (679-crumb dry-run).

## СТІНА 1 — лінк USB нестабільний

USB відпадав **≥4 рази** за сесію: працює → `USB disconnect` (dmesg) → інколи
залипає в bootloader (`PX4 BL FMU v6C.x`, flag is_bootloader) → BlueOS
ardupilot-manager зависає (`/serials` 500, `board` null). Лікується лише
перетисканням кабелю / power-cycle політника / зміною USB-кабелю (був підозра на
charge-only кабель — інший кабель допоміг). **Висновок: USB механічно непридатний
для бойового/рухомого НРК.** Потрібен надійний лінк:
- **TELEM/serial** (роз'єм із фіксацією) — треба звільнити порт; АБО
- **DroneCAN serial-тунель по CAN1** (`CAN_D1_UC_SER_EN=1`) — по вже наявному CAN,
  але потрібен софт на CM4 (dronecan/pymavlink; борт офлайн — не встановити;
  потрібен разовий інтернет).

## СТІНА 2 — GUIDED відхиляється автопілотом

`MAV_CMD_DO_SET_MODE → GUIDED` дає **COMMAND_ACK: MAV_RESULT_FAILED**. Тобто план A
(CM4 шле GUIDED velocity `SET_POSITION_TARGET_LOCAL_NED`) поки НЕ працює — автопілот
не пускає в GUIDED. Найімовірніша причина: **немає валідної оцінки позиції/курсу**
(компас нездоровий — SYS_STATUS health bit 0x4; борт GNSS-denied/compass-free).
GUIDED у Rover вимагає позиції/yaw.

Додатково: коли пульт УВІМКНЕНО, борт тримається в MANUAL (custom_mode 0) і мій
GUIDED ігнорується; коли пульт ВИМКНЕНО — `Radio Failsafe`, arm заборонено. Тобто
поточний RC/mode-конфіг ще й конфліктує з зовнішнім керуванням.

## Конфіг (з pixhawk-6c-pro.params, підтверджено)

- ArduPilot Rover, node 10, CAN1 @1M. SERIAL1/2=MAVLink2.
- `MODE_CH=0` (пульт не вибирає режим), `INITIAL_MODE=0` (MANUAL), `MODE1=4` (HOLD).
- `RCMAP_THROTTLE=2`, `RCMAP_ROLL=1` (steering). RC 1000..2000, нейтраль ~1500.
- `RC5_OPTION=153` (arm-перемикач на пульті). `ARMING_CHECK=0`.
- `FS_GCS_ENABLE=1`, `FS_GCS_TIMEOUT=5`, `FS_ACTION=2` (Hold).

## Два шляхи до реального руху (рішення команди)

1. **Полагодити GUIDED** — розібратися з EKF/компасом/джерелом курсу в ArduPilot
   (QGC + ArduPilot-фахівець), щоб GUIDED вмикався. Тоді план A (елегантний,
   використовує наш dead-reckoning як GUIDED velocity) запрацює.
2. **RC-override у MANUAL** — CM4 шле `RC_CHANNELS_OVERRIDE` (throttle=ch2,
   steering=ch1), backtrack (v,ω) → throttle/steering. Працює БЕЗ GUIDED на цьому
   борту. Грубіше, але доступне вже. Потрібен стабільний лінк.

Обидва потребують СТІНИ 1 (надійний лінк) розв'язаної першою.

## Безпека

Увесь тест — на стенді, колеса від землі, оператор з рукою на живленні. Мотори з
CM4 так і не крутились (GUIDED не увімкнувся; RC-override не запускали через
падіння лінка). Backtrack listen-only (`can0-listen`, `backtrack-observe`) не
передавав у CAN нічого.
