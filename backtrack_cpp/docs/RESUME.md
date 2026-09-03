# Де ми зупинились і з чого продовжити (2026-08-10)

Короткий «якір» для наступної сесії. Деталі — у `FIELD_NOTES.md` (журнал) і
`SYSTEM_ARCHITECTURE_QUESTIONS.md` (блокери/відповіді).

## Зроблено (усе listen-only, у репо)

- Код на борту (Pi під BlueOS), 47/47 тестів. CAN listen-only піднімається сам
  (`deploy/can0-listen.service`). Логер `log_can.sh`.
- `can_sniff` виправлений: коректна DroneCAN-склейка (transfer_id+toggle),
  LSB-first декод, декод `esc.RawCommand`.
- **Одометрію розшифровано:** `esc.Status` від **node 50**,
  **esc_index 0 = ЛІВА, esc_index 2 = ПРАВА**, «+» rpm = вперед.
- **Новий HAL** `hal/dronecan_esc` (читає цю одометрію, READ-ONLY, `set_erpm`
  no-op) + **`apps/run_observe`** — listen-only dry-run backtrack-мозку
  (odometry-only курс, тригер SIGUSR1, логує намір, у CAN не пише).
- **Dry-run валідовано на стенді:** 679-crumb трек з реальних коліс → тригер →
  pure-pursuit видав коректну команду повернення. `can0 TX = 0`.

## Блокер до реального руху

Малина **не має командного каналу до автопілота**: mavlink2rest порожній (BlueOS
не отримує нічого від польотника). Прошивка **ArduPilot**; вільних TELEM/serial
**немає**; CM4 сидить на **Pixhawk CAN1** (та сама DroneCAN-шина, що й ESC).
Прямий `esc.RawCommand` від CM4 — заборонено (автопілот активний майстер ~130 Гц,
два майстри = небезпечно; командний бік ми не декодували).

## `.params` ОТРИМАНО — наступний гейт: рішення A/B/C

Параметри в репо: `pixhawk-6c-pro/pixhawk-6c-pro.params` (+ Lua). Розбір і опції —
у SYSTEM_ARCHITECTURE_QUESTIONS. Коротко: ArduPilot Rover, node 10, CAN1 1М,
failsafe=Hold(5с), **DroneCAN serial-тунель вимкнено (`CAN_D1_UC_SER_EN=0`)**.

**Мозок backtrack авто-стартує** listen-only: `deploy/backtrack-observe.service`
(run_observe). У CAN нічого не шле.

**ОБРАНО варіант A** (CM4 → MAVLink-over-CAN тунель → GUIDED). Повний план:
`docs/COMMAND_CHANNEL_PATH_A.md`.

Найближчий крок — **КОМАНДА** вмикає в ArduPilot DroneCAN serial-тунель
(`CAN_D1_UC_SER_EN=1` + MAVLink SERIALx у тунель). Критерій: у mavlink2rest
(`:6040`) з'явиться HEARTBEAT від sysid 1 (автопілот). Поки його немає — я
transmit-код не пишу (нема як перевірити наживо).

Після появи HEARTBEAT: я пишу `Mavlink2RestSink` (GUIDED velocity через
mavlink2rest) + командний режим, ганяю dry-run, потім контрольований перший рух на
стенді (крихітний газ, ліміти, рука на живленні) — лише з явним дозволом.

## ОНОВЛЕННЯ 2026-08-17 — живий тест командного каналу (див. LIVE_COMMAND_TEST_2026-08-17.md)

Підключили Pixhawk по **USB** у малину (найшвидший лінк для тесту). Доведено:
малина **керує автопілотом** (міняли режим HOLD→GUIDED, status 200), борт
**армиться**. АЛЕ реальний рух заблокований двома стінами:
- **СТІНА 1 — USB нестабільний:** відпадав ≥4 рази (disconnect / bootloader /
  ardupilot-manager 500). Непридатний для рухомого борту → треба TELEM або
  CAN-тунель (надійний лінк).
- **СТІНА 2 — GUIDED відхиляється:** `DO_SET_MODE GUIDED → MAV_RESULT_FAILED`
  (немає валідного курсу/позиції; компас нездоровий, GNSS-denied). План A (GUIDED
  velocity) поки не працює.

Два шляхи далі: (1) полагодити GUIDED (EKF/компас у QGC + ArduPilot-фахівець);
(2) **RC-override у MANUAL** — CM4 шле `RC_CHANNELS_OVERRIDE` (throttle=ch2,
steering=ch1), backtrack (v,ω)→throttle/steering. Працює без GUIDED. Обидва
потребують стабільного лінка. Код готовий під обидва — доробити швидко.

## Дані сесії

Сирі логи — на борту: `~/backtrack_cpp/field_logs/2026-08-10/`
(`esc_/cmd_standtest2.csv`, `observe.csv`). Розбір одометрії —
`field_logs/2026-08-10/standtest2_analysis.md` (у репо).
