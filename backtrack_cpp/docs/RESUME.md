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

## Наступний крок — ПОТРІБЕН `.params`

Користувач приносить **експорт параметрів ArduPilot** (QGC → Parameters → Tools →
Save to file). Далі:

1. Прочитати CAN-конфіг (`CAN_P*`, `CAN_D*`), з'ясувати, як увімкнути **MAVLink
   поверх CAN (DroneCAN tunnel)** → BlueOS/CM4 зможе слати GUIDED, а автопілот
   лишається єдиним майстром моторів (безпечний шлях, без нового дроту). Також —
   failsafe (`FS_*`), RC (SBUS?), skid-steer/сервоконфіг.
2. Спланувати ввімкнення каналу — **зміна параметрів ArduPilot = дія команди**
   (я параметри політника лише читаю, не міняю).
3. Контрольований перший рух на стенді: крихітний газ, обмеження швидкості/часу,
   рука на вимкненні живлення. Тільки з явним дозволом слати в CAN.

## Дані сесії

Сирі логи — на борту: `~/backtrack_cpp/field_logs/2026-08-10/`
(`esc_/cmd_standtest2.csv`, `observe.csv`). Розбір одометрії —
`field_logs/2026-08-10/standtest2_analysis.md` (у репо).
