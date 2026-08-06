# GRC-code — Link-Loss Backtrack для гусеничного НРК

C++17 (stdlib-only) система повернення UGV додому по власному треку при
втраті зв'язку, без GNSS і компаса. Робочий код — у `backtrack_cpp/`.

## Стан проєкту (2026-07-30)

- Ядро (dead reckoning + pure pursuit) готове, 47 юніт-тестів зелені
  (`./build.sh && ./build/unit_tests` у `backtrack_cpp/`).
- **Архітектура борта з'ясована нещодавно і ВІДРІЗНЯЄТЬСЯ від першої версії
  коду**: пульт → QGC (скоро ELRS) → **Pixhawk 6C Pro** → 2× FlipSky FSESC
  75450 напряму по CAN (шина DroneCAN: політник, Matek DroneCAN→PWM,
  ватметр CAN-L4-BM, GPS M9N). CM4 — companion тільки під backtrack.
  RS485 на борту НЕМАЄ → модулі `hal/rs485_link`, `op_console` і
  операторський протокол застарілі. `hal/vesc_socketcan` (VESC-native)
  ймовірно теж — шина DroneCAN. Деталі: `backtrack_cpp/docs/ARCHITECTURE.md`.

## Якщо це польова сесія (ноутбук біля малини)

Читай і виконуй `backtrack_cpp/docs/FIELD_SESSION_BRIEF.md` — там усе:
деплой, план тестів, куди складати логи, що документувати, які питання
ставити людям поруч.

## Ключові документи

| Файл | Що |
|---|---|
| `backtrack_cpp/docs/FIELD_SESSION_BRIEF.md` | процедура польової сесії |
| `backtrack_cpp/docs/CAN_TEST_PLAN.md` | перше підключення до живої CAN-шини |
| `backtrack_cpp/docs/ARCHITECTURE.md` | реальна архітектура НРК + пастки CAN-only |
| `backtrack_cpp/docs/SYSTEM_ARCHITECTURE_QUESTIONS.md` | відкриті питання (v3) |
| `backtrack_cpp/docs/HARDWARE_BRINGUP.md` | оверлеї CM4, VESC Tool чек-лист |
| `backtrack_cpp/docs/REFERENCE.md` | техдовідник ядра |
| `backtrack_cpp/docs/FIELD_NOTES.md` | журнал польових знахідок (ВЕДИ ЙОГО) |

## Правила

- Обидва варіанти збірки мають лишатись робочими: CMake і `./build.sh`
  (Linux-only драйвери огороджені; на macOS збираються ядро+тести+op_console).
- На живій шині борта — СПОЧАТКУ listen-only; нічого не слати в CAN, поки
  команда явно не дозволить.
- Всі знахідки/аномалії — у `docs/FIELD_NOTES.md`, логи — у `field_logs/`
  з датою в імені. Питання, що виникли — дописуй у
  `docs/SYSTEM_ARCHITECTURE_QUESTIONS.md`.
- Тести: перед комітом `./build.sh && ./build/unit_tests` (47+ мають пройти).
