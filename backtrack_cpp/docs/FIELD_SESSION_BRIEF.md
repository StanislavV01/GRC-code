# Бриф польової сесії (Claude на ноуті біля малини)

Ти — Claude Code на ноутбуці, підключеному до однієї мережі з CM4 на борту
НРК. Твої задачі по порядку: (1) задеплоїти код на малину, (2) пройти
CAN-розвідку за планом, (3) зібрати логи, (4) задокументувати все, що
з'ясувалось або пішло не так, (5) закрити відкриті питання через людей поруч.

**Головне правило безпеки: на шину борта НІЧОГО не передавати. Тільки
listen-only. Будь-яка передача в CAN — лише за явним дозволом команди.**

## 0. Зв'язок із малиною

```bash
# знайти малину (якщо ip невідомий):
ping -c2 raspberrypi.local || arp -a | grep -i "b8:27\|dc:a6\|e4:5f\|d8:3a"
ssh pi@raspberrypi.local     # логін/пароль спитати у команди
```

Немає ssh-доступу → попросити команду ввімкнути ssh (`sudo raspi-config`
→ Interface Options → SSH) або дати монітор+клавіатуру. Зафіксуй у
FIELD_NOTES: ip, hostname, користувача, версію ОС (`cat /etc/os-release`),
модель (`cat /proc/device-tree/model`).

## 1. Деплой

```bash
cd backtrack_cpp
PI=pi@<ip> ./deploy/deploy.sh        # rsync + збірка на Pi + smoke-run демо
```

`deploy.sh` вимагає rsync+ssh на ноуті та g++ на Pi
(`sudo apt install -y g++` — якщо Pi без інтернету, ставити з offline-кеша
або збирати з таром; задокументуй, як вийшло).

Якщо оверлеї ще не ставились НА ЦІЙ малині (перевір:
`grep mcp2515 /boot/firmware/config.txt /boot/config.txt 2>/dev/null`):

```bash
ssh pi@<ip> "cd backtrack_cpp && ./deploy/setup_cm4.sh"   # + sudo reboot
```

УВАГА: setup_cm4.sh вмикає й can0.service на 500k — для розвідки шини це
зайве, вимкни: `sudo systemctl disable --now can0` (бітрейт підбираємо
руками, listen-only). Сервіс `vehicle` НЕ вмикати взагалі — код під стару
архітектуру (див. ARCHITECTURE.md).

Перевірка деплоя: `ssh pi@<ip> "cd backtrack_cpp && ./build/unit_tests"`
→ 47+ passed.

## 2. CAN-розвідка

Виконуй `docs/CAN_TEST_PLAN.md` крок за кроком. Скорочено:

```bash
ssh pi@<ip>
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000 listen-only on
cd backtrack_cpp && ./build/can_sniff | tee ~/sniff_1M.log
# тиша/помилки -> down і вгору на 500000, 250000...
# помилка ip link на 1000000 = MCP2515@12МГц не тримає 1М (очікувано,
#   див. ARCHITECTURE.md розділ "Апаратне обмеження HAT") -> одразу 500к,
#   і якщо на 500к теж тиша, а шина точно жива -> шина на 1М, наш HAT
#   непридатний: зафіксуй, це рішення для команди (3 варіанти в CAN_TEST_PLAN §5)
```

Ключові перевірки (все — у FIELD_NOTES):
- [ ] фактичний бітрейт: ___
- [ ] node id всіх вузлів (očікуємо ~5: політник, 2 VESC, ватметр, GPS)
- [ ] `esc.Status` (1034): є? частота? — це наша одометрія
- [ ] оператор дає газ на підставці → rpm у сніфері живі, знаки/боки
- [ ] `esc.RawCommand` (1030): node id відправника = політник
- [ ] 11-бітні кадри (не-DroneCAN): були/ні
- [ ] 60 с сирого дампа: `candump -l can0` або
      `./build/can_sniff --seconds 60 | tee ~/raw60.log`

## 3. Логи — куди і як

На ноуті, у корені `backtrack_cpp/`:

```bash
mkdir -p field_logs/$(date +%Y-%m-%d)
scp "pi@<ip>:~/sniff*.log" "pi@<ip>:~/raw60.log" field_logs/$(date +%Y-%m-%d)/
ssh pi@<ip> "ip -details -statistics link show can0" \
    > field_logs/$(date +%Y-%m-%d)/can0_state.txt
ssh pi@<ip> "dmesg | grep -i -E 'mcp|can|spi'" \
    > field_logs/$(date +%Y-%m-%d)/dmesg_can.txt
```

Плюс усе нестандартне: повний dmesg при проблемах з оверлеєм, вивід
невдалих команд тощо.

## 4. Документування

- **`docs/FIELD_NOTES.md`** — журнал сесії: дата, хто поруч, що робили,
  що вийшло/ні, всі числа з чек-листа вище. Пиши по ходу, не в кінці.
- Все, що суперечить `ARCHITECTURE.md` — виправляй одразу в
  `ARCHITECTURE.md` з позначкою дати.
- Нові питання — дописуй у `SYSTEM_ARCHITECTURE_QUESTIONS.md`.
- Наприкінці: коміт усього (`git add -A && git commit`) — логи теж.

## 5. Питання людям поруч (поки все крутиться)

Пріоритет — блокери з `SYSTEM_ARCHITECTURE_QUESTIONS.md` v3:

1. Прошивка політника: ArduPilot Rover чи PX4? Версія? (видно в QGC →
   Vehicle Setup / Summary; сфотографуй)
2. Чи вільний TELEM1/TELEM2 на політнику під CM4? Через який порт зараз
   заходить інтернет-модем QGC?
3. Попросити **експорт параметрів політника** (QGC → Parameters → Tools →
   Save to file) → поклади у `field_logs/<дата>/params.params`. З нього
   знімемо failsafe-конфіг, CAN-бітрейт, skid-steer налаштування.
4. Попросити dataflash-лог типової поїздки → `field_logs/<дата>/`.
5. Глибина ковзного буфера треку (м або хв)? Крок перевірки лінка при
   поверненні (кожні N м)? Скільки секунд без лінка = старт повернення?
6. Лінк відновився під час повернення — віддавати керування миттєво чи
   по явній дії оператора?

## 6. Чого НЕ робити

- Не вмикати `vehicle.service` і не запускати `run_vehicle` на борту —
  він написаний під стару (хибну) архітектуру з RS485.
- Не знімати listen-only без явного дозволу команди.
- Не вмикати термінатор 120 Ом на HAT (шина вже термінована).
- Не міняти параметри політника самостійно — тільки читання/експорт.
