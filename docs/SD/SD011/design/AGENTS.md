# Дизайн-система: «Трак T1»

Режим: cli
Назначение: `t1ctl` — единственный операторский интерфейс MentorPi T1. SSH на хост Raspberry Pi (Debian), не внутри контейнера. Не TUI. Копирайт CLI — **английский**.

Исторические кадры SD002 остаются в `docs/SD/SD002/design/`. Живая ДС и новые кадры — `docs/SD/SD011/design/`.

## Сетка и полотно

- Сетка: **80×24** (короткий вывод: status, mode, error)
- Справка `--help`: **80×52** (полный словарь STATUS не влезает в 24; в реальном SSH справка скроллится). В HTML help локально `--rows: 52`, artboard 960×1284
- Ячейка: 12×24 px
- Chrome: 36 px, вид **label** (имя кадра + сетка). Не macos, не три точки
- Artboard короткий: 960×612 px
- Фон chrome: `#D8CEB6`
- Фон терминала: `#E9E1CC`

Зачем 80, не 100: оператор на Pi по SSH; kv и help читаются в классической ширине. Строка ≤ 80.

## Цветовая модель

- ANSI-16 по ролям; макет — truecolor
- NO_COLOR / пайп / `TERM=dumb`: те же символы и пробелы; без ANSI и без `.fg-*`. Смысл — словами (`active`, `forbidden`, префикс `error:`). Не подменять цвет иконками. Отдельный кадр NO_COLOR не нужен

## Палитра (только эти цвета) → роли

- bg: `#E9E1CC` (холст)
- fg: `#241E16` (резина трака)
- muted / dim: `#6A6356`
- accent / warn: `#B33A1A` (окись; warn не отдельный hue)
- ok: `#3F4F20` (олива шасси)
- err: `#8C1E2E`
- info: `#241E16` (= fg, без седьмого оттенка)
- border (chrome): `#B7AD96`

Runtime: dim `\033[2m`, bold `\033[1m`, ok `\033[32m`, err `\033[31m`, failed `\033[1;31m`. Макет задаёт truecolor-роли, не 16-color approximation.

## Типографика

- Mono: **Red Hat Mono**, regular 400 + bold 700; dim = muted, не меньший кегль
- Кегль в макете: 20px; line-height = 24px
- Лигатуры: выключены (`font-variant-ligatures: none`)

## Box-drawing

- Стиль: **ascii** (`-`, `|`, `+`, стрелка перехода `->`)
- Signature-линейки `=---` в выводе нет
- На кадрах статуса рамку не ставим — только kv. Пустая строка = пауза между блоком и next-step / `error:`

## Промпт и chrome

- Промпт: `$ ` (класс `.prompt`, цвет accent)
- Команда: `.cmd`, цвет fg
- Подпись кадра: `t1ctl — <сценарий>` слева, `80×24` или `80×52` справа

## Signature

Одно место: значение `forbidden` — **bold err**, сразу под ним ключ `reason`. Не баннер, не reverse-bar, не boxed title. `follow` и `manual` без роли (как сейчас в коде). Next-step при запрете: `  t1ctl mode allow`.

## Команды

| Вызов | Смысл | stdout | exit |
|-------|--------|--------|------|
| `t1ctl` / `t1ctl status` | print status (default) | kv | 0 |
| `t1ctl start` | start demo | `Started.` | 0 / 1 |
| `t1ctl restart` | restart demo | `Restarted.` | 0 / 1 |
| `t1ctl stock` | restore stock autostart | `Restored stock autostart.` | 0 / 1 |
| `t1ctl viewer` / `status` / `start` / `stop` | Foxglove bridge | kv (+ hint) | 0 / 1 |
| `t1ctl mode forbid` | hold motion → Forbidden | `Motion held.` + transition | 0 / 1 |
| `t1ctl mode allow` | release hold → follow | `Motion allowed.` + transition | 0 / 1 |
| `t1ctl mode manual` | select operator pad | `Manual.` + transition | 0 / 1 |
| `t1ctl --help` / `-h` | this help | справка | 0 |
| `t1ctl --version` / `-V` | print version | `t1ctl 1.4.0` | 0 |

`t1ctl mode` без подкоманды — ошибка разбора (не скрытый status). Пульт Select переключает только `follow` ↔ `manual` и **не** выводит из `forbidden`.

## Состояния (текст, без SLA)

Ключ–значение. Ключи lowercase, выравнивание **18 колонок** (как `ui.cpp`: влезает `remote controller`). Значения — systemd-слова плюс режим. Ключ dim, значение ролью. Имена unit в kv не показываем, кроме help.

```
demo               active | inactive | failed
stock              inactive | active
chassis            active | inactive
mode               follow | manual | forbidden
reason             operator | no /pnc/desired_twist | no /control/motion_restriction | no /control/status
remote controller  active | inactive
lidar              active | degraded | inactive
camera             active | degraded | inactive
imu                active | degraded | inactive
odometry           active | degraded | inactive
platform model     active | degraded | inactive
version            1.4.0
```

`reason` печатается **только если** `mode` = `forbidden`. Иначе строки нет (как суффикс IMU `(no /imu)` — только когда есть что сказать).

Роли значений:

- `active` → ok (зелёный), кроме `stock active` (без роли: уступили Hiwonder)
- `failed` → bold err
- `demo inactive` при `stock inactive` → err (оба контура down)
- `degraded` → err
- `forbidden` → bold err
- `follow` / `manual` / `inactive` (кроме demo-down) / `reason` / `version` → fg

Переход режима (успех write):

```
Motion held.
mode               follow -> forbidden
reason             operator
```

Стрелка ascii `->`. Старое значение fg, новое — ролью нового состояния. Полный kv после write **не** дублируем: оператор при необходимости зовёт `t1ctl status`.

Отказ write: не печатать `mode follow` по умолчанию парсера — это ложь. Показать то, что известно (`demo` / `stock`), затем `error:` и фразу `motion is not allowed`.

Next-step: две ведущие пробела, команда целиком, как у `demo failed`.

## Компоненты

- Help: USAGE → COMMANDS → VIEWER → MODE → полный STATUS (поля и значения) → FLAGS. Без пустых строк между секциями (как `print_help`). Имена fg, описания dim, заголовки bold. Колонки STATUS: ключ 18 + значение 15 + пробел + описание; описание ≤ 44 символа
- Статус: kv, без абзацев и без рамок
- Ошибка: известный kv (если не врёт) + `error:` на stderr + indented detail + next-step на stdout
- Таблица: пробелы, не CSS grid
- Прогресс/спиннер: нет
- Машинный JSON / `--plain`: нет

## Антипаттерны

- Чёрный фон + кислотный зелёный, scanlines
- Три точки macOS, градиентный titlebar
- Радуга ролей на каждый ключ help
- Nerd-font, emoji, GUI-кнопки в `.term`
- JetBrains Mono, Fira Code, Menlo, SF Mono, IBM Plex как основной
- Rounded/heavy box-drawing и псевдоокна вокруг статуса
- Выдуманные секунды, PID, «lorem», SLA
- Подписи о составе SD / отсутствующих фичах в выводе CLI
- Баннер `MOTION HELD` / reverse-bar на всю ширину
- Печатать `mode follow`, если `/control/state` не прочитан
- Русский текст в stdout/stderr `t1ctl`

## Handoff

- Стек: **Clap** (CLI11 C++), пакет `t1ctl` 1.4.0
- Язык UI: English; ключи и значения lowercase
- stdout: help, статус, подтверждения, next-step
- stderr: блок `error:` (префикс `error:` bold err)
- exit: 0 на help/status/успех write; 1 на ошибку start/restart/stock/viewer/mode
- Топики режима: `/control/state`, `/control/status`. Контейнер `mentorpi-t1`, unit `mentorpi-t1.service`
- NO_COLOR: уважать `NO_COLOR` и не-TTY — без ANSI, текст тот же

## Технически

- Токены: `design-system/tokens.css`
- Эталоны: `design-system/Цвета.html`, `Типографика.html`, `Линии и формы.html`
- Макеты: `Вывод — *.html` в корне этой папки
- Карта: `screens.md`
- Plan: `design-system/plan.md`
