# Дизайн-система: «Трак T1»

Режим: cli
Назначение: `t1ctl` — единственный операторский интерфейс MentorPi T1. SSH на хост Raspberry Pi (Debian), не внутри контейнера. Не TUI. Копирайт CLI — **английский**.

Исторические кадры SD011 — `docs/SD/SD011/design/`. Живая ДС и новые кадры калибровки — `docs/SD/SD012/design/`.

## Сетка и полотно

- Сетка: **80×24** (короткий вывод: status, calib, error)
- Справка `--help`: **80×67** (COMMANDS + CALIB + словарь STATUS). В HTML help локально `--rows: 67`, artboard 960×1644
- Ячейка: 12×24 px
- Chrome: 36 px, вид **label** (имя кадра + сетка). Не macos, не три точки
- Artboard короткий: 960×612 px
- Фон chrome: `#D8CEB6`
- Фон терминала: `#E9E1CC`

Зачем 80, не 100: оператор на Pi по SSH; kv и help читаются в классической ширине. Строка ≤ 80.

## Цветовая модель

- ANSI-16 по ролям; макет — truecolor
- NO_COLOR / пайп / `TERM=dumb`: те же символы и пробелы; без ANSI и без `.fg-*`. Смысл — словами (`factory`, `unused`, префикс `error:`). Не подменять цвет иконками. Отдельный кадр NO_COLOR не нужен

## Палитра (только эти цвета) → роли

- bg: `#E9E1CC` (холст)
- fg: `#241E16` (резина трака)
- muted / dim: `#6A6356`
- accent / warn: `#B33A1A` (окись; warn не отдельный hue)
- ok: `#3F4F20` (олива шасси)
- err: `#8C1E2E`
- info: `#241E16` (= fg, без седьмого оттенка)
- border (chrome): `#B7AD96`

Runtime: dim `\033[2m`, bold `\033[1m`, ok `\033[32m`, err `\033[31m`, failed/unused `\033[1;31m`. Макет задаёт truecolor-роли, не 16-color approximation.

## Типографика

- Mono: **Red Hat Mono**, regular 400 + bold 700; dim = muted, не меньший кегль
- Кегль в макете: 20px; line-height = 24px
- Лигатуры: выключены (`font-variant-ligatures: none`)

## Box-drawing

- Стиль: **ascii** (`-`, `|`, `+`, стрелка перехода и поправки `->`)
- Signature-линейки `=---` в выводе нет
- На кадрах статуса рамку не ставим — только kv. Пустая строка = пауза между блоком и next-step / `error:`

## Промпт и chrome

- Промпт: `$ ` (класс `.prompt`, цвет accent)
- Команда: `.cmd`, цвет fg
- Подпись кадра: `t1ctl — <сценарий>` слева, `80×24` или `80×67` справа

## Signature

Два места, без баннера:

1. `residual` — невязка камера↔лидар, `m` и `deg` через ` / `. Нет порога и нет роли ok/err на числе: решение принимает оператор.
2. `source unused` / `calibration unused` — **bold err**, сразу `reason` (только в `t1ctl calib`). В общем `t1ctl status` ключ `reason` не занимаем.

Поправка этапа: `->`, старое значение fg, новое fg. Грубый разворот осей: ключ `cause`, значение `axis swap` (без роли).

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
| `t1ctl calib` / `calib status` | live poses, residual, origin | kv | 0 |
| `t1ctl calib floor` | camera from floor, IMU from gravity | hold kv → proposal | 0 / 1 |
| `t1ctl calib corner` | camera to lidar at a corner | hold kv → proposal | 0 / 1 |
| `t1ctl calib drive` | lidar yaw from a pad run | hold kv → proposal | 0 / 1 |
| `t1ctl calib lidar --height M --pitch DEG --roll DEG` | enter lidar height and tilt | proposal delta | 0 / 1 |
| `t1ctl calib camera --height M` | enter camera height | proposal delta | 0 / 1 |
| `t1ctl calib accept` | keep last stage as draft | `Corner kept.` / `Drive kept.` / `Lidar kept.` / `Camera kept.` | 0 / 1 |
| `t1ctl calib reject` | drop last stage | `Floor dropped.` (etc.) | 0 / 1 |
| `t1ctl calib save` | write file; live after restart | summary + `Calibration saved.` | 0 / 1 |
| `t1ctl calib abort` | drop draft; file unchanged | `Draft dropped.` | 0 / 1 |
| `t1ctl --help` / `-h` | this help | справка | 0 |
| `t1ctl --version` / `-V` | print version | `t1ctl 1.4.1` | 0 |

`t1ctl calib` без подкоманды — status (не ошибка разбора). `t1ctl mode` без подкоманды — по-прежнему ошибка. Команда калибровки не публикует Twist.

## Состояния (текст, без SLA)

Ключ–значение. Ключи lowercase, выравнивание **18 колонок**. Значения — systemd-слова плюс калибровка. Ключ dim, значение ролью. Имена unit в kv не показываем, кроме help.

В `t1ctl status` добавляется один ключ, после `platform model`:

```
calibration        factory | file | unused
version            1.4.1
```

`calibration` — что **сейчас живёт в контуре**, не черновик процедуры.

- `factory` → fg (как `follow`)
- `file` → ok
- `unused` → bold err; next-step `  t1ctl calib`

`t1ctl calib` / `calib status`:

```
source             factory | file | unused
reason             invalid file | …
residual           <m> m / <deg> deg
draft              floor corner drive lidar
stage              floor | corner | drive | lidar
cause              axis swap
camera xyz         <x> <y> <z> m
camera rpy         <roll> <pitch> <yaw> deg
lidar xyz / rpy
imu xyz / rpy
```

Числа: метры, 3 знака; градусы, 1 знак. Поля xyz/rpy — по 6 колонок, знак в поле. В конце строки xyz/rpy — происхождение: `factory` | `computed` | `measured`. Высота и наклон 2D-лидара всегда `measured`, не `computed`.

`reason` в `t1ctl calib` печатается **только если** `source` = `unused`. `draft` — только если есть несохранённые этапы. `cause` — только при грубом развороте осей.

Поза в CLI — установка относительно базы (метры/градусы), без оптического RPY камеры.

Роли значений calib:

- `file` → ok
- `unused` → bold err
- `factory` / `computed` / `measured` / числа / `axis swap` / `residual` → fg
- `active` и прочие как в SD011

Предложение этапа (успех, stdout, exit 0): `stage`, опционально `cause`, `residual` до `->` после, только изменившиеся оси с `->` и единицей, затем next-step:

```
  t1ctl calib accept
  t1ctl calib reject
```

Набор данных (удержание stdout): `stage`, 1–2 dim-строки с отступом 2 пробела (что сделать), наблюдаемые счётчики без знаменателя «N / quota». Спиннера нет. Команда не двигает робота; в `calib drive` явная фраза, что едет оператор с пульта.

Отказ этапа: известный kv (если не врёт) + `error: calib <stage> failed` на stderr + indented detail + next-step. Предложение не печатать. Черновик и файл не трогать.

Save: сводка только дельт (ось, было `->` стало, тег этапа `floor`/`corner`/`drive`/`measured`), затем `Calibration saved.`, ключ `live after restart`, next `  t1ctl restart`.

Accept/reject/abort — одна фраза как `Motion held.`, без полного kv.

## Компоненты

- Help: USAGE → COMMANDS → VIEWER → MODE → CALIB → полный STATUS → FLAGS. Без пустых строк между секциями. Имена fg, описания dim, заголовки bold. CALIB: подкоманда pad 18. Колонки STATUS: ключ 18 + значение 15 + пробел + описание; описание ≤ 44 символа
- Статус: kv, без абзацев и без рамок
- Ошибка: известный kv (если не врёт) + `error:` на stderr + indented detail + next-step на stdout
- Таблица: пробелы, не CSS grid
- Прогресс: обновляемый kv на том же потоке, не бар и не спиннер
- Машинный JSON / `--plain`: нет
- Confirm `y/N`: нет; accept/reject — отдельные команды

## Антипаттерны

- Чёрный фон + кислотный зелёный, scanlines
- Три точки macOS, градиентный titlebar
- Радуга ролей на каждый ключ help
- Nerd-font, emoji, GUI-кнопки в `.term`
- JetBrains Mono, Fira Code, Menlo, SF Mono, IBM Plex как основной
- Rounded/heavy box-drawing и псевдоокна вокруг статуса
- Выдуманные секунды, PID, «lorem», SLA, порог calibrated
- Подписи о составе SD / отсутствующих фичах в выводе CLI
- Баннер `CALIBRATION INVALID` / reverse-bar / прогресс `####`
- Оптический RPY камеры и радианы в операторском kv
- Путь файла калибровки в выводе (место хранения — SA)
- Команда, которая сама публикует Twist
- Русский текст в stdout/stderr `t1ctl`

## Handoff

- Стек: **Clap** (CLI11 C++), пакет `t1ctl` 1.4.1
- Язык UI: English; ключи и значения lowercase
- stdout: help, статус, предложения, подтверждения, next-step, hold kv
- stderr: блок `error:` (префикс `error:` bold err)
- exit: 0 на help/status/успех этапа и write; 1 на отказ этапа / save
- NO_COLOR: уважать `NO_COLOR` и не-TTY — без ANSI, текст тот же
- Числа в макетах — пример вывода (заводские позы T1 + иллюстрация невязки), не критерий приёмки

## Технически

- Токены: `design-system/tokens.css`
- Эталоны: `design-system/Цвета.html`, `Типографика.html`, `Линии и формы.html`, `Компоненты.html`
- Макеты: `Вывод — *.html` в корне этой папки
- Карта: `screens.md`
- Plan: `design-system/plan.md`
