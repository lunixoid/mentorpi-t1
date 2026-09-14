# Дизайн-система: «Трак T1»

Режим: cli
Назначение: `t1ctl` — состояние и управление нашим контуром MentorPi T1 по SSH. Оператор на Raspberry Pi / в контейнере образа. Не TUI. Копирайт CLI — **английский**.

## Сетка и полотно

- Сетка: 80×24
- Ячейка: 12×24 px
- Chrome: 36 px, вид **label** (имя кадра + сетка). Не macos, не три точки
- Artboard: 960×612 px
- Фон chrome: `#D8CEB6`
- Фон терминала: `#E9E1CC`

## Цветовая модель

- ANSI-16 по ролям; макет — truecolor
- NO_COLOR / пайп / `TERM=dumb`: те же символы и пробелы; без ANSI и без `.fg-*`. Смысл состояний — словами systemd: `active`, `inactive`, `failed`; префикс `error:`. Не подменять цвет иконками и не рисовать отдельный кадр, пока политика не сменится

## Палитра (только эти цвета) → роли

- bg: `#E9E1CC` (холст)
- fg: `#241E16` (резина трака)
- muted / dim: `#6A6356`
- accent / warn: `#B33A1A` (окись; warn не отдельный hue)
- ok: `#3F4F20` (олива шасси)
- err: `#8C1E2E`
- info: `#241E16` (= fg, без седьмого оттенка)
- border (chrome): `#B7AD96`

## Типографика

- Mono: **Red Hat Mono**, regular 400 + bold 700; dim = muted, не меньший кегль
- Кегль в макете: 20px; line-height = 24px
- Лигатуры: выключены (`font-variant-ligatures: none`)

## Box-drawing

- Стиль: **ascii** (`-`, `|`, `+`)
- Signature-линейки `=---` нет (QA: убрана из вывода)
- Заголовок панели: не используем boxed title на кадрах статуса; эталон «Линии и формы» фиксирует алфавит

## Промпт и chrome

- Промпт: `$ ` (класс `.prompt`, цвет accent)
- Команда: `.cmd`, цвет fg
- Подпись кадра: `t1ctl — <сценарий>` слева, `80×24` справа

## Signature

Нет. Кадры начинаются с kv / `Started.` / help. Не вставлять `=---` и boxed title.

## Команды

| Вызов | Смысл |
|-------|--------|
| `t1ctl` / `t1ctl status` | print status |
| `t1ctl start` | start demo |
| `t1ctl restart` | restart demo |
| `t1ctl stock` | restore stock autostart |
| `t1ctl --help` / `-h` | this help |
| `t1ctl --version` / `-V` | print version |

## Состояния (текст, без SLA)

Ключ–значение. Ключи lowercase (`demo`, `stock`, `version`), значения — как у `systemctl is-active` / `is-failed` (`active`, `inactive`, `failed`). Ключ в dim, значение ролью. Ключи фиксированы, выравнивание ключа в 12 колонок. Строки chassis нет: `t1ctl` шасси не измеряет. На кадрах статуса **нет** подписей про состав образа / дорожную карту фичей.

```
demo        active | inactive | failed
stock       inactive | active
version     0.1.0
```

`demo` — `mentorpi-t1.service`. `stock` — `start_node.service`. Имя `start_node` в kv не показываем. `0.1.0` — версия пакета `t1ctl`, не SLA продукта.

- **running:** `demo active`, `stock inactive`, `version`
- **down:** оба `inactive`; next step `t1ctl start`; `demo inactive` роль warn; no extra prose
- **error:** `demo failed`; stderr `error: demo start failed`; next `t1ctl start` / `t1ctl stock`
- after **start/restart:** `Started.` / `Restarted.` then the kv block
- after **stock:** `Restored stock autostart.` then `demo inactive`, `stock active` (`demo inactive` без warn — уступили stock)

## Компоненты

- Help: USAGE → COMMANDS → полный STATUS (поля и все значения) → FLAGS. Язык English. Не радуга флагов: имена fg, описания dim, заголовки bold
- Статус: kv (`demo` / `stock` / `version`). Без абзацев «фичей нет»
- Ошибка: kv + `error:` + команды; код в data-spec, не на экране
- Таблица: пробелы, не CSS grid
- Прогресс/спиннер: нет

## Антипаттерны

- Чёрный фон + кислотный зелёный, scanlines
- Три точки macOS, градиентный titlebar
- Радуга ролей на каждый ключ help
- Nerd-font, emoji, GUI-кнопки в `.term`
- JetBrains Mono, Fira Code, Menlo, SF Mono как основной
- Rounded/heavy box-drawing и псевдоокна вокруг статуса
- Выдуманные секунды, PID-простыни, «lorem»
- Подписи о составе SD / отсутствующих фичах в выводе CLI

## Handoff

- Стек: **Clap** (C++)
- Язык UI: English; ключи и systemd-значения статуса — lowercase
- stdout: help, статус, подтверждения start/restart/stock
- stderr: блок `error:` (префикс `error:`)
- exit: 0 на help/status/успех; 1 на ошибку старта/рестарта
- Машинный вывод: в этом изменении нет `--plain`/JSON
- NO_COLOR: уважать `NO_COLOR` и не-TTY — без ANSI, текст тот же

## Технически

- Токены: `design-system/tokens.css`
- Эталоны: `design-system/Цвета.html`, `Типографика.html`, `Линии и формы.html`
- Макеты: `Вывод — *.html` в корне этой папки
- Карта: `screens.md`
- Plan: `design-system/plan.md`
