# Дизайн-система: «Трак T1»

Рабочая папка SD004: дополнение ДС `t1ctl` (F24 — режим управления и связь с пультом). Палитра, сетка, шрифт, ascii, English, kv 12 колонок — как SD002. Эталоны Цвета / Типографика / Линии живут в `docs/SD/SD002/design/design-system/` и сюда не копируются. Ключ `chassis` уже в production с SD003 (дизайн-макеты тогда скипнули); в этой папке он впервые в кадрах.

Режим: cli
Назначение: `t1ctl` — состояние и управление нашим контуром MentorPi T1 по SSH. Оператор на Raspberry Pi / в контейнере образа. Не TUI. Копирайт CLI — **английский**. Режим follow/manual переключается кнопкой на пульте, не командой `t1ctl`. В статусе видны режим и наличие связи с пультом. Скорость / Twist / «робот едет» в выводе нет.

## Сетка и полотно

- Сетка: 80×24
- Ячейка: 12×24 px
- Chrome: 36 px, вид **label** (имя кадра + сетка). Не macos, не три точки
- Artboard: 960×612 px
- Фон chrome: `#D8CEB6`
- Фон терминала: `#E9E1CC`

## Цветовая модель

- ANSI-16 по ролям; макет — truecolor
- NO_COLOR / пайп / `TERM=dumb`: те же символы и пробелы; без ANSI и без `.fg-*`. Смысл состояний — словами: `active`, `inactive`, `failed`, `follow`, `manual`; префикс `error:`. Не подменять цвет иконками и не рисовать отдельный кадр, пока политика не сменится

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
- Заголовок панели: не используем boxed title на кадрах статуса; эталон «Линии и формы» (SD002) фиксирует алфавит

## Промпт и chrome

- Промпт: `$ ` (класс `.prompt`, цвет accent)
- Команда: `.cmd`, цвет fg
- Подпись кадра: `t1ctl — <сценарий>` слева, `80×24` справа

## Signature

Нет. Кадры начинаются с kv / `Started.` / help. Не вставлять `=---` и boxed title.

## Команды

Команд на смену режима нет.

| Вызов | Смысл |
|-------|--------|
| `t1ctl` / `t1ctl status` | print status |
| `t1ctl start` | start demo |
| `t1ctl restart` | restart demo |
| `t1ctl stock` | restore stock autostart |
| `t1ctl --help` / `-h` | this help |
| `t1ctl --version` / `-V` | print version |

## Состояния (текст, без SLA)

Ключ–значение. Ключи lowercase, значения lowercase (systemd-слова плюс `follow` / `manual`). Ключ в dim, значение ролью. Ключи фиксированы, выравнивание ключа в **12 колонок**. Порядок стабильный, одна колонка. На кадрах статуса **нет** подписей про состав образа / дорожную карту фичей / скорость.

```
demo        active | inactive | failed
stock       inactive | active
chassis     active | inactive
mode        follow | manual
remote      active | inactive
version     0.1.0
```

`demo` — `mentorpi-t1.service`. `stock` — `start_node.service`. Имя `start_node` в kv не показываем. `chassis` — echo `/vehicle/status` (как SD003). `0.1.0` — версия пакета `t1ctl`, не SLA продукта.

- `mode follow` — следование за человеком. В выводе нет `autofollow` / `AutoFollow`.
- `mode manual` — ручной пульт.
- `remote active` — пульт на связи.
- `remote inactive` — пульта нет / связь потеряна.
- Третьего значения режима (`Forbidden`) в этом SD нет.
- Если `demo` не active: `mode`/`remote` **всё равно в блоке**; значения `mode follow`, `remote inactive` (контур не читает пульт). Не `n/a` / `unknown`.

Цвет значений:

- `demo active`, `chassis active`, `remote active` — роль **ok** (олива)
- `demo inactive` при down (stock тоже inactive) — роль **warn**; после `stock` — fg (уступили stock)
- `demo failed` — роль **err**
- `stock` любое, `chassis inactive`, `remote inactive`, `version` — **fg**
- `mode follow` и `mode manual` — оба **fg** (выбор режима, не health). Не красить manual warn и follow ok
- Не иконки, не `[ON]`, не `connected`

Сценарии статуса (контур жив, робот стоит — скорость в t1ctl не показываем):

- **follow + pad:** `demo active`, `stock inactive`, `chassis active`, `mode follow`, `remote active`, `version`
- **manual + pad:** то же, `mode manual`, `remote active`
- **manual + lost:** то же, `mode manual`, `remote inactive` (режим остаётся manual)
- **follow + no pad:** то же, `mode follow`, `remote inactive` (boot / пульт не подключён)

Прочие сюжеты SD002:

- **down:** `demo inactive` (warn), `stock inactive`; `mode follow`, `remote inactive`; next step `t1ctl start`; no extra prose
- **error:** `demo failed`; stderr `error: demo start failed`; next `t1ctl start` / `t1ctl stock`
- after **start/restart:** `Started.` / `Restarted.` then the same kv block (все шесть ключей)
- after **stock:** `Restored stock autostart.` then `demo inactive`, `stock active` (`demo inactive` без warn); `mode follow`, `remote inactive`

## Компоненты

- Help: USAGE → COMMANDS → полный STATUS (поля и все значения, включая chassis / mode / remote) → FLAGS. Язык English. Не радуга флагов: имена fg, описания dim, заголовки bold. Pad STATUS: ключ 12, value 15, пробел, описание. Пустые строки между секциями help **сняты**, чтобы блок уместился в 80×24
- Статус: kv в порядке выше. Без абзацев «фичей нет», без стика/скорости
- Ошибка: kv + `error:` + команды; код в data-spec, не на экране
- Таблица: пробелы, не CSS grid
- Прогресс/спиннер: нет
- TUI / второй экран: нет

## Антипаттерны

- Чёрный фон + кислотный зелёный, scanlines
- Три точки macOS, градиентный titlebar
- Радуга ролей на каждый ключ help
- Nerd-font, emoji, GUI-кнопки в `.term`
- JetBrains Mono, Fira Code, Menlo, SF Mono как основной
- Rounded/heavy box-drawing и псевдоокна вокруг статуса
- Выдуманные секунды, PID-простыни, «lorem»
- Подписи о составе SD / отсутствующих фичах в выводе CLI
- Спидметр, ASCII-стик, emoji пульта, кадр «робот едет»
- `autofollow` / `AutoFollow` / `connected` / `[ON]` / `n/a` / `unknown` в выводе
- Третье значение режима; warn на `remote inactive`; цвет `mode` как health

## Handoff

- Стек: **Clap** (C++)
- Язык UI: English; ключи и значения статуса — lowercase
- stdout: help, статус, подтверждения start/restart/stock
- stderr: блок `error:` (префикс `error:`)
- exit: 0 на help/status/успех; 1 на ошибку старта/рестарта
- Машинный вывод: в этом изменении нет `--plain`/JSON
- NO_COLOR: уважать `NO_COLOR` и не-TTY — без ANSI, текст тот же

## Технически

- Токены: `design-system/tokens.css` (копия SD002, тот же набор)
- Эталоны Цвета / Типографика / Линии: `docs/SD/SD002/design/design-system/` — здесь не дублируем
- Макеты: `Вывод — *.html` в корне этой папки
- Карта: `screens.md`
- Plan: `design-system/plan.md` (только дельта SD004)
