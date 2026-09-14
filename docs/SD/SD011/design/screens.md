# Карта кадров

Рабочая папка: `docs/SD/SD011/design/`. Команда: `t1ctl` 1.4.0. Сетка: 80×24 (справка 80×52). Язык вывода: English.

| Файл | Режим | Команда | Задача | Поток |
|------|-------|---------|--------|-------|
| Вывод — help.html | CLI | `t1ctl --help` | USAGE, COMMANDS, VIEWER, MODE, полный STATUS, FLAGS | stdout, exit 0 |
| Вывод — статус норма.html | CLI | `t1ctl status` | AutoFollow, все датчики `active`, `mode follow` | stdout, exit 0 |
| Вывод — статус запрет.html | CLI | `t1ctl status` | `mode forbidden`, `reason` (деградация), next `mode allow` | stdout, exit 0 |
| Вывод — режим запрет.html | CLI | `t1ctl mode forbid` | `Motion held.` follow → forbidden, `reason operator` | stdout, exit 0 |
| Вывод — режим разрешение.html | CLI | `t1ctl mode allow` | `Motion allowed.` forbidden → follow | stdout, exit 0 |
| Вывод — режим отказ.html | CLI | `t1ctl mode allow` | контейнер down; `error:`; `motion is not allowed` | kv+next stdout; `error:` stderr; exit 1 |

Эталоны ДС (не сценарии продукта): `design-system/Цвета.html`, `Типографика.html`, `Линии и формы.html`.

NO_COLOR: отдельного кадра нет — политика в `AGENTS.md`.

Исторические кадры SD002 (start/stock/viewer не повторяем здесь): `docs/SD/SD002/design/`.
