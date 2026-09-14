# Карта кадров

Рабочая папка: `docs/SD002/design/`. Команда: `t1ctl`. Сетка: 80×24. Язык вывода: English.

| Файл | Режим | Команда | Задача | Поток |
|------|-------|---------|--------|-------|
| Вывод — help.html | CLI | `t1ctl --help` | USAGE, COMMANDS, полный STATUS, FLAGS | stdout, exit 0 |
| Вывод — статус жив.html | CLI | `t1ctl` (= `status`) | `demo active`, `stock inactive`, `version` | stdout, exit 0 |
| Вывод — статус не запущен.html | CLI | `t1ctl` | оба `inactive`; next `start` | stdout, exit 0 |
| Вывод — запуск.html | CLI | `t1ctl start` | Started. + `demo active` | stdout, exit 0 |
| Вывод — перезапуск.html | CLI | `t1ctl restart` | Restarted. + `demo active` | stdout, exit 0 |
| Вывод — ошибка.html | CLI | `t1ctl start` | `demo failed`; `error:` | kv stdout; `error:` stderr; exit 1 |
| Вывод — исходный Hiwonder.html | CLI | `t1ctl stock` | `demo inactive`, `stock active` | stdout, exit 0 |

Эталоны ДС (не сценарии продукта): `design-system/Цвета.html`, `Типографика.html`, `Линии и формы.html`.

NO_COLOR: отдельного кадра нет — политика в `AGENTS.md`.
