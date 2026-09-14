# SD002. Дизайн модификации

## Компоненты
| Компонент | Действие | Зачем | Макет |
|-----------|----------|-------|-------|
| `t1ctl` (CLI) | создать | Консоль: состояние demo/stock, запуск, перезапуск | `design/Вывод — help.html` |
| Вывод статуса (active) | создать | Оператор видит `demo active` и `version` | `design/Вывод — статус жив.html` |
| Вывод статуса (inactive) | создать | Оператор видит оба `inactive` и `t1ctl start` | `design/Вывод — статус не запущен.html` |
| Вывод запуска | создать | Из утилиты поднять demo | `design/Вывод — запуск.html` |
| Вывод перезапуска | создать | Из утилиты перезапустить demo | `design/Вывод — перезапуск.html` |
| Вывод ошибки | создать | Старт не удался: `error:` и следующие команды | `design/Вывод — ошибка.html` |
| Вывод `stock` | создать | Одна команда по SSH — исходный автозапуск Hiwonder | `design/Вывод — исходный Hiwonder.html` |

## Макеты
- `docs/SD002/design/Вывод — help.html` — English help: COMMANDS + полный STATUS (поля и значения) + FLAGS
- `docs/SD002/design/Вывод — статус жив.html` — `demo active`, `stock inactive`, `version`
- `docs/SD002/design/Вывод — статус не запущен.html` — оба `inactive`, next `t1ctl start`
- `docs/SD002/design/Вывод — запуск.html` — `Started.` и блок `demo active`
- `docs/SD002/design/Вывод — перезапуск.html` — `Restarted.` и блок `demo active`
- `docs/SD002/design/Вывод — ошибка.html` — `demo failed`, `error: demo start failed`, `start` / `stock`
- `docs/SD002/design/Вывод — исходный Hiwonder.html` — `demo inactive`, `stock active`
- ДС: `docs/SD002/design/AGENTS.md`, эталоны в `docs/SD002/design/design-system/`

## Что удаляем
- Подписи в CLI про состав образа и отсутствующие фичи (в т.ч. «фичей следования в этом образе ещё нет»). В статусе — `demo` / `stock` / `version`; шасси нет.
- Штатный вывод Hiwonder и игры не являются UI этого изменения; после `t1ctl stock` действует их автозапуск.

## Состояния
- **default:** `t1ctl` / `t1ctl status` — `demo active`, `stock inactive`, `version 0.1.0`
- **empty:** оба `inactive`; next `t1ctl start`; без абзацев про проект
- **error:** `demo failed`; префикс `error: demo start failed`; `t1ctl start` или `t1ctl stock`
- **после start/restart:** `Started.` / `Restarted.` и блок default
- **stock:** `demo inactive`, `stock active`, `version`
