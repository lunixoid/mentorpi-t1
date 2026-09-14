# Карта кадров

Рабочая папка: `docs/SD/SD004/design/`. Команда: `t1ctl`. Сетка: 80×24. Язык вывода: English.

Дельта SD004: режим (`mode`) и связь с пультом (`remote`) в том же kv, что статус. Ключ `chassis` (SD003, макеты тогда не делали) входит в те же кадры. Эталоны ДС — в `docs/SD/SD002/design/design-system/`.

Подтверждения `t1ctl start` / `restart` / `stock` и кадр ошибки **не копируем**: после `Started.` / `Restarted.` / `Restored stock autostart.` / `error:` наследуют тот же блок kv (порядок и роли — `AGENTS.md`), что статус.

Кадра «робот едет», мастер и JSON нет: t1ctl скорость не показывает.

| Файл | Режим | Команда | Задача | Поток |
|------|-------|---------|--------|-------|
| Вывод — help.html | CLI | `t1ctl --help` | USAGE, COMMANDS, STATUS (demo/stock/chassis/mode/remote/version и все значения), FLAGS | stdout, exit 0 |
| Вывод — статус следование пульт на связи.html | CLI | `t1ctl` (= `status`) | контур жив; `mode follow`; `remote active`; стоит | stdout, exit 0 |
| Вывод — статус ручной.html | CLI | `t1ctl` | контур жив; `mode manual`; `remote active`; стик не отклонён | stdout, exit 0 |
| Вывод — статус ручной пульт потерян.html | CLI | `t1ctl` | `mode manual` остаётся; `remote inactive` | stdout, exit 0 |
| Вывод — статус следование без пульта.html | CLI | `t1ctl` | `mode follow`; `remote inactive` (boot / пульт не подключён) | stdout, exit 0 |

NO_COLOR: отдельного кадра нет — политика в `AGENTS.md`.
