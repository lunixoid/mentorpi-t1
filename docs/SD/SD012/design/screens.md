# Карта кадров

Рабочая папка: `docs/SD/SD012/design/`. Команда: `t1ctl` 1.4.1. Сетка: 80×24 (справка 80×67). Язык вывода: English.

| Файл | Режим | Команда | Задача | Поток |
|------|-------|---------|--------|-------|
| Вывод — help.html | CLI | `t1ctl --help` | COMMANDS + CALIB + STATUS `calibration` | stdout, exit 0 |
| Вывод — статус норма.html | CLI | `t1ctl status` | контур жив, `calibration factory` | stdout, exit 0 |
| Вывод — статус калибровка не применена.html | CLI | `t1ctl status` | `calibration unused`, next `t1ctl calib` | stdout, exit 0 |
| Вывод — калибровка завод.html | CLI | `t1ctl calib` | позы из модели, невязка, next `calib floor` | stdout, exit 0 |
| Вывод — калибровка пол набор.html | CLI | `t1ctl calib floor` | удержание: пол в кадре, счётчики без quota | stdout hold |
| Вывод — калибровка пол предложение.html | CLI | `t1ctl calib floor` | дельты камеры/IMU, accept/reject | stdout, exit 0 |
| Вывод — калибровка угол оси.html | CLI | `t1ctl calib corner` | `cause axis swap`, yaw 180→0 | stdout, exit 0 |
| Вывод — калибровка езда набор.html | CLI | `t1ctl calib drive` | пульт ведёт оператор; команда не едет | stdout hold |
| Вывод — калибровка лидар.html | CLI | `t1ctl calib lidar` | ручной ввод высоты и наклона | stdout, exit 0 |
| Схема — замер лидара.html | схема | — | точки замера высоты и наклона LD19 | не CLI |
| Схема — замер камеры.html | схема | — | точка замера высоты Aurora 930 | не CLI |
| Вывод — калибровка сохранение.html | CLI | `t1ctl calib save` | сводка дельт + `live after restart` | stdout, exit 0 |
| Вывод — калибровка файл.html | CLI | `t1ctl calib` | `source file`, lidar xyz `measured` | stdout, exit 0 |
| Вывод — калибровка отказ контур.html | CLI | `t1ctl calib floor` | demo down, нет глубины | kv+next stdout; `error:` stderr; exit 1 |
| Вывод — калибровка отказ сцена.html | CLI | `t1ctl calib corner` | одна стена, нет угла | kv+next stdout; `error:` stderr; exit 1 |
| Вывод — калибровка файл не прочитан.html | CLI | `t1ctl calib` | `source unused` + `reason`, живые позы factory | stdout, exit 0 |

Эталоны ДС: `design-system/Цвета.html`, `Типографика.html`, `Линии и формы.html`, `Компоненты.html`.

NO_COLOR: отдельного кадра нет — политика в `AGENTS.md`.

Кадры accept/reject/abort не рисуем: одна фраза по образцу `Motion held.` (`Floor kept.` / `Floor dropped.` / `Draft dropped.`).

Исторические кадры SD011: `docs/SD/SD011/design/`.
