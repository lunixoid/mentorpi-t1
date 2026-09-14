# SD011. Дизайн модификации

## Компоненты
| Компонент | Действие | Зачем | Макет |
|-----------|----------|-------|-------|
| Help `t1ctl` | изменить | Оператор видит новую команду режима и словарь `forbidden`/`reason` в той же справке, что и текущий CLI | `design/Вывод — help.html` |
| Вывод статуса `t1ctl` | изменить | Оператор различает `follow`, `manual`, `forbidden`, видит причину запрета и следующий шаг для снятия удержания | `design/Вывод — статус запрет.html` |
| Команда смены режима с хоста | создать | Оператор может запретить движение, разрешить его обратно и явно выбрать ручной режим без пульта как единственного интерфейса | `design/Вывод — режим запрет.html` |
| Ошибка смены режима | создать | При недоступном контейнере или ноде режима CLI не врёт про `follow`, а явно показывает отказ и безопасный следующий шаг | `design/Вывод — режим отказ.html` |

## Макеты
- `docs/SD/SD011/design/Вывод — help.html` — `t1ctl --help`: COMMANDS, VIEWER, MODE, STATUS, FLAGS
- `docs/SD/SD011/design/Вывод — статус норма.html` — `t1ctl status`: `mode follow`, все обязательные статусы `active`
- `docs/SD/SD011/design/Вывод — статус запрет.html` — `t1ctl status`: `mode forbidden`, `reason`, next `t1ctl mode allow`
- `docs/SD/SD011/design/Вывод — режим запрет.html` — `t1ctl mode forbid`: `Motion held.`, переход `follow -> forbidden`, `reason operator`
- `docs/SD/SD011/design/Вывод — режим разрешение.html` — `t1ctl mode allow`: `Motion allowed.`, переход `forbidden -> follow`
- `docs/SD/SD011/design/Вывод — режим отказ.html` — отказ `t1ctl mode allow`: контейнер down, `error: mode change failed`, движение не разрешено
- ДС: `docs/SD/SD011/design/AGENTS.md`, эталоны — `docs/SD/SD011/design/design-system/Цвета.html`, `docs/SD/SD011/design/design-system/Типографика.html`, `docs/SD/SD011/design/design-system/Линии и формы.html`

## Что удаляем
- Нет. Исторические кадры `docs/SD/SD002/design/` и `docs/SD/SD004/design/` остаются как архив предыдущих SD; живая ДС `t1ctl` — в `docs/SD/SD011/design/`.

## Состояния
- **default:** `t1ctl status` показывает `mode follow`; при запрете движения самый заметный ключ в kv-блоке — `mode forbidden`, под ним `reason`, ниже next `t1ctl mode allow`
- **empty:** отдельного кадра нет; отсутствие запрета означает отсутствие строки `reason`
- **error:** смена режима не удалась; CLI показывает только достоверный kv, затем `error:` и не печатает ложный `mode follow`
- **forbidden by operator:** `t1ctl mode forbid` удерживает движение и печатает переход в `forbidden` с причиной `operator`
- **forbidden by degradation:** `t1ctl status` показывает `mode forbidden` и конкретную причину по топику, например `no /pnc/desired_twist`
