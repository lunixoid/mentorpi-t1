# SD004. Дизайн модификации

## Компоненты
| Компонент | Действие | Зачем | Макет |
|-----------|----------|-------|-------|
| Вывод статуса `t1ctl` | изменить | Оператор видит режим (`mode`) и связь с пультом (`remote`) в том же kv, что demo/stock/chassis | `design/Вывод — статус следование пульт на связи.html` |
| Help STATUS | изменить | В `--help` те же ключи и все значения, без новых команд | `design/Вывод — help.html` |
| Команды CLI | — | Режим переключается кнопкой на пульте, не `t1ctl` | — |

## Макеты
- `docs/SD/SD004/design/Вывод — help.html` — USAGE / COMMANDS / STATUS (demo, stock, chassis, mode, remote, version) / FLAGS
- `docs/SD/SD004/design/Вывод — статус следование пульт на связи.html` — `mode follow`, `remote active`
- `docs/SD/SD004/design/Вывод — статус ручной.html` — `mode manual`, `remote active`
- `docs/SD/SD004/design/Вывод — статус ручной пульт потерян.html` — `mode manual`, `remote inactive`
- `docs/SD/SD004/design/Вывод — статус следование без пульта.html` — `mode follow`, `remote inactive`
- ДС: `docs/SD/SD004/design/AGENTS.md`; эталоны цвета/типа/линий — `docs/SD/SD002/design/design-system/`

Порядок ключей: `demo`, `stock`, `chassis`, `mode`, `remote`, `version`. Значения: `mode` — `follow` \| `manual`; `remote` — `active` \| `inactive`. `mode` всегда цветом fg; `remote active` — ok.

## Что удаляем
- Нет. Новых команд в `t1ctl` нет. Скорость, стик и «робот едет» в CLI не появляются.

## Состояния
- **default:** контур жив, `mode follow`, `remote inactive` (пульт не обязан быть на связи после boot)
- **empty:** не отдельный экран; пульт отсутствует = `remote inactive`
- **error:** ошибки старта demo не меняются (SD002); `remote inactive` — не ошибка
- **follow + pad:** `mode follow`, `remote active`
- **manual:** `mode manual`, `remote active`
- **manual + lost:** `mode manual`, `remote inactive`
