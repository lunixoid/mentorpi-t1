# Design plan: t1ctl — дельта SD004 (F24)

Палитра, сетка 80×24, Red Hat Mono, ascii, промпт `$ `, English, kv 12 колонок — **без изменений** (см. SD002 `design-system/plan.md` и эталоны Цвета/Типографика/Линии там же). Сюда не копируем эталоны и не пересобираем холст.

## Предмет

Оператор по SSH смотрит `t1ctl`: жив ли контур **и** какой режим выбран, есть ли связь с пультом. Режим переключает **кнопка на пульте**, не новая команда CLI. Help остаётся USAGE → COMMANDS → STATUS → FLAGS.

Не показываем: скорость, Twist, «робот едет», мастер, JSON, pygame/js0, третье значение режима (`Forbidden`).

## Словарь (порядок стабильный)

```
demo        active | inactive | failed
stock       inactive | active
chassis     active | inactive
mode        follow | manual
remote      active | inactive
version     0.1.0
```

`chassis` уже в production с SD003 (макеты тогда скипнули). `mode follow` — следование за человеком; в выводе нет `autofollow` / `AutoFollow`. `remote` — link приёмника пульта, не `connected` / `[ON]`.

Роли цвета: `demo active` / `chassis active` / `remote active` = ok (олива). `mode follow` и `mode manual` оба **fg** (это выбор, не health). `remote inactive` = fg, как `stock inactive`, не warn.

Если `demo` не active: ключи `mode`/`remote` всё равно в блоке; значения `mode follow`, `remote inactive`. Не `n/a` / `unknown`.

## Кадры этой фазы

Четыре статуса при живом контуре (робот стоит — t1ctl скорость не рисует) + help. Подтверждения start/restart/stock не дублируем: тот же kv.

## Help

STATUS вырос (chassis + mode + remote, все значения). Чтобы удержать 80×24, **убраны пустые строки между секциями** help. Pad как SD002: ключ 12, value 15, пробел, описание dim. Описания chassis — как в текущем `t1ctl` (`/vehicle/status echo…`), без секунд таймаута.

## Самокритика

Не выдумывать спидметр, ASCII-стик, emoji пульта, второй экран TUI, warn на `manual`, ok на `follow`, чёрный матричный терминал.
