# STATUS: Адаптер шасси (SIT)

sd: SD003
phase: qa-handoff
ba: approved
design: skipped
sa: approved

## Скипы
- BA: —
- Design: пользователь явно: дизайн не нужен, сразу тех анализ (строка chassis в t1ctl без макетов)

## Заметки
- Имплементация и ручной QA пользователя завершены
- Design-review skipped: UI не менялся, дизайна не было
- Фича F02 из каталога SD001
- Единственный выход на шасси: `platform_adapter` → `/hiwonder_controller/cmd_vel`; `/cmd_vel` запрещён; `MACHINE_TYPE=MentorPi_Tank`
- Физический стоп = T1 UART halt (JGB37×2, battery 0x1af4, полный zero IDs 1–4). `Twist=0` / `MotorsState=0` не критерий. Снято: `motor_rps_canon` / signed-zero как гарантия остановки
- Watchdog — параметр `cmd_timeout_ms` (default 100), не SLA. Event + keepalive `rate_hz`. Runtime: preflight на первый nonzero процесса; edge halt на nonzero→zero и прямой реверс; повторные zero UART подавлены до следующего nonzero
- `t1ctl chassis` — топик `/vehicle/status` жив; unified probe, 2 попытки на холодный промах
- Ручной QA: нейтраль физически останавливает; реверс отзывчив; первая команда после startup едет; моторы не шумят после release. Направления стика — SD004
- Движение на стенде — только оператор. Агент ненулевой Twist не публикует
- Релиз / тег / деплой — пользователь
