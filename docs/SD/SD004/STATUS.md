# STATUS: Ручное управление с пульта

sd: SD004
phase: qa-handoff
ba: approved
design: approved
sa: approved

## Скипы
- BA: —
- Design: —

## Заметки
- Имплементация и ручной QA пользователя завершены
- Design-review: принят ранее по макетам (`remote` в макете, в CLI — `remote controller`); в этом витке UI не менялся, `design.md` и preview не трогали
- F24: приёмник ShanWan `2563:0575` в USB Pi; `linux_joy` → `/joy` → `pad_teleop`. Не SDL `joy_node`, не pygame
- На шасси только `platform_adapter` → `/hiwonder_controller/cmd_vel`; `/cmd_vel` запрещён. Физический стоп — SD003 hardware halt; пульт сразу шлёт software zero на release/stale/Follow/disconnect
- Command freshness (`cmd_freshness_ms`) отдельно от remote presence (`joy_timeout_ms`). Event + keepalive без дубля. Shaping: enter 0.10 / release 0.06, linear_min 0.10, angular_min 0.40, max_linear 0.5, max_angular 2.0 — калибровка стенда, не SLA
- `t1ctl`: unified probe `/vehicle/status` + `/control/status` (transient_local + reliable), 2 попытки на холодный промах; `remote_controller: false` не timeout
- Ручной QA: управление корректно; forward/back и yaw правильные; нейтраль физически останавливает; реверс отзывчив; deadzone/anti-deadzone ощущаются правильно; моторы не шумят после release
- Движение на стенде — только оператор. Релиз / тег / деплой — пользователь
