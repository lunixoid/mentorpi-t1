# STATUS: Быстрый t1ctl

sd: SD009
phase: qa-handoff
ba: approved
design: skipped
sa: approved

## Скипы
- BA: утверждён ответами оператора в Plan (бюджет ~5 с, degraded одной строкой, start без повторного status)
- Design: та же kv-сетка t1ctl, без новых экранов; hint-строки снимаются, отдельный design.md не нужен

## Заметки
- Закрыт: пользователь подтвердил закрытие SD009 после QA на стенде
- План утверждён; канон в `docs/SD/SD009/tech.md`
- t1ctl 1.2.0 (as-built T4)
- T1–T4 сделаны: rclpy embed `ros_probe.py`, один `docker exec`, окно 2 с, `kRosProbeAttempts = 1`; печать/действия/viewer без live sensor-probe
- As-built топики/кадры — из текущего дерева (Aurora `/aurora/…`, `depth_camera_link`, `depth_cam_frame`), не из черновика SD008
- Design-review skipped (та же kv-сетка)
