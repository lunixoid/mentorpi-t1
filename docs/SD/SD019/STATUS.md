# STATUS: Поведение FollowPerson

sd: SD019
phase: qa-handoff
ba: approved
design: skipped
sa: approved

## Скипы
- BA: —
- Design: skipped — нет UI; статус поведения только ROS-топик; UX/`t1ctl`/Foxglove не входят
- Design-review: skipped — дизайна не было

## Заметки
- Каталог: F10; BA [solution.md](solution.md); СА [tech.md](tech.md) (T1–T4)
- T1: `FollowPersonStatus.msg`, `mentorpi_msgs` 1.1.0
- T2: пакет `mission_control` 0.1.0, `evaluate_follow_behavior`, CTest
- T3: нода `mission_control` → `/pnc/follow_person/status`; watchdog nearest; без Twist
- T4: `stage1.launch.py`, `mentorpi_bringup` 0.2.0, README
- F11 и F14 в этот SD не входят
- Закрыт: пользователь подтвердил закрытие SD019. Релиз / тег / деплой — пользователь.
