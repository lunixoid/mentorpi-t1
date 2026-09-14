# STATUS: Кадры Mac по Wi-Fi

sd: SD016
phase: qa-handoff
ba: approved
design: skipped
sa: approved

## Скипы
- BA: —
- Design: skipped — пользователь: UX не нужен; экраны t1ctl и Foxglove не меняются
- Design-review: skipped — UI не менялся

## Заметки
- Каталог: F08; канал Mac↔Pi для perception (Ethernet если ping с Pi, иначе точка робота)
- СА утверждён в чате (Plan UI недоступен); канон в [tech.md](tech.md) (T1–T5)
- T1: dual unicast FastDDS в `activate-dds.sh`
- T2: ICMP SOCK_DGRAM + `/perception/dds_peer`; `mentorpi_perception` 0.1.3 → 0.1.4 с T5
- T3: подписка `person_detect` на `/perception/dds_peer`
- T4: pixi `0.1.2`, README, ops SD005
- T5: гистерезис `ethernet_ping_fail_threshold=3`
- Закрыт: пользователь подтвердил закрытие SD016. Релиз / тег / деплой — пользователь.
