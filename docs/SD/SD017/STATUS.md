# STATUS: Сопровождение цели во времени

sd: SD017
phase: qa-handoff
ba: approved
design: skipped
sa: approved

## Скипы
- BA: —
- Design: skipped — экраны t1ctl и Foxglove не меняются; смена цели видна по `track_id` в `/perception/nearest_person`
- Design-review: skipped — UI не менялся

## Заметки
- Каталог: F09; BA [solution.md](solution.md); СА [tech.md](tech.md) (T1–T4)
- T1: ByteTrack `track()` + `Detection2D.id`
- T2: `person_target_lock` + unit-тесты
- T3: проводка в `person_perception`, YAML гистерезиса, coast при таймауте
- T4: `mentorpi_perception` 0.2.0, pixi `0.2.1` (`lap`), ops SD005, ссылки F09
- QA: `YOLO11n track failed: No module named 'lap'` — `lap>=0.5.12` в pixi (conda-forge), workspace `0.2.1`
- Закрыт: пользователь подтвердил закрытие SD017. Релиз / тег / деплой — пользователь.
