# STATUS: Задержка пайплайна детекции людей

sd: SD014
phase: qa-handoff
ba: approved
design: skipped
sa: approved

## Скипы
- BA: —
- Design: skipped — пользователь: СА без дизайна; overlay в существующей Image-панели Foxglove (SD005), отдельных экранов t1ctl нет
- Design-review: skipped — UI не менялся (design skipped)

## Заметки
- Каталог: F08 в `docs/SD/SD001/tech.md`; закрытый SD013 даёт пайплайн, этот SD — задержка относительно реальности
- Успех: детекции не отстают от реальности более чем на 100 мс; весь контур робот↔Mac, не только картинка в Foxglove
- Overlay на каждом кадре сырого RGB для продукта не обязателен; на изображении — желательно для отладки
- Отставание контура следования на роботе пользователь не проверял
- СА утверждён в Plan; канон в `docs/SD/SD014/tech.md`. T3/T7/T8 отклонены. T6 сделан: IR off, 10 fps
- D3/T3 **отклонены**. T7/T8 **отклонены**: compressed на стенде нет, JPEG-нода не нужна после замера CPU T6
- As-built CPU (debug off, Mac on): defaults rgb 15 + IR → aurora ~100–104%, docker **163%**, TX ~19.4 МБ/с. T6 rgb 10 + IR off → aurora **~63%**, docker **115–122%** (плюс живой foxglove_bridge ~17%, которого не было в срезе «до»), TX ~16.0 МБ/с. Writer RGB остался RELIABLE. hz RGB ~10.05. IR-топика нет
- T1–T2, T4–T6 реализованы, ждут QA где ещё не закрыто. T7/T8 не делаем
- T1: `PersonArray.header` (stamp/frame из `detections_2d`); `mentorpi_msgs` 1.0.0; persons/nearest из `on_detections`; таймер `rate_hz` — только пустой список по `detections_timeout_ms` (`header.stamp = now()`); WARN_THROTTLE если `now − stamp > 100ms` на непустом списке. Overlay не трогали. `mentorpi_perception` 0.1.2 (patch за event-driven persons). Сборку и деплой агент не запускал
- T2: параметр `publish_overlay` default false (YAML + declare); `on_rgb` без `toCvCopy`/publish пока false; runtime `set_parameters` через `add_on_set_parameters_callback`; overlay bgr8 как SD013 при true. Версия `mentorpi_perception` остаётся 0.1.2
- T4: `t1ctl debug on|off` и `t1ctl debug` (status) через docker exec + `ros_debug.py` (`get_parameters`/`set_parameters` на `/person_perception.publish_overlay`). stdout `T1CTL_DEBUG_OK` и `overlay: on|off`. YAML не пишется. `t1ctl` 1.5.3
- T5: ops SD005 и `t1ctl viewer status` — бой: persons / nearest / detections_2d / сырой RGB; overlay только после `t1ctl debug on` (bgr8, та же Image-панель). `foxglove_bridge.yaml` не меняли. Версию t1ctl не поднимали (уже 1.5.3)
- Закрыт: пользователь подтвердил закрытие SD014; `F08` в каталоге `SD001` уже отмечена выполненной (SD013 + SD014). Релиз / тег / деплой — пользователь.
