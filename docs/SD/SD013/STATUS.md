# STATUS: Пайплайн детекции людей (инференс на Mac)

sd: SD013
phase: qa-handoff
ba: approved
design: skipped
sa: approved

## Скипы
- BA: —
- Design: skipped — пользователь: СА без дизайна; рамки в существующей Image-панели Foxglove (SD005), отдельных экранов t1ctl нет
- Design-review: skipped — UI не менялся (design skipped)

## Заметки
- Каталог: F08 в `docs/SD/SD001/tech.md`; трекинг людей (F09) в этот SD не входит
- Виток: onboard YOLOv8n OpenCV DNN на CPU Pi 5 (ветка `feature/sd013`) отклонён — CPU занят целиком, человека в кадре нет
- Инференс на MacBook Air M3 24 ГБ; RGB Aurora — робот → Mac; гипотезы — Mac → робот; points2 остаётся на роботе для range; LAN стенда, без облака
- СА утверждён в Plan; канон в `docs/SD/SD013/tech.md` (6 задач). T1–T6 реализованы, ждут QA пользователя
- T6: CTest `test_person_geometry` на header-only `person_geometry.hpp` (синтетическое organized 640×400, без ROS/сети/MPS). `mentorpi_perception` 0.1.1. Сборку и `colcon test` агент не запускал (arm64-сборщик — пользователь)
- T4: `/perception/persons/overlay` bgr8 из RGB + Detection2D (и без глубины); нет RGB — топик молчит. Hint `persons overlay` в `t1ctl viewer status` (t1ctl 1.5.2). ops SD005. `foxglove_bridge.yaml` не сужали
- T5: `person_perception` в `stage1.launch.py` после camera_layer, YAML I1 из share; Mac/HailoRT/vendor YOLO/`bringup.launch.py` в launch нет. `mentorpi_bringup` 0.1.6, `exec_depend` на `mentorpi_perception`. `build-arm64.sh`: apt vision-msgs, cv-bridge, tf2-ros, tf2-geometry-msgs, libopencv-dev в `ros:humble` (не OpenCV 4.10). Сборку и деплой агент не запускал
- Mac обязателен: на роботе детекции нет. Отказ Mac/сети для следования = «в кадре никого нет»
- Локализация и лидар как сенсор класса «человек» в этот SD не входят
- Рамки детекции — в существующей Image-панели Foxglove (SD005)
- I5: Humble `vision_msgs/Pose2D` — центр рамки в `bbox.center.position.x/y`, не `center.x/y`
- Закрыт: пользователь подтвердил закрытие SD013; `F08` отмечена выполненной в каталоге `SD001`. Релиз / тег / деплой — пользователь.
