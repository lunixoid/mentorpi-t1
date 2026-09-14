# STATUS: Offline-режим детекции людей на роботе

sd: SD026
phase: closed
ba: approved
design: skipped
sa: approved

## Скипы
- BA: —
- Design: skipped — пользователь: СА без дизайна; команда `t1ctl` и `make pi-detect` без макетов
- Design-review: skipped (дизайна не было)

## Заметки
- **SD026 ЗАКРЫТ (2026-09-10, решение оператора).** После питания и `t1ctl restart` источник
  рамок — Mac, как до SD. Offline на Pi — только по команде `t1ctl detect offline` /
  `make pi-detect` (обратно `t1ctl detect mac` / `make pi-detect ARGS=mac`). Контракт
  `/perception/persons` и `/perception/nearest_person` тот же; геометрия F08 и lock F09 не
  менялись. Задачи T1–T7 закрыты. Ops этого SD: [`ops.md`](ops.md). Закрытые SD не патчили.
- Версии на закрытии: `mentorpi_person_detect` 0.1.5, `mentorpi_perception` 0.6.0,
  `mentorpi_bringup` 0.6.1, `t1ctl` 1.7.0. `mentorpi_msgs` не менялся. Не коммичено — коммит,
  `make env` (overlay-builder с ncnn), `make build` / `make deploy` и релиз за оператором.
- As-built относительно SA: инференс NCNN CPU, `num_threads=2`, `infer_period_ms=500`
  (между инференсами перепубликация последней рамки с stamp текущего RGB, чтобы не рвать
  `nearest_timeout_ms=300`); в launch `OMP_NUM_THREADS` / `NCNN_NUM_THREADS` = 2. Vulkan/Mesa
  нет. Декод Ultralytics `out0` — `[84×8400]`; WH, оставшиеся в ячейках сетки P3/P4/P5,
  домножаются на stride.
- QA на стенде до закрытия: 0.1.3 — люди не детектились (layout out0); 0.1.4 — overlay видел
  рамку ~80×116, nearest почти пустой, шасси рывками (`cloud_fresh=0`, Aurora не отдаёт
  `/aurora/points2` при полной загрузке CPU). 0.1.5 в дереве как ответ на stale points;
  качественная приёмка следования после деплоя 0.1.5 — за оператором.
- **Хвосты за пределами SD026:**
  1. Рамка onboard всё ещё не full-body (~80×116 px @ 640×400). Геометрию / порог 40 точек /
     slab не трогали (D5).
  2. Полная загрузка CPU детекции глушит `/aurora/points2` → nearest `valid: false` при живой
     рамке на overlay. Тот же класс, что хвост SD025 (`coasting` / `points_timeout_ms=700`).
     Отдельного SD не заводили.
  3. GPU-инференс (v3dv / LiteRT) — отдельный SD, если CPU NCNN снова забьёт плату (D1.2).
- Каталог: F08/F09 в `docs/SD/SD001/tech.md`; текущий пайплайн — SD013–SD017 (инференс на Mac) + SD026 (offline на Pi)
- СА утверждён в Plan; канон в `docs/SD/SD026/tech.md` (7 задач). Движок: NCNN YOLO11n CPU, без Vulkan/Mesa
- Первая итерация: после питания — как сейчас (Mac); offline включается командой `t1ctl` и с хоста `make pi-detect`
- Поведение offline неотличимо от текущего Mac-контура следования
- Покупка ИИ-ускорителя недопустима
- Отказ onboard-детекции для следования = как сейчас без Mac (ближайшего нет, робот стоит)
