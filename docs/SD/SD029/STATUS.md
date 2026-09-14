# STATUS: Кадры камеры без потерь на роботе

sd: SD029
phase: closed
ba: approved
design: skipped
sa: approved

## Скипы
- BA: —
- Design: UI не меняется, кроме одной строки `t1ctl status` (текст задан в SA I4) — решение оператора: «СА без дизайна»

## Заметки
- **SD029 ЗАКРЫТ (2026-09-12, решение оператора).** Кадры камеры доходят до нод на Pi с частотой
  камеры: профиль Fast DDS «только UDPv4, буфер приёма 8 МБ» для всех нод stage1 плюс
  `net.core.rmem_max = 16777216` на хосте, который ставит `make deploy`. Задачи T1–T4 закрыты.
  Канон: [`tech.md`](tech.md). Закрытые SD не патчили.
- Версии на закрытии: `mentorpi_bringup` 0.7.0, `t1ctl` 1.8.0. Пакеты восприятия и детекции не
  менялись (их версии — из SD028). Не коммичено: коммит и релиз за оператором.
- Проверено перед закрытием (агент, стенд, робот в `forbidden`, питание от блока): кадры 10.0 Гц в
  mac и 8.9–9.9 Гц в offline при работающей NCNN, разрывы ≤ 0.22 с; рамки onboard-детектора
  10.2 Гц против 7.1 до SD029; Mac-режим с `make mac-detect` — 10.0 Гц; `dds buffers active` и
  `degraded` при заниженном `rmem_max`; неходовые AC SD028. **Ходовой вердикт по следованию
  оператор в чате не приводил** — закрытие по его решению.
- **T1–T4 сделаны (2026-09-11), ждём ходовую QA оператора и код-ревью.** Design-review skipped (UI не
  менялся, кроме строки `t1ctl status` по I4).
  - T1 (cursor-agent, composer-2.5): `mentorpi_bringup/config/fastdds_camera_frames.xml`,
    `FASTRTPS_DEFAULT_PROFILES_FILE` в `stage1.launch.py` до первой ноды (нет файла → `LogInfo`),
    bringup 0.7.0. Ревью: без замечаний.
  - T2 (cursor-agent): `host/sysctl.d/60-mentorpi-t1-dds.conf`, блок в `mk/deploy.sh`. Ревью нашло
    дефект: `ssh_pi "sysctl -n …"` без пути — у `pi` в неинтерактивном ssh `sysctl` нет в PATH,
    при `set -euo pipefail` деплой оборвался бы до замены overlay. Исправлено отдельной сессией
    cursor-agent на `/sbin/sysctl -n`.
  - T3 (cursor-agent): строка `dds buffers` (active / degraded со значением / unknown), help, тесты
    в `t1ctl_test` (active, degraded, unknown, рендер), t1ctl 1.8.0. Ревью: без замечаний, локально
    `ctest` зелёный.
  - Локально: `pre-commit` по всем файлам SD029 зелёный; `make build` — 14 пакетов, t1ctl aarch64 1.8.0;
    `make deploy` — `==> sysctl net.core.rmem_max=16777216`.
  - Стенд (робот в `forbidden`, питание от блока), `scratch/sd029/verify_sd029.sh`:
    - хост: `rmem_max` 16777216, `rmem_default` 212992, файл root 0644, `t1ctl 1.8.0`,
      `dds buffers active`; при временном `rmem_max=212992` — `degraded (rmem_max 212992 < 8388608,
      run make deploy)`, код выхода 0, после `sysctl -p` снова `active`;
    - профиль в окружении `aurora930_node`, `person_perception`, `person_detect_pi`, `motion_control`;
    - зонд с профилем, mac: облако, глубина, RGB 10.0 Гц, max разрыв 0.11 с;
      offline (NCNN): 8.9 / 9.7 / 9.9 Гц в трёх прогонах, max разрыв ≤ 0.22 с;
    - `/perception/detections_2d_onboard` в offline 10.2 Гц (было 7.1), в mac молчит;
    - Mac-режим: `make mac-detect` на Mac → `/perception/detections_2d` 10.0 Гц на роботе;
    - SD028 неходовые: пороги трекера и `handover_radius_m=0.70` в стартовых строках, `param get`
      `fuse_score` False / `track_buffer_s` 3.0 / `handover_radius_m` 0.7; отказ старта:
      `track_buffer_s must be > 0`, `track_low_thresh must be <= track_high_thresh`,
      `handover_radius_m must be >= 0`.
  - Стенд оставлен: `source: mac`, `forbidden`, профиль и sysctl SD029 стоят.
- Заведён 2026-09-11 по итогам QA SD028 и замеров глубины (см. [SD028 STATUS](../SD028/STATUS.md)).
  Оператор делегировал решения: «Заводи sd029, транспорт сравни и выбери сам, пиши БА, СА без
  дизайна — всё сам». Поэтому BA и SA утверждены без гейтов и Plan-режима; сравнение транспортов
  и выбор — агент.
- Реализация — через `cursor-agent --model composer-2.5`, каждая T-задача в отдельной сессии,
  агент (Claude) проверяет результат. Коммит — оператор. Сборку, деплой и проверку на стенде
  оператор разрешил агенту в этом SD («сборку и деплой ты запускать можешь… стенд включен и стоит
  безопасно, питается от блока, вольтаж впритык»).
- Сравнение транспортов (offline, NCNN, зонд + драйвер с профилем, по 2 прогона): UDPv4 8 МБ —
  облако 8.2/10.1 Гц, глубина и RGB 10.0 Гц, драйвер 84–92 %, подписчик 36–42 %; SHM 32 МБ + UDP —
  10.0–10.1 Гц, драйвер ~85 %, подписчик 46–47 %. Выбран UDPv4 8 МБ. Общую загрузку ядер между
  вариантами не сравниваем: в варианте UDP восприятие оставалось на буфере 208 КБ и почти не
  получало облако.
- Хвосты вне SD029: переход восприятия на картинку глубины (−~1 ядро CPU); `points_timeout_offline_ms`
  1000 можно пересмотреть после ходовой проверки; драйвер игнорирует `qos_overrides` (RELIABLE).
