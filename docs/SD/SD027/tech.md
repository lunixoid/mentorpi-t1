# SD027. Технический дизайн

Окна свежести, подобранные на стенде в offline, действуют **только** пока источник рамок `offline`. После питания и после `t1ctl restart` снова Mac: `nearest_timeout_ms=300`, `points_timeout_ms=700`. Переключение — тот же `t1ctl detect`, без рестарта нод. Гейт `!coasting` не меняется.

## ToDo

Порядок: конфиг Mac по умолчанию, затем живой выбор окна по `detections_source`, затем `t1ctl`.

- [x] T1. Mac-окна в yaml/launch; offline-окна отдельными параметрами
  - **Файлы:** `src/mentorpi_perception/config/person_perception.yaml`, `src/mentorpi_bringup/launch/stage1.launch.py`, `src/mentorpi_perception/package.xml`, `src/mentorpi_bringup/package.xml`
  - **Что нужно сделать:** Вернуть рабочие Mac-значения: `points_timeout_ms=700`, у `motion_control` `nearest_timeout_ms=300`. Добавить `points_timeout_offline_ms=1000` и `nearest_timeout_offline_ms=1000`. У `motion_control` объявить `detections_source` default `mac` (как у перцепшна), чтобы `t1ctl detect` мог переключить оба узла. Комментарии — не SLA. Версии: `mentorpi_perception` 0.6.1, `mentorpi_bringup` 0.6.2.
  - **Критерии приёмки:**
    1. AC1. После деплоя и `t1ctl restart` (источник Mac) `ros2 param get /motion_control nearest_timeout_ms` → `300`, `ros2 param get /person_perception points_timeout_ms` → `700`.
    2. AC2. `ros2 param get /person_perception points_timeout_offline_ms` и `/motion_control nearest_timeout_offline_ms` → `1000`.
    3. AC3. `ros2 param get /mission_control nearest_timeout_ms` по-прежнему `1000`.
  - **Проверка:** `make build` / `make deploy` / `t1ctl restart`. Три `param get` из AC1–AC3, источник `t1ctl detect` → `mac`.

- [x] T2. Ноды применяют offline-окно при `detections_source=offline`
  - **Файлы:** `src/mentorpi_perception/src/person_perception.cpp`, `src/motion_control/src/motion_control.cpp`, `src/motion_control/package.xml`
  - **Что нужно сделать:** Оба узла хранят Mac- и offline-таймаут. Активное окно = offline-значение при `detections_source==offline`, иначе Mac. Смена `detections_source` через `set_parameters` (уже есть у перцепшна) пересчитывает окно без рестарта ноды. `motion_control` получает такой же callback. `param get` именованных ключей не подменяется: 300/700 остаются Mac-числами, 1000 — ключи `*_offline_ms`. Версия `motion_control` 0.4.1.
  - **Критерии приёмки:**
    1. AC1. В логе перцепшна при `t1ctl detect offline` есть `detections_source=offline` и применение `points_timeout` 1000.
    2. AC2. В логе `motion_control` после того же переключения активное `nearest_timeout_ms` 1000.
    3. AC3. `t1ctl detect mac` возвращает активные окна 700 и 300 без `t1ctl restart`.
  - **Проверка:** journal/`docker logs` нод плюс следование в AutoFollow offline vs Mac.

- [x] T3. `t1ctl detect` ставит `detections_source` и на `motion_control`
  - **Файлы:** `host/t1ctl/src/ros_detect.py`, `host/t1ctl/CMakeLists.txt`, `host/t1ctl/tests/test_status.cpp`, `docs/SD/SD026/ops.md`
  - **Что нужно сделать:** Хелпер `ros_detect.py` при `offline`/`mac` вызывает `set_parameters` `detections_source` у `/person_perception` и у `/motion_control`. Нет сервиса `motion_control` — отказ detect, как сейчас при отсутствии перцепшна. `enabled` у `person_detect_pi` как в SD026. Версия `t1ctl` 1.7.1. В ops SD026 описать окна Mac vs offline.
  - **Критерии приёмки:**
    1. AC1. `t1ctl detect offline` → `source: offline` и `ros2 param get /motion_control detections_source` → `offline`.
    2. AC2. `t1ctl detect mac` → оба `detections_source` снова `mac`.
    3. AC3. `t1ctl --version` → `1.7.1`.
  - **Проверка:** команды AC1–AC3 на стенде после деплоя `t1ctl`.
