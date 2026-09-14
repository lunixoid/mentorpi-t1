# SD026. Offline-детекция людей на роботе — инструкция оператора

Контракт и as-built: [`tech.md`](tech.md). Статус закрытия: [`STATUS.md`](STATUS.md). Панели Foxglove — в [`../SD005/ops.md`](../SD005/ops.md), этот файл их не дублирует.

После питания и после `t1ctl restart` источник рамок — **Mac** (`make mac-detect`). Offline на Pi включается **только** явной командой. Режим в YAML на диск не пишется.

**Не публиковать** `ros2 topic pub` в `/cmd_vel` и `/hiwonder_controller/cmd_vel`. Не запускать vendor `bringup.launch.py` параллельно с нашим контуром. Не `docker rm MentorPi`. ИИ-ускоритель и вендорский YOLO в этот контур не входят.

## Доступ

| Что | Как |
| --- | --- |
| Pi | `pi@192.168.88.56`, пароль `raspberrypi` |
| CLI | `t1ctl` **на хосте Pi**, не в контейнере |
| ROS 2 | контейнер `mentorpi-t1`, всегда `bash` и `.hiwonderrc` (иначе пустой `ros2 topic list`) |
| С Mac | `make pi-detect` / `make pi-detect ARGS=mac` (тот же SSH, что `make deploy`) |

Обёртка для ROS внутри контейнера (повторять перед каждой `ros2`-командой или зайти в интерактивный `bash`):

```bash
docker exec -it -u ubuntu mentorpi-t1 bash -lc '
  source /home/ubuntu/ros2_ws/.hiwonderrc
  source /home/ubuntu/mentorpi_t1_ws/install/setup.bash
  exec bash
'
```

Команды `ros2` после `.hiwonderrc` в той же сессии часто падают с `context is invalid`. Каждую проверку — одним `bash -lc 'source …; ros2 …'`.

## 1. Выкладка

С **Mac** (не собирать overlay на Pi). Если менялся `docker/overlay-builder/Dockerfile` (ncnn) — сначала `make env`.

```bash
make build
make deploy
```

`make provision` из‑за этого SD не требуется. Не копировать Mac-бинарь `t1ctl` на Pi.

Версии as-built: `mentorpi_person_detect` 0.1.5, `mentorpi_perception` 0.6.0, `mentorpi_bringup` 0.6.1, `t1ctl` 1.7.0.

## 2. Включить / выключить offline

На **хосте Pi**:

```bash
t1ctl detect
```

Ожидание после питания / `t1ctl restart`:

```
T1CTL_DETECT_OK=1
source: mac
```

Включить offline:

```bash
t1ctl detect offline
```

Ожидание:

```
T1CTL_DETECT_OK=1
source: offline
```

Вернуть Mac без рестарта overlay:

```bash
t1ctl detect mac
```

Ожидание:

```
T1CTL_DETECT_OK=1
source: mac
```

С машины разработки (тот же эффект по SSH):

```bash
make pi-detect              # → t1ctl detect offline
make pi-detect ARGS=mac     # → t1ctl detect mac
```

Порядок на роботе: `offline` сначала включает `person_detect_pi.enabled`, затем `person_perception.detections_source=offline`. `mac` — сначала источник Mac, затем `enabled=false`. Контейнер или нода недоступны — ненулевой exit, режим не «тихо становится mac».

```bash
t1ctl detect offline
# при мёртвом контейнере / нет ноды, например:
# T1CTL_DETECT_OK=0
# detail: node /person_detect_pi is not available
```

`t1ctl restart` и перезагрузка Pi сбрасывают offline → снова Mac.

## 3. Что смотреть

В offline `person_perception` читает только `/perception/detections_2d_onboard`. Если Mac всё ещё шлёт `/perception/detections_2d`, они в геометрию не попадают. Бой — те же `/perception/persons` и `/perception/nearest_person`.

Overlay (`t1ctl debug on`) рисует рамки **выбранного** источника, даже без валидной глубины. Нет RGB — overlay молчит.

As-built инференса: NCNN YOLO11n CPU, `num_threads=2`, `infer_period_ms=500` (~2 Гц). Между инференсами нода перепубликует последнюю рамку с stamp текущего RGB, чтобы не рвать `nearest_timeout_ms=300`. Launch задаёт `OMP_NUM_THREADS` / `NCNN_NUM_THREADS` = 2. Цифр SLA FPS нет.

Если onboard не даёт гипотез (пустой кадр или отказ детектора) — `nearest` пустой, робот в режиме следования стоит. Вымышленных людей нет.

Проверка источника и нод (один `bash -lc`):

```bash
docker exec -u ubuntu mentorpi-t1 bash -lc '
  source /home/ubuntu/ros2_ws/.hiwonderrc
  source /home/ubuntu/mentorpi_t1_ws/install/setup.bash
  ros2 param get /person_perception detections_source
  ros2 param get /person_detect_pi enabled
'
```

Ожидание в offline:

```
String value is: offline
Boolean value is: True
```

Ожидание после питания / `mac`:

```
String value is: mac
Boolean value is: False
```

Рамки onboard (человек в кадре, offline включён):

```bash
docker exec -u ubuntu mentorpi-t1 bash -lc '
  source /home/ubuntu/ros2_ws/.hiwonderrc
  source /home/ubuntu/mentorpi_t1_ws/install/setup.bash
  ros2 topic echo --once /perception/detections_2d_onboard
'
```

Ожидание: `class_id: person`, непустой `id` (ByteTrack), stamp как у RGB.

Nearest (нужны живые `/aurora/points2`, не только рамка):

```bash
docker exec -u ubuntu mentorpi-t1 bash -lc '
  source /home/ubuntu/ros2_ws/.hiwonderrc
  source /home/ubuntu/mentorpi_t1_ws/install/setup.bash
  ros2 topic echo --once /perception/nearest_person
'
```

Ожидание при человеке в кадре и свежем облаке: `valid: true`. Overlay с рамкой при `valid: false` — смотреть облако, не детектор.

## 4. Диагностика

| Симптом | Что проверить |
| --- | --- |
| `source: mac` сразу после `detect offline` | Контейнер `mentorpi-t1` running? Ноды `/person_detect_pi` и `/person_perception` в `ros2 node list`? `t1ctl detect` печатает `detail` при отказе |
| После `t1ctl restart` снова Mac | Так задумано. Режим не сохраняется. Снова `t1ctl detect offline` |
| Нет рамок onboard | `enabled` true? RGB жив (`/aurora/rgb/image_raw`)? Человек в кадре? Журнал: `sudo journalctl -u mentorpi-t1` — строка `yolo person boxes=` |
| Overlay молчит | Сначала `t1ctl debug on`. Нет RGB — overlay не публикуется |
| Overlay есть рамка, `nearest` пустой | Рамки 2D есть, геометрия нет: `ros2 topic hz /aurora/points2`. При `cloud_fresh=0` NCNN забивает CPU — Aurora не отдаёт облако (`points_timeout_ms=700`). As-built: `infer_period_ms=500`, 2 потока |
| Отказ = нет человека | Пустой кадр или детектор молчит → `nearest` invalid, следование стоит. Это не поломка контура |

Лог overlay-контейнера через `docker logs` часто пустой — смотреть `journalctl -u mentorpi-t1`.
