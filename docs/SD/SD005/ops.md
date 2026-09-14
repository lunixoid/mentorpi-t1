# SD005. Foxglove viewer на Mac

Коротко для разработчика. Подробности контракта данных и bridge — в `docs/SD/SD005/tech.md`. Операторский bringup demo/stock — в `docs/SD/SD002/ops.md`. Lidar bringup и контракт сенсора — в `docs/SD/SD006/tech.md`. Depth-камера Deptrum — в `docs/SD/SD008/tech.md`.

Цель: read-only визуализация TF, одометрии, лидара, модели платформы и depth-камеры T1 demo-контура с Mac **без VNC**. Запасной путь; основной просмотр — ярлык **MentorPi rviz** на столе Pi по VNC (SD032). Управление движением через viewer недоступно и не предусмотрено.

## Что нужно заранее

| Требование | Зачем |
| --- | --- |
| Mac и Pi в одной сети | WebSocket `ws://<PI_HOST>:8765` с Mac до хоста Pi |
| Demo-контур активен | `t1ctl status` → `demo active`, контейнер `mentorpi-t1` running |
| Lidar готов (SD006) | `t1ctl status` → `lidar active`; при `degraded` — диагностика до Foxglove |
| Platform model готова (SD007) | `t1ctl status` → `platform model active`; при `degraded` — диагностика до Foxglove |
| Depth-камера готова (SD008) | `t1ctl status` → `camera active`; при `degraded` — диагностика до Foxglove |
| IMU готов (SD010) | `t1ctl status` → `imu active`, `odometry active`; при `degraded` — диагностика до Foxglove |
| Детекция людей (SD013, SD026, F09 SD017) | **По умолчанию** после питания / `t1ctl restart` — Mac: `make mac-detect`, README `host/mac_person_detect/README.md`, рамки `/perception/detections_2d`. **Offline** на Pi — только по команде: `t1ctl detect offline` или `make pi-detect` (обратно `t1ctl detect mac` / `make pi-detect ARGS=mac`); рамки `/perception/detections_2d_onboard`. Бой: `/perception/persons`, `/perception/nearest_person` — контракт один; overlay — только после `t1ctl debug on` (рамки выбранного источника) |
| `t1ctl` на Pi | lifecycle моста через `t1ctl debug on|off` и печать параметров подключения |
| Foxglove Desktop на Mac | viewer; локальный ROS 2 на Mac **не** нужен |

По умолчанию Pi: `192.168.88.56` (`PI_HOST` для `make deploy`). `t1ctl debug` при включённом мосте печатает актуальный IP из `hostname -I`. Perception Mac↔Pi (SD016) держит оба FastDDS peer (`192.168.88.56` и `192.168.149.1`); URL viewer этим не меняется.

Сеть: контейнер `mentorpi-t1` в `--network host`, bridge слушает `0.0.0.0:8765`. С Mac должен открываться TCP до `:8765` на IP Pi (firewall на Pi обычно не мешает в lab-сети).

## Установка Foxglove Desktop (Mac)

1. Скачать [Foxglove Desktop](https://foxglove.dev/download) для macOS (Apple Silicon или Intel — по машине).
2. Установить как обычное приложение (перетащить в Applications).
3. Запустить Foxglove Desktop. Аккаунт для базового сценария SD005 не обязателен.

Дальше подключение идёт через **Foxglove WebSocket**, не через нативный ROS 2 discovery с Mac.

## Подготовка на Pi

Команды — на **хосте Pi**, не внутри контейнера.

### 1. Поднять demo-контур

Если demo ещё не активен:

```bash
t1ctl start
t1ctl status    # demo active, stock inactive, lidar/camera/platform model active|degraded
```

Viewer работает только в **demo** (`mentorpi-t1` + `stage1.launch.py`). В stock (`MentorPi`, vendor `bringup.launch.py`) bridge SD005 нет, `/odom_raw`, `/scan`, потоки Aurora и контракт TF SD005 не гарантированы.

На стенде T1 при питании Raspberry Pi 5 от Anker A1695 **камера и лидар одновременно не работают** (см. ограничение питания в `docs/SD/SD008/tech.md`). Для проверки камеры в Foxglove отключить лидар или смириться с `lidar degraded`.

### 2. Проверить лидар (SD006)

Перед открытием Foxglove убедиться, что сенсор готов:

```bash
t1ctl status
```

Ожидания для строки `lidar`:

| Значение | Смысл |
| --- | --- |
| `active` (зелёный) | `/scan` публикуется, TF `base_footprint -> lidar_frame` готов |
| `degraded` (красный) | demo поднят, но scan или TF отсутствует |
| `inactive` | контейнер down или probe пропущен |

При `degraded` — проверить логи lidar-слоя:

```bash
docker logs mentorpi-t1 2>&1 | grep lidar_layer
```

Типичные причины: `LIDAR_TYPE` не задан, `hiwonder_peripherals` отсутствует, лидар физически не подключён, для `A1` нет `/dev/lidar` и `/dev/ldlidar`. Demo-контур при этом остаётся живым.

### 3. Проверить модель платформы (SD007)

Перед открытием Foxglove убедиться, что URDF и обязательные кадры готовы:

```bash
t1ctl status
```

Ожидания для строки `platform model`:

| Значение | Смысл |
| --- | --- |
| `active` (зелёный) | `/robot_description` публикуется, TF `base_footprint -> base_link` и сенсорные кадры готовы |
| `degraded` (красный) | demo поднят, но description или один из обязательных TF отсутствует |
| `inactive` | контейнер down или probe пропущен |

При `degraded` — проверить логи model-слоя:

```bash
docker logs mentorpi-t1 2>&1 | grep robot_model_layer
```

Типичные причины: Xacro/URDF не развернулся, `robot_state_publisher` не стартовал, mesh-ресурсы недоступны в install space. Demo-контур при этом остаётся живым.

### 4. Проверить depth-камеру (SD008)

Перед открытием Foxglove убедиться, что камера готова:

```bash
t1ctl status
```

Ожидания для строки `camera`:

| Значение | Смысл |
| --- | --- |
| `active` (зелёный) | цвет, глубина и TF `base_footprint -> depth_camera_link` готовы |
| `degraded` (красный) | demo поднят, но нет цвета и/или глубины и/или TF |
| `inactive` | контейнер down или probe пропущен |

При `degraded`:

```bash
docker logs mentorpi-t1 2>&1 | grep camera_layer
```

Типичные причины: нет USB Aurora (`3251:1930`), `DEPTH_CAMERA_TYPE` не `aurora`, пакет `deptrum-ros-driver-aurora930` отсутствует в overlay image, камера не питается совместно с лидаром. Demo-контур при этом остаётся живым.

Фактические топики стенда T1 (Aurora 930), без заранее заданного whitelist на bridge:

| Поток | Topic | Панель Foxglove |
| --- | --- | --- |
| RGB ~10 Гц (SD014) | `/aurora/rgb/image_raw` | **Image** (если compressed нет в `ros2 topic list`) |
| Цвет compressed | `/aurora/rgb/image_raw/compressed` | **Image**, предпочтительно, **если топик есть в графе** |
| Глубина image | `/aurora/depth/image_raw` | **Image**, если нет облака |
| Глубина cloud | `/aurora/points2` | **3D** PointCloud2, кадр `depth_camera_link` |
| IR | выключен (`ir_enable` false, SD014 T6) | не смотреть |
| Optical TF | `base_footprint -> depth_camera_link` | **3D** TF; цвет дополнительно в `rgb_camera_link` от драйвера |

Детекции людей в **бою** (SD013 / SD014) — не overlay. Смотреть:

| Поток | Topic | Панель Foxglove |
| --- | --- | --- |
| Гипотезы | `/perception/persons` | **Raw Messages** / **Plot** |
| Ближайший | `/perception/nearest_person` | **Raw Messages** |
| Рамки 2D (Mac) | `/perception/detections_2d` | **Raw Messages** (источник по умолчанию) |
| Рамки 2D (offline) | `/perception/detections_2d_onboard` | **Raw Messages** (после `t1ctl detect offline`) |
| Сырой RGB | `/aurora/rgb/image_raw` (или compressed, если есть в графе) | **Image**, как camera 2d |

`/perception/persons/overlay` — **только debug**, после `t1ctl debug on`. Формат **bgr8**, та же панель **Image**, без смены раскладки. По умолчанию топик молчит (после reboot / `t1ctl restart` снова off). Рамки на overlay — из **выбранного** источника (`t1ctl detect status`: `mac` → `/perception/detections_2d`, `offline` → onboard), даже без валидной глубины. Нет RGB — overlay не публикуется. Whitelist `foxglove_bridge` не сужаем. Viewer read-only и не шлёт команды на шасси.

### Offline-детекция на роботе (SD026)

После питания и после `t1ctl restart` источник рамок снова **Mac** (как сброс `t1ctl debug off` для overlay). Offline включается **только** явной командой оператора.

На Pi:

```bash
t1ctl detect          # source: mac|offline
t1ctl detect offline  # NCNN YOLO11n на Pi, Mac-рамки игнорируются
t1ctl detect mac      # обратно на Mac, onboard-инференс выключен
```

С машины разработки (тот же эффект по SSH):

```bash
make pi-detect              # → t1ctl detect offline
make pi-detect ARGS=mac     # → t1ctl detect mac
```

В offline `person_perception` читает только `/perception/detections_2d_onboard`; если Mac всё ещё шлёт `/perception/detections_2d`, они в геометрию не попадают. Гипотезы и nearest — те же топики `/perception/persons` и `/perception/nearest_person`, что в Mac-режиме.

Если onboard не даёт гипотез (пустой кадр или отказ детектора) — `nearest` пустой, робот в режиме следования стоит; для контура это неотличимо от «Mac не в сети». Вымышленных людей нет. Отдельный статус «детектор упал» не вводится.

Перезагрузка Pi или `t1ctl restart` сбрасывает offline → снова Mac. Режим не сохраняется в YAML на диске.

### F09. Сопровождение цели и гистерезис (SD017)

Смена «ближайшего человека» при двух людях в кадре видна по полю `person.track_id` в `/perception/nearest_person` при `valid=true`. Стабильный id приходит с **выбранного** источника рамок: на Mac — `person_detect` с YOLO `track()` + ByteTrack (не старый `predict()` без id в `Detection2D.id`); в offline — `person_detect_pi` с тем же ByteTrack на `/perception/detections_2d_onboard`. Без id в `Detection2D` F09 на стенде не работает как задумано.

Пороги гистерезиса смены цели — параметры ноды `person_perception` (исходные значения в `src/mentorpi_perception/config/person_perception.yaml`):

| Параметр | Default | Смысл |
| --- | --- | --- |
| `switch_margin_m` | 0.30 | Насколько ближе (м) должен быть challenger, чтобы начать смену |
| `target_lost_s` | 2.0 | Сколько секунд держать coast без рамки locked id |
| `challenger_dwell_s` | 2.0 | Непрерывное присутствие challenger с запасом ≥ `switch_margin_m` |

Проверка в контейнере:

```bash
docker exec mentorpi-t1 bash -lc 'source /home/ubuntu/ros2_ws/.hiwonderrc && source /home/ubuntu/mentorpi_t1_ws/install/setup.bash && ros2 topic echo /perception/nearest_person --once'
docker exec mentorpi-t1 bash -lc 'source /home/ubuntu/ros2_ws/.hiwonderrc && source /home/ubuntu/mentorpi_t1_ws/install/setup.bash && ros2 param get /person_perception switch_margin_m'
docker exec mentorpi-t1 bash -lc 'source /home/ubuntu/ros2_ws/.hiwonderrc && source /home/ubuntu/mentorpi_t1_ws/install/setup.bash && ros2 param get /person_perception target_lost_s'
docker exec mentorpi-t1 bash -lc 'source /home/ubuntu/ros2_ws/.hiwonderrc && source /home/ubuntu/mentorpi_t1_ws/install/setup.bash && ros2 param get /person_perception challenger_dwell_s'
```

В echo `/perception/nearest_person` при `valid: true` смотреть `person.track_id` — смена числа означает смену locked-цели. В Foxglove — та же панель **Raw Messages** на `/perception/nearest_person`.

Compressed **не** считается гарантированным as-built: T1 подтвердил raw color. Для цвета в Foxglove — compressed, если топик есть в графе, иначе `/aurora/rgb/image_raw`. Whitelist `foxglove_bridge` не сужаем заранее — если понадобится фильтр, оператор правит `src/mentorpi_bringup/config/foxglove_bridge.yaml` сам.

### 5. Проверить IMU (SD010)

Полный QA слоя, негативы и provision: `docs/SD/SD010/ops.md`. Ниже — только Foxglove после того, как IMU уже `active`.

Перед открытием Foxglove убедиться, что IMU-слой и одометрия готовы:

```bash
t1ctl status
```

Ожидания для строк `imu` и `odometry`:

| Строка | Значение | Смысл |
| --- | --- | --- |
| `imu` | `active` (зелёный) | `/imu`, `/imu_odom` и TF `base_footprint -> imu_link` готовы |
| `imu` | `degraded` (красный) | demo поднят, но нет `/imu`, `/imu_odom` и/или TF `imu_link` |
| `imu` | `inactive` | контейнер down или probe пропущен |
| `odometry` | `active` (зелёный) | `/odom_raw` и TF `odom -> base_footprint` готовы |
| `odometry` | `degraded` (красный) | demo поднят, но нет `/odom_raw` и/или TF odom |
| `odometry` | `inactive` | контейнер down или probe пропущен |

При `imu degraded`:

```bash
docker logs mentorpi-t1 2>&1 | grep imu_layer
```

Типичные причины: `enable_imu:=false`, `hiwonder_peripherals` или `imu_filter` отсутствует, IMU не отдаёт `/ros_robot_controller/imu_raw`, нода `imu_odometry` не стартовала. Demo-контур при этом остаётся живым.

Фактические топики стенда T1 (as-built SD010), без заранее заданного whitelist на bridge:

| Поток | Topic | Панель Foxglove |
| --- | --- | --- |
| IMU filtered | `/imu` | **Plot** или **IMU** — `orientation.x/y/z/w`, `linear_acceleration.x/y/z` |
| IMU odom stream | `/imu_odom` | **Plot** или **Raw Messages** (`nav_msgs/Odometry`); **не 3D** — кадра `imu_odom` в TF нет |
| Pose follow (SD005) | `/odom_raw` | **Plot** или **Raw Messages**; 3D fixed frame `odom` |
| IMU mount TF | `base_footprint -> imu_link` | **3D** TF (RobotModel / sensor frames) |

Проверка в контейнере (сверка с Foxglove):

```bash
docker exec mentorpi-t1 bash -lc 'source /home/ubuntu/ros2_ws/.hiwonderrc && source /home/ubuntu/mentorpi_t1_ws/install/setup.bash && ros2 topic echo --once /imu'
docker exec mentorpi-t1 bash -lc 'source /home/ubuntu/ros2_ws/.hiwonderrc && source /home/ubuntu/mentorpi_t1_ws/install/setup.bash && ros2 topic echo --once /imu_odom'
docker exec mentorpi-t1 bash -lc 'source /home/ubuntu/ros2_ws/.hiwonderrc && source /home/ubuntu/mentorpi_t1_ws/install/setup.bash && ros2 run tf2_ros tf2_echo base_footprint imu_link'
```

У `/imu_odom` publisher `imu_odometry` **не публикует TF** — parent `imu_odom` в дереве TF отсутствует. В 3D-панели этот поток не отобразится по TF; смотреть в **Plot** или **Raw Messages**. Fixed frame 3D по-прежнему `odom`; поза follow — `/odom_raw` + TF `odom -> base_footprint`.

### 6. Bridge для viewer

Мост **не** стартует вместе со стеком (`viewer_bridge:=false` по умолчанию в `stage1.launch.py`). После `t1ctl start`, `t1ctl restart` или reboot моста нет — его включает только оператор:

```bash
t1ctl debug on     # overlay + foxglove_bridge
t1ctl debug off    # гасит overlay и мост; rviz с ярлыка VNC не трогает
t1ctl debug        # статус overlay и bridge; при bridge on — websocket
```

Ожидаемый вывод `t1ctl debug` при включённом мосте:

- `overlay` — `on|off`
- `bridge` — `on|off`
- при `bridge: on` — `websocket: ws://<PI_HOST>:8765`
- `ros domain id` — значение из `/home/ubuntu/ros2_ws/.hiwonderrc` (на стенде обычно `1`)
- `fixed frame` — `odom`
- `robot model` — `/robot_description`
- `odometry` — `/odom_raw`
- `tf` — `odom -> base_footprint -> base_link`
- `sensor frames` — `lidar_frame`, `imu_link`, `depth_cam_frame`
- `lidar scan` — `/scan`
- `lidar tf` — `base_footprint -> lidar_frame`
- `camera 2d` — `/aurora/rgb/image_raw/compressed or /aurora/rgb/image_raw`
- `persons` — `/perception/persons`
- `nearest person` — `/perception/nearest_person`
- `detections 2d` — `/perception/detections_2d`
- `persons overlay` — `/perception/persons/overlay after t1ctl debug on (bgr8)`
- `camera 3d` — `/aurora/points2`
- `camera tf` — `base_footprint -> depth_camera_link`
- `imu` — `/imu`
- `imu odom` — `/imu_odom`
- `imu tf` — `base_footprint -> imu_link`

## Подключение с Mac

1. Foxglove Desktop → **Open connection** (или `Cmd+O`).
2. Тип: **Foxglove WebSocket**.
3. URL: `ws://<PI_HOST>:8765` (из `t1ctl debug` при `bridge: on`, например `ws://192.168.88.56:8765`).
4. Connect.

Если соединение не устанавливается: ping до Pi, на Pi `t1ctl debug` (bridge on, порт слушает), demo не в stock.

### ROS_DOMAIN_ID

Bridge и ROS-граф живут **внутри** контейнера с окружением Hiwonder (`.hiwonderrc`, `ROS_DOMAIN_ID=1` на стенде). Для Foxglove WebSocket на Mac **совпадение `ROS_DOMAIN_ID` на Mac не требуется** — Mac не подключается к DDS напрямую.

`ROS_DOMAIN_ID` в выводе `t1ctl debug` — справочно: тот же domain, что у demo-графа на роботе. Он нужен, если с Mac (или другой машины) запускают **нативные** ROS 2-утилиты в ту же DDS-сеть, что и контейнер; для SD005 это не основной путь.

### Что смотреть в Foxglove (базовый объём BA + SD006 + SD007 + SD008)

После подключения добавить панели (Layout → Add panel). Viewer **read-only**: команды движения и публикация топиков с Mac недоступны.

| Панель | Настройка | Ожидание |
| --- | --- | --- |
| **3D** | Fixed frame: `odom` | Кадр `base_footprint` в дереве TF; поза меняется при движении робота |
| **3D** | RobotModel: `/robot_description` | Узнаваемый корпус T1 и сенсорные links (`lidar_frame`, `imu_link`, `depth_cam_link` / `depth_camera_link`) |
| **3D** | TF enabled | Цепочка `odom -> base_footprint -> base_link` и сенсорные кадры от `base_link` |
| **3D** | Topic: `/scan` (LaserScan) | Точки скана вокруг робота; frame `lidar_frame`, TF `base_footprint -> lidar_frame` |
| **3D** | Topic: `/aurora/points2` (PointCloud2) | Облако глубины в кадре камеры на платформе; frame `depth_camera_link` |
| **Image** | Topic: `/aurora/rgb/image_raw/compressed` или `/aurora/rgb/image_raw` | Цвет с Aurora; compressed только если топик есть в графе |
| **Raw Messages** / **Plot** | Topic: `/perception/persons` | Гипотезы людей (бой) |
| **Raw Messages** | Topic: `/perception/nearest_person` | Ближайший человек (бой) |
| **Raw Messages** | Topic: `/perception/detections_2d` или `/perception/detections_2d_onboard` | Рамки 2D выбранного источника (`t1ctl detect status`) |
| **Image** | Topic: `/aurora/depth/image_raw` | Depth image, если облака нет |
| **Plot** или **Raw Messages** | Topic: `/odom_raw` | `nav_msgs/Odometry`, `header.frame_id=odom`, `child_frame_id=base_footprint` |
| **Plot** или **IMU** | Topic: `/imu` | `sensor_msgs/Imu`: `orientation.x/y/z/w`, `linear_acceleration.x/y/z` меняются при наклоне/движении платформы |
| **Plot** или **Raw Messages** | Topic: `/imu_odom` | `nav_msgs/Odometry`, `header.frame_id=imu_odom`, `child_frame_id=base_footprint`; **не 3D** — TF-кадра `imu_odom` нет |

Движение робота для проверки выполняет оператор (ручной пульт в Manual и т.п.) — не через Foxglove и не через `ros2 topic pub` к cmd_vel.

**Отладка (не прод-раскладка):** после `t1ctl debug on` добавить **Image** на `/perception/persons/overlay` (bgr8, та же панель Image, без смены layout). Пока debug off — overlay молчит; смотреть сырой RGB и топики `/perception/persons`, `/perception/nearest_person`, `/perception/detections_2d`.

Критерий успеха SD005 + SD006 + SD007 + SD008 + SD010:
- в 3D виден TF `odom` → `base_footprint` → `base_link` и сенсорные кадры;
- RobotModel из `/robot_description` показывает корпус и сенсоры в согласованных положениях;
- в `/odom_raw` приходят сообщения с согласованными frame id;
- в 3D отображается `LaserScan` на `/scan` в координатах робота;
- в **Image** виден цвет Aurora (compressed если топик есть, иначе `/aurora/rgb/image_raw`);
- в бою люди видны на `/perception/persons`, `/perception/nearest_person`, `/perception/detections_2d` и сыром RGB; overlay в эту раскладку не входит;
- в 3D облако `/aurora/points2` стоит в `depth_camera_link` на платформе (fixed frame `odom`); если облака нет — depth image `/aurora/depth/image_raw`;
- в **Plot** или **IMU** на `/imu` видны ненулевые `orientation` и `linear_acceleration`; при желании `/imu_odom` в **Plot**/**Raw Messages** (не в 3D).

## Demo vs stock

| | Demo (SD005 + SD006) | Stock (Hiwonder) |
| --- | --- | --- |
| Контейнер | `mentorpi-t1` | `MentorPi` |
| Включение | `t1ctl start` | `t1ctl stock` |
| `foxglove_bridge` | да после `t1ctl debug on` | нет в нашем bringup |
| Одометрия viewer | `/odom_raw` | другой граф, не контракт SD005 |
| Lidar viewer | `/scan`, frame `lidar_frame` | не контракт SD006 |
| Platform model viewer | `/robot_description`, TF `odom` → `base_footprint` → `base_link`, сенсорные кадры | не контракт SD007 |
| Camera viewer | `/aurora/rgb/image_raw` (+ compressed если есть), `/aurora/points2` или `/aurora/depth/image_raw`, TF `depth_camera_link` | не контракт SD008 |
| TF viewer | `odom` → `base_footprint` → `base_link`, `base_footprint` → `lidar_frame`, `imu_link`, `depth_camera_link` | не документировано для SD005 |
| Параллельно с другим контуром | **нельзя** | **нельзя** |

Не запускать demo и stock одновременно. Viewer SD005 рассчитан только на demo.

## Read-only и безопасность

- Bridge: `src/mentorpi_bringup/config/foxglove_bridge.yaml` — без `clientPublish`, пустые whitelist для топиков и сервисов с клиента; capability `assets` включена для mesh-ресурсов RobotModel.
- Viewer не публикует в шасси, не вызывает сервисы движения, не заменяет `t1ctl` / пульт.
- `t1ctl debug off` гасит overlay и `foxglove_bridge`; demo-контур, `platform_adapter` и окно rviz с ярлыка VNC продолжают работать.

## Расширение на другие топики (позже)

Базовый сценарий BA + SD006 + SD007 + SD008 — TF + `/odom_raw` + `/scan` + `/robot_description` + цвет/глубина Aurora. IR штатно выключен (SD014). Дополнительные топики (IMU, кастомные msg) — **конфигурационно**, без смены Mac-workflow:

1. Убедиться, что топик публикуется в demo-контуре (`ros2 topic list` внутри контейнера с `source .hiwonderrc` и overlay `setup.bash`).
2. При необходимости расширить параметры `foxglove_bridge` (фильтры топиков, capabilities) в `config/foxglove_bridge.yaml` и перезапустить bridge: `t1ctl debug off` → `t1ctl debug on` (или `t1ctl restart` для полного demo — мост снова off). Whitelist с клиента по-прежнему пустой (read-only); сужение **server-side** topic list — только если оператор явно выберет набор.
3. В Foxglove Desktop добавить панели под новые топики; layout можно сохранить локально на Mac.

## Диагностика

| Симптом | Проверка |
| --- | --- |
| `demo contour inactive` | `t1ctl start` |
| `lidar degraded` | `t1ctl status` (строка `lidar`); `docker logs mentorpi-t1 2>&1 \| grep lidar_layer` |
| `platform model degraded` | `t1ctl status` (строка `platform model`); `docker logs mentorpi-t1 2>&1 \| grep robot_model_layer` |
| `camera degraded` | `t1ctl status` (строка `camera`); `docker logs mentorpi-t1 2>&1 \| grep camera_layer` |
| `imu degraded` | `t1ctl status` (строки `imu`, `odometry`); `docker logs mentorpi-t1 2>&1 \| grep imu_layer` |
| `odometry degraded` | `t1ctl status` (строка `odometry`); в контейнере `ros2 topic echo --once /odom_raw`, `tf2_echo odom base_footprint` |
| `lidar inactive` | `t1ctl start`; контейнер `mentorpi-t1` running |
| Нет `/scan` | В контейнере: `ros2 topic echo --once /scan`; для `A1` на T1: `ls -l /dev/ldlidar`, `ros2 node list \| grep LD19` |
| Нет TF lidar | В контейнере: `ros2 run tf2_ros tf2_echo base_footprint lidar_frame`; при `platform model active` — TF из URDF, иначе `lidar_frame_tf_fallback` |
| Нет цвета | В контейнере: `ros2 topic list`; echo `/aurora/rgb/image_raw/compressed` если топик есть, иначе `/aurora/rgb/image_raw` |
| Нет глубины | В контейнере: echo `/aurora/points2` или `/aurora/depth/image_raw` |
| Нет TF camera | В контейнере: `ros2 run tf2_ros tf2_echo base_footprint depth_camera_link`; при выключенной модели — `depth_cam_frame_tf_fallback` |
| Нет `/robot_description` | В контейнере: `ros2 topic echo --once --qos-durability transient_local --qos-reliability reliable /robot_description`; проверить `robot_state_publisher` и `robot_model_layer` |
| RobotModel пустой / без mesh | `t1ctl status` → `platform model`; в контейнере проверить `ros2 pkg prefix mentorpi_description`; bridge capability `assets` должна быть включена (см. `foxglove_bridge.yaml`) |
| Нет TF model base | В контейнере: `ros2 run tf2_ros tf2_echo base_footprint base_link`; проверить `robot_state_publisher` |
| Нет TF imu / depth (модель) | В контейнере: `ros2 run tf2_ros tf2_echo base_footprint imu_link` и `... depth_camera_link` (optical; mount `depth_cam_link`) |
| `viewer bridge inactive` | `t1ctl debug on`; в контейнере `ros2 node list \| grep foxglove_bridge` |
| `viewer bridge start failed` | `t1ctl debug on` печатает stderr launch (последние строки лога `ros2 launch`); типично: `package 'foxglove_bridge' not found` |
| `package 'foxglove_bridge' not found` / нет executable | В образе нет `ros-humble-foxglove-bridge`: `FORCE_REBUILD=1 make provision` и пересоздать `mentorpi-t1`, либо временно поправить apt на official зеркала и `docker exec -u root mentorpi-t1 apt-get update && apt-get install -y ros-humble-foxglove-bridge` |
| `docker build` / `apt-get` 404 на `mirrors.tuna.tsinghua.edu.cn` | Overlay Dockerfile должен переписать apt source-листы перед `apt-get update`; пересобрать image: `FORCE_REBUILD=1 make provision` (см. `docs/SD/SD002/ops.md`) |
| `docker build` / `EXPKEYSIG F42ED6FBAB17C654` на `packages.ros.org` | Overlay Dockerfile обновляет ROS keyring перед `apt-get update`; пересобрать image: `FORCE_REBUILD=1 make provision` (см. `docs/SD/SD002/ops.md`) |
| Mac не коннектится | URL из `t1ctl debug` при `bridge: on`, ping, на Pi `ss -ltn \| grep 8765` |
| Нет TF / пустой 3D | В контейнере: `ros2 run tf2_ros tf2_echo odom base_footprint` |
| Нет `/odom_raw` | В контейнере: `ros2 topic echo --once /odom_raw` |
| LaserScan не виден в 3D | Fixed frame `odom`; topic `/scan`; проверить `lidar active` в `t1ctl status` |
| Цвет не виден в Image | Topic compressed если есть в `ros2 topic list`, иначе `/aurora/rgb/image_raw`; `camera active` в `t1ctl status` |
| Нет гипотез людей | `t1ctl detect` — какой источник? В **mac**: echo `/perception/detections_2d`, `make mac-detect` на Mac? В **offline**: `t1ctl detect offline` не сброшен restart'ом? echo `/perception/detections_2d_onboard`; в контейнере `ros2 param get /person_detect_pi enabled` → `true`. Бой: echo `/perception/persons`, `/perception/nearest_person` |
| `track_id` мерцает / nearest прыгает между людьми | Источник mac: `person_detect` с ByteTrack (`track()`, не `predict()`), echo `/perception/detections_2d` — непустой `id`. Источник offline: echo `/perception/detections_2d_onboard` — непустой `id`. На Pi echo `/perception/nearest_person` — `person.track_id`; параметры гистерезиса: `ros2 param get /person_perception switch_margin_m` и др. (см. F09 SD017 выше) |
| Нет рамок на overlay | Сначала `t1ctl debug on` (иначе overlay молчит). RGB жив? `t1ctl detect status`: в **mac** — `make mac-detect` и echo `/perception/detections_2d`; в **offline** — `person_detect_pi` enabled и echo `/perception/detections_2d_onboard`. Нет RGB — overlay молчит |
| Облако не видно в 3D | Fixed frame `odom`; topic `/aurora/points2`; TF `base_footprint -> depth_camera_link`; иначе Image `/aurora/depth/image_raw` |
| RobotModel не виден в 3D | Fixed frame `odom`; topic `/robot_description`; проверить `platform model active` в `t1ctl status` |
| Нет `/imu` | В контейнере: `ros2 topic echo --once /imu`; `imu active` в `t1ctl status`; если `[imu_layer] degraded` — в образе нет `ros-humble-imu-complementary-filter`: `make provision` (пересоберёт image) |
| Нет orientation/accel в Foxglove | Plot или IMU panel на `/imu`; поля `orientation.x/y/z/w`, `linear_acceleration.x/y/z`; сверить с `ros2 topic echo /imu` |
| Нет `/imu_odom` | В контейнере: `ros2 topic echo --once /imu_odom`; проверить `imu active` (нужны и `/imu`, и `/imu_odom`) |
| `/imu_odom` не виден в 3D | Ожидаемо: нода не публикует TF `imu_odom`; смотреть Plot/Raw Messages, не 3D Odometry |
| Включён stock | `t1ctl start` для возврата в demo |

Внутри контейнера для `ros2` всегда: `source /home/ubuntu/ros2_ws/.hiwonderrc` и `source /home/ubuntu/mentorpi_t1_ws/install/setup.bash` (как в `docs/SD/SD002/ops.md`).

## Ссылки

- Техдизайн, контракт T1/T2/T3: `docs/SD/SD005/tech.md`
- Lidar bringup, контракт сенсора, финальный QA: `docs/SD/SD006/tech.md`
- Platform model, URDF/TF контракт: `docs/SD/SD007/tech.md`
- Depth-камера Deptrum, as-built топики/кадры: `docs/SD/SD008/tech.md`
- IMU bringup, `/imu_odom`, status: `docs/SD/SD010/tech.md`
- Детекции людей: `docs/SD/SD013/tech.md`
- Offline-детекция на Pi: `docs/SD/SD026/tech.md`
- Overlay debug vs прод-топики: `docs/SD/SD014/tech.md`
- Сопровождение цели, гистерезис: `docs/SD/SD017/tech.md`
- Bringup demo/stock: `docs/SD/SD002/ops.md`
- Реализация lifecycle моста в `t1ctl debug`: `host/t1ctl/src/viewer.cpp`, `host/t1ctl/src/ui.cpp`
