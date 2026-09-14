# SD010. IMU и одометрия гусениц — инструкция оператора

Фича **F04**: в demo-контуре живой `/imu`, поток `/imu_odom` (заготовка F15), строки `t1ctl status` (`imu` / `odometry`) и сценарий Foxglove. Поза follow по-прежнему `/odom_raw` + TF `odom → base_footprint`. EKF `/odom` **не** поднимаем.

Контракт и as-built: [`tech.md`](tech.md). Viewer-панели: [`../SD005/ops.md`](../SD005/ops.md) §5.

**Не публиковать** `ros2 topic pub` в `/cmd_vel` и `/hiwonder_controller/cmd_vel`. Не запускать vendor `bringup.launch.py` параллельно с нашим контуром. Не `docker rm MentorPi`.

## Доступ

| Что | Как |
| --- | --- |
| Pi | `pi@192.168.88.56`, пароль `raspberrypi` |
| CLI | `t1ctl` **на хосте Pi**, не в контейнере |
| ROS 2 | контейнер `mentorpi-t1`, всегда `bash` и `.hiwonderrc` (иначе пустой `ros2 topic list`) |
| Viewer | Foxglove Desktop на Mac → `ws://192.168.88.56:8765` |

Обёртка для ROS внутри контейнера (повторять перед каждой `ros2`-командой или зайти в интерактивный `bash`):

```bash
docker exec -it -u ubuntu mentorpi-t1 bash -lc '
  source /home/ubuntu/ros2_ws/.hiwonderrc
  source /home/ubuntu/mentorpi_t1_ws/install/setup.bash
  exec bash
'
```

## 1. Выкладка

С **Mac** (не собирать overlay на Pi):

```bash
make build
make deploy
make provision
```

`make provision` нужен, если в overlay-образе ещё нет apt-пакета `ros-humble-imu-complementary-filter`. Цель сама пересоберёт image, если нет

`/opt/ros/humble/lib/imu_complementary_filter/complementary_filter_node`.

После правки `docker/mentorpi-t1/Dockerfile`:

```bash
FORCE_REBUILD=1 make provision
```

Проверка, что фильтр — **apt**, не ручная сборка в `third_party_ws`:

```bash
ssh pi@192.168.88.56
docker exec mentorpi-t1 bash -lc 'dpkg -l ros-humble-imu-complementary-filter; ls -l /opt/ros/humble/lib/imu_complementary_filter/complementary_filter_node'
```

Код-only после первого provision: `make build` + `make deploy`, без provision.

Локально на Mac (парсер статуса, без стенда):

```bash
cmake -S host/t1ctl -B host/t1ctl/build-host && cmake --build host/t1ctl/build-host
./host/t1ctl/build-host/t1ctl_test
# ожидается: ok
```

Не копировать Mac-бинарь `t1ctl` на Pi.

## 2. Поднять demo

На **хосте Pi**:

```bash
t1ctl start
t1ctl status
```

Ожидание:

```
demo              active
stock             inactive
chassis           active
lidar             active
camera            active
imu               active
odometry          active
platform model    degraded
version           1.3.0
```

`platform model: degraded` — **не SD010** (probe ждёт кадр `depth_cam_frame`, в URDF его нет). Для этой фичи не блокер.

В логах контейнера без падения `stage1`:

```bash
docker logs mentorpi-t1 2>&1 | grep imu_layer
```

Ожидание: `[imu_layer] … imu TF from URDF … fallback suppressed` (или fallback, если модель выключена). Не должно быть abort launch.

## 3. T1 — слой IMU и TF `imu_link`

В контейнере (после `source` из §доступ):

```bash
ros2 topic echo --once /imu
```

Ожидание: `sensor_msgs/msg/Imu`, `header.frame_id: imu_link`, ненулевая `orientation` после прогрева фильтра. Publisher — нода `imu_filter`.

```bash
ros2 run tf2_ros tf2_echo base_footprint imu_link
```

Ожидание: Translation ≈ `[0.005, 0.011, 0.121]`, RPY (deg) ≈ `[180, 0, -90]`. Источник **один**: при дефолтном `enable_robot_model:=true` ноды `imu_link_tf_fallback` **нет**.

```bash
ros2 node list | grep -E 'imu|ekf|robot_state'
ros2 topic list | grep -E '/imu|/odom'
```

Ожидание: есть `/imu_calib`, `/imu_filter`, `/robot_state_publisher`; **нет** `ekf_filter_node`; топики `/imu`, `/imu_corrected`, `/ros_robot_controller/imu_raw`; **нет** `/odom` (fusion). `/odom_raw` есть.

Цепочка as-built: `/ros_robot_controller/imu_raw` → `imu_calib` → `/imu_corrected` → `imu_filter` → `/imu`. Overlay remap не нужен.

### Негатив T1: `enable_imu:=false`

Unit `t1ctl` не принимает launch-аргументы. Останавливаем unit (он вызывает `t1-stop` — иначе останутся сироты нод), затем ручной launch.

Сессия A (хост Pi):

```bash
sudo systemctl stop mentorpi-t1.service
docker exec -it -u ubuntu mentorpi-t1 bash -lc '
  source /home/ubuntu/ros2_ws/.hiwonderrc
  export MACHINE_TYPE=MentorPi_Tank
  export DEPTH_CAMERA_TYPE=aurora
  source /home/ubuntu/mentorpi_t1_ws/install/setup.bash
  exec ros2 launch mentorpi_bringup stage1.launch.py enable_imu:=false
'
```

Сессия B:

```bash
docker logs mentorpi-t1 2>&1 | grep imu_layer
# [imu_layer] degraded — enable_imu:=false — vendor filter skipped

# в контейнере:
ros2 topic list | grep imu
# есть /ros_robot_controller/imu_raw; нет /imu и /imu_corrected
ros2 node list | grep imu
# нет imu_calib / imu_filter; imu_odometry может быть жива и молчать
```

Контур (RRC, `/odom_raw`) жив, вымышленного IMU нет. `demo` у `t1ctl` может быть `inactive` — unit остановлен; смотри логи слоя и топики.

Вернуть штатный контур:

```bash
# в сессии A: Ctrl+C launch
t1ctl start
```

### Негатив T1: `enable_robot_model:=false`

Тот же приём, аргумент `enable_robot_model:=false`.

Ожидание: лог `[imu_layer] imu_link_tf_fallback active`; нода `/imu_link_tf_fallback`; **нет** `robot_state_publisher`; `/imu` жив; TF `base_footprint → imu_link` есть; `/odom_raw` и TF `odom → base_footprint` живы; нет `ekf_filter_node` и нет `/odom`.

Снова `Ctrl+C` и `t1ctl start`.

## 4. T2 — `/imu_odom`

На штатном контуре (`t1ctl start`):

```bash
ros2 topic echo --once /imu_odom
ros2 topic hz /imu_odom
```

Ожидание:

| Поле | Значение |
| --- | --- |
| тип | `nav_msgs/msg/Odometry` |
| publisher | `/imu_odometry` |
| `header.frame_id` | `imu_odom` |
| `child_frame_id` | `base_footprint` |
| `pose.pose.position` | `(0, 0, 0)` — ускорение в позицию **не** интегрируется |
| `twist.linear` | нули |
| `orientation` / `twist.angular` | ненулевые после прогрева |
| частота | ~50 Hz |

TF от этой ноды быть не должно:

```bash
ros2 run tf2_ros tf2_echo imu_odom base_footprint
# Invalid frame ID "imu_odom" … frame does not exist
```

При `enable_imu:=false` (как в негативе T1): нода `/imu_odometry` жива, `/imu_odom` не заполняется фейковыми данными.

`/odom_raw` остаётся источником позы follow. Follow и `platform_adapter` `/imu_odom` не читают.

## 5. T3 — `t1ctl status`

На живом demo:

```bash
t1ctl status
```

| Строка | `active` если | иначе при живом контейнере |
| --- | --- | --- |
| `imu` | `/imu` + `/imu_odom` + TF `base_footprint → imu_link` | `degraded` + причина `(no /imu)` / `(no /imu_odom)` / `(no imu_link TF)` |
| `odometry` | `/odom_raw` + TF `odom → base_footprint` | `degraded` + `(no /odom_raw)` / `(no odom TF)` |
| обе | контейнер down | `inactive` |

Негатив: ручной `enable_imu:=false` → `imu: degraded (no /imu)` (и обычно нет `/imu_odom`); `odometry: active`.

Контейнер down (`docker stop mentorpi-t1`) → `imu` и `odometry` `inactive`. Потом `docker start mentorpi-t1` и `t1ctl start`.

`lidar` / `camera` не должны сломаться. `t1ctl_test` на Mac — `ok` (§1).

## 6. T4 — Foxglove

```bash
t1ctl viewer start    # если bridge ещё не поднят
t1ctl viewer status
```

В выводе должны быть ключи:

```
imu               /imu
imu odom          /imu_odom
imu tf            base_footprint -> imu_link
```

и в блоке **FOXGLOVE DESKTOP**:

- Plot or IMU: `/imu` — `orientation.x/y/z/w`, `linear_acceleration.x/y/z`
- Plot/Raw: `/imu_odom` (**не 3D** — кадра `imu_odom` в TF нет)
- 3D fixed frame по-прежнему `odom`; `/odom_raw` как в SD005

На Mac: Foxglove → Open connection → Foxglove WebSocket → `ws://192.168.88.56:8765`. Read-only: из Foxglove не слать команды на шасси.

| Панель | Топик | Что увидеть |
| --- | --- | --- |
| Plot / IMU | `/imu` | ориентация и `linear_acceleration` меняются при наклоне платформы |
| Plot / Raw Messages | `/imu_odom` | Odometry, нулевая позиция |
| 3D | TF | `base_footprint → imu_link` (RobotModel) |

Если блока FOXGLOVE DESKTOP нет, а `websocket` в статусе пустой: в контейнере нет `ss` (`iproute2`) — probe порта врёт 0. Временный обход: `docker exec -u root mentorpi-t1 apt-get update && apt-get install -y iproute2` (в overlay-образ **не** зафиксировано). Bridge при этом может уже слушать `:8765`.

Сверка с ROS: `ros2 topic echo --once /imu` — те же поля, что на графике.

## 7. T5 — документы

Без стенда, сверка с тем, что увидели выше:

- as-built в [`tech.md`](tech.md) совпадает с `ros2 topic list` / `tf2_echo` / нодами
- [`../SD001/tech.md`](../SD001/tech.md): **F04** отмечена `[x]`, ссылка на SD010
- явно: `/odom` (EKF) и продуктовый follow на IMU — **F15**

## 8. Что эта фича не делает

- нет оценки bias, GNSS, карты
- нет EKF и топика `/odom`
- `/imu_odom` не используется follow / `platform_adapter`
- нет нового publisher на шасси

## 9. Неисправности

| Симптом | Что смотреть |
| --- | --- |
| `imu: degraded (no /imu)` при живом `ros2 topic echo /imu` | Probe 1.2.1 ждал 2 с — ложный негатив после restart. Накатить t1ctl **1.3.0**. Пока: подождать 5 с и `t1ctl status` ещё раз |
| `imu: degraded`, топика `/imu` нет | `journalctl -u mentorpi-t1 \| grep imu_layer`; `dpkg -l ros-humble-imu-complementary-filter`; `make provision` |
| слой degraded, нет `need_compile` | забыли `source .hiwonderrc` |
| пустой `ros2 topic list` | то же: нет `ROS_DOMAIN_ID=1` |
| дубли нод после негатива | запуск без `systemctl stop` / `t1-stop`; стоп + `t1ctl start` |
| `/imu_odom` не виден в 3D | ожидаемо: смотреть Plot/Raw |
| `platform model: degraded` | вне SD010 (`depth_cam_frame`) |
| нет блока FOXGLOVE DESKTOP | нет `ss`/`iproute2` в контейнере |
| zombie `foxglove_bridge` | `sudo systemctl restart mentorpi-t1.service` |

## 10. Версии (стенд QA)

`mentorpi_bringup` **0.1.3**, `mentorpi_localization` **0.1.0**, `t1ctl` **1.3.0**.
