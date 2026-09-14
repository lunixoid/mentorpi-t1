# SD002. Образ, контейнер, два launch

Коротко для оператора. Точка входа: `make provision`. Штатный `MentorPi` не удаляем, `start_node.sh` не правим.

## FROM и create

База — **image**, с которого создан контейнер `MentorPi` (`docker inspect MentorPi` → `Config.Image`). Софт Hiwonder в этом image; тег `ros:humble` с Docker Hub сам по себе не «с играми». Два контейнера делят родительские слои; наш тег `mentorpi-t1` — overlay сверху. `docker commit` не используем (он снимает грязный RW `MentorPi`, логи `.ros` и т.п.).

1. На Pi: `docker build -t mentorpi-t1` из `docker/mentorpi-t1/Dockerfile` (`FROM` image `MentorPi`, `COPY install` и `config` → `/home/ubuntu/mentorpi_t1_ws/`). Контекст — `/home/pi/mentorpi_t1_ws`. Штатный `MentorPi` для этого не останавливаем. Повторный provision не пересобирает image, если тег уже есть (`FORCE_REBUILD=1` — пересборка). Место на диске нужно под тонкий слой overlay, не под второй полный вендор.
2. `docker create --name mentorpi-t1 --network host --privileged --restart no` плюс те же bind, что у `MentorPi` (`/dev`, `/var/lib/dbus`, `/home/pi/docker/tmp` → `/home/ubuntu/shared`, pulse, `/tmp/.X11-unix`), команда `tail -f /dev/null`. Image контейнера — `mentorpi-t1`. Не копировать `restart=always` у `MentorPi`: иначе после boot оба контейнера снова `Up`.
3. `docker start mentorpi-t1`. Обновления кода после этого: `make deploy` → заменить overlay в контейнере, `docker restart mentorpi-t1`, `t1ctl restart` (без пересборки image).
4. Не `docker rm MentorPi`. Второй контейнер с тем же именем не создаём. `FORCE_REBUILD=1` при живом контейнере пересобирает image, но контейнер остаётся на старом слое, пока его не удалить и не provision снова.

Оба контейнера **могут быть created**. Running — только режим: demo → `mentorpi-t1`, stock → `MentorPi`. Не `docker rm MentorPi`.

## Не два launch и не два running контейнера сразу

| Контур | Контейнер | Кто стартует | Launch |
|--------|-----------|--------------|--------|
| Наш | `mentorpi-t1` | `mentorpi-t1.service` | `ros2 launch mentorpi_bringup stage1.launch.py` |
| Hiwonder | `MentorPi` | `start_node.service` → `start_node.sh` | `ros2 launch bringup bringup.launch.py` (игры) |

Одновременно running только один контейнер и active только один unit. `t1ctl start` останавливает `MentorPi`; `t1ctl stock` останавливает `mentorpi-t1`. Provision: `disable --now start_node`, затем `enable --now mentorpi-t1` (`ExecStartPre` стопает `MentorPi`). После boot — наш unit, `start_node` disabled.

## Конфликт с app Hiwonder

Штатные `start_app` / lidar following пишут `Twist` на шасси. На этом стенде драйвер слушает `/hiwonder_controller/cmd_vel` (и `/cmd_vel` для приложений — мы `/cmd_vel` не используем). Наш контур **не должен** работать параллельно с этим графом: два источника команд дерутся за шасси, учебное следование может поехать на ближайшее препятствие.

Не запускать вручную `bringup.launch.py` в `MentorPi`, пока жив наш launch, и наоборот.

## Команды оператора

На хосте Pi (не внутри контейнера):

- `t1ctl` — статус (`demo` / `stock` / `chassis` / `mode` / `remote controller` / `version`)
- `t1ctl start` — demo: `start_node` stop+disable, `docker stop MentorPi`, `mentorpi-t1.service`
- `t1ctl restart` — перезапуск demo (снова только `mentorpi-t1`). Unit `ExecStop` вызывает in-container `t1-stop` (SIGTERM launch, ожидание, SIGINT leftover `ros_robot_controller`, sweep `--ros-args`), чтобы не копить сирот нод
- `t1ctl stock` — stock: demo stop+disable, `docker stop mentorpi-t1`, `docker start MentorPi`, `start_node` enable+start

Режим (`mode follow` / `manual`) меняется кнопкой Select на пульте, не командой `t1ctl`. Связь с пультом в статусе — ключ `remote controller` (`active` / `inactive`), не `remote`. Приёмник 2.4G в USB **Raspberry Pi** — ShanWan `2563:0575` (`USB WirelessGamepad`): `/dev/input/by-id/usb-2563_USB_WirelessGamepad-joystick` (часто `js0`) → `linux_joy` → `/joy` → `pad_teleop`. Не ROS 2 `joy_node` (SDL, параметр `dev` игнорирует). Приёмник в USB-host платы RRC → `/ros_robot_controller/joy` (на этом стенде молчит). Включи пульт, в `t1ctl` дождись `remote controller active`, затем **Select**, затем стик. В follow стик не едет. Пульт в Manual пишет `/vehicle/cmd_vel`; на гусеницы по-прежнему только `platform_adapter` → `/hiwonder_controller/cmd_vel`. Не `/cmd_vel`. Не pygame и не вендорский `joystick_control`.

## F02: робот стоит по умолчанию

В нашем launch есть `control_state`, `pad_teleop`, `linux_joy` (USB Pi, by-id/`js*` → `/joy`), vendor chassis SIT (`ros_robot_controller`, `odom_publisher`) и `platform_adapter`. На моторы уходит `/hiwonder_controller/cmd_vel`; по умолчанию адаптер шлёт нули (нет входа / watchdog / Forbidden), поэтому робот стоит. В AutoFollow `pad_teleop` не пишет `/vehicle/cmd_vel`. В Manual без стика — нули; нет Joy дольше `cmd_freshness_ms` (дефолт 100 мс) — тоже нули; `remote controller` гаснет через 1 с. Режим остаётся `manual`. Диагностика **внутри контейнера** требует `source /home/ubuntu/ros2_ws/.hiwonderrc`, затем overlay `setup.bash`. Без `.hiwonderrc` нет `ROS_DOMAIN_ID=1` и типов `ros_robot_controller_msgs` — `ros2 topic echo` врёт «топик не публикуется». Twist-нули STM32 не останавливают. Overlay RRC при старте шлёт T1 UART init: `set_motor_type(0x01 JGB37)` дважды, `set_battery_level(0x1af4)`, полный `set_motor_speed` zero IDs 1–4 — это зависимость MentorPi_Tank, не JetRover_Mecanum. `t1-stop` после SIGTERM launch ждёт, пока RRC отпустит `/dev/rrc`, затем ту же UART-последовательность (несколько полных zero-кадров). Не открывать второй Board, пока жив RRC. Launch `respawn` поднимает контроллер снова. Критерий стоянки — обе гусеницы физически стоят; `ros2 topic echo` нулей **не** доказательство стопа. Не запускать игры параллельно.

`ros_robot_controller` нужен `pyserial` (`serial`). В RW слое `MentorPi` он уже есть; базовый image его не содержит. Overlay Dockerfile ставит `pip install --user pyserial`. Если контроллер умер с `No module named 'serial'` в уже созданном контейнере: `docker exec -u ubuntu mentorpi-t1 python3 -m pip install --user pyserial`, затем `t1ctl restart` (и `pkill` leftover launch, если ноды задвоились).

`foxglove_bridge` (SD005 viewer) и `imu_complementary_filter` (SD010, нода `complementary_filter_node` для vendor `imu_filter.launch.py`) в базовом image тоже нет. Overlay Dockerfile ставит `ros-humble-foxglove-bridge` и `ros-humble-imu-complementary-filter` (не метапакет `ros-humble-imu-tools`). Перед `apt` переписывает source-листы базового image с `mirrors.tuna.tsinghua.edu.cn` на official (`ports.ubuntu.com`, `packages.ros.org`) — иначе `docker build` падает с 404; и обновляет ROS keyring из `rosdistro` — иначе `apt-get update` падает с `EXPKEYSIG F42ED6FBAB17C654` на `packages.ros.org`. Повторный provision сам пересобирает image, если в нём нет `complementary_filter_node`. После правки Dockerfile: `FORCE_REBUILD=1 make provision` (контейнер пересоздаётся, если image id сменился; см. раздел «FROM и create»). Временный обход без пересборки image: вручную поправить apt в контейнере (те же official URL), обновить keyring (`curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg`), `apt-get update && apt-get install -y ros-humble-foxglove-bridge ros-humble-imu-complementary-filter`, затем `t1ctl viewer start` / `t1ctl restart`.

Overlay RRC — MentorPi_Tank: при старте UART init (JGB37 type ×2, battery 0x1af4, полный zero IDs 1–4). Кинематика odom — tank (знаки stock, не JetRover mecanum invert 3/4). `motor_rps_canon` снят: signed-zero не гарантия стопа. Критерий стоянки физический; topic echo нулей недостаточен.

## demo failed (unit failed)

`mentorpi-t1.service` с `Restart=no`. Падение **не** включает Hiwonder. Робот стоит до ручного вмешательства: `t1ctl start` (наш снова) или `t1ctl stock` (игры). Сам стенд на исходный образ не откатится.
