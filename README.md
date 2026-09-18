# MentorPi T1

Hiwonder MentorPi T1 — гусеничный робот на Raspberry Pi 5. С завода на нём стоит образ Hiwonder с учебными приложениями.

Проект собирает поверх вендорского образа свой стек на ROS 2 Humble и позволяет переключаться между ним и оригинальным. Под своим стеком робот ездит с пульта или сам следует за человеком, учебные приложения при этом не запускаются.

Что лежит в репозитории:

- `src/` — ROS-пакеты: режимы движения, слежение за человеком, работа с датчиками, мост на вендорское шасси;
- `host/t1ctl/` — утилита оператора `t1ctl`: статус робота, переключение стеков и режимов, калибровка датчиков;
- `host/systemd/`, `host/sudoers.d/`, `host/sysctl.d/` — автозапуск и настройки хоста Raspberry Pi;
- `Makefile`, `mk/`, `docker/` — сборка под arm64 и выкладка на робота.

Переключение стеков:

| Стек | Что запускается | Контейнер Docker | Команда |
|------|-----------------|------------------|---------|
| **demo** | сборка из этого репозитория | `mentorpi-t1` | `t1ctl start` |
| **stock** | штатные приложения Hiwonder | `MentorPi` | `t1ctl stock` |

Вместе они не работают: оба шлют команды скорости на шасси, поэтому `t1ctl` при запуске одного останавливает другой. После включения питания робот поднимает demo.

Сама `t1ctl` установлена на Raspberry Pi и запускается **на хосте, а не внутри контейнера**.

## Что робот умеет сейчас

- Ехать по стику USB-пульта — режим **manual**.
- Следовать за человеком — режим **follow**: подъезжает и останавливается примерно в 0.5 м. Скорость тем выше, чем дальше человек, но не больше 0.25 м/с; разгон и торможение плавные.
- Распознавать человека на борту робота — нода `person_detect_pi` поднимается вместе со стеком, см. [Слежение за человеком](#слежение-за-человеком).
- Показывать датчики на 3D-сцене rviz по VNC (ярлык «MentorPi rviz» на столе Pi).
- Хранить и применять калибровку креплений датчиков.

Чего пока нет:

- **Нет защиты от столкновений.** Робот не видит препятствия и не тормозит перед ними — следите за ним сами и держите пульт под рукой.
- В режиме follow робот не сдаёт назад и не разворачивается на месте: если человек оказался сзади или слишком близко, робот просто стоит.
- Нет карты и навигации (Nav2), нет автономных маршрутов.

## Быстрый старт

```bash
ssh pi@192.168.88.56        # Ethernet; по Wi-Fi робота — 192.168.149.1

t1ctl start                 # поднять demo (stock при этом останавливается)
t1ctl status                # проверить, что всё поднялось
```

Пример статуса:

```
demo              active      # стек этого репозитория работает
stock             inactive    # вендорский стек выключен
chassis           active      # шасси отвечает
mode              follow      # текущий режим движения
remote controller active      # пульт на связи
lidar             active
camera            active
imu               active
odometry          active
platform model    active
calibration       file        # применены калибровки из файла
dds buffers       active
version           2.0.0
```

Дальше:

1. Включите пульт и дождитесь в статусе строки `remote controller active`.
2. Нажмите **Select** — робот переключится между follow и manual.
3. В manual ведите стиком. В follow стик не действует.

Если что-то пошло не так: `t1ctl mode forbid` запрещает движение, на шасси идут нули. Вернуть движение — `t1ctl mode allow`.

Юнит `mentorpi-t1.service` сам не перезапускается (`Restart=no`). Если он упал, робот просто стоит и на штатный софт Hiwonder не переключается — поднимите стек вручную (`t1ctl start`) или отдайте робота вендорскому стеку (`t1ctl stock`).

## Режимы движения

Режим один на весь стек, им владеет нода `control_state`. При старте включается **follow**.

| Режим | Кто задаёт скорость | Как включить |
|-------|---------------------|--------------|
| `follow` | нода `motion_control` (слежение за человеком) | Select на пульте или `t1ctl mode allow` |
| `manual` | стик пульта | Select на пульте или `t1ctl mode manual` |
| `forbidden` | никто, на шасси идут нули | `t1ctl mode forbid` |

Из `forbidden` кнопкой Select выйти нельзя — только `t1ctl mode allow`.

Пульт — это приёмник 2.4 ГГц ShanWan (`2563:0575`), воткнутый **в USB-порт Raspberry Pi**. Нода `linux_joy` читает его напрямую из `/dev/input` и публикует `/joy`; кнопка Select там имеет индекс 8. Если воткнуть приёмник в USB-порт платы контроллера RRC, пульт приходит в топик `/ros_robot_controller/joy` (Select — индекс 10); на этой сборке робота он молчит.

В manual робот едет, только пока приходят свежие данные с пульта: нет пакета 100 мс — команда обнуляется, нет связи 1 с — `remote controller` становится `inactive`. Режим при этом не меняется, робот просто стоит.

## Команды `t1ctl`

Все команды выполняются на хосте Pi.

```
t1ctl                  # то же, что t1ctl status
t1ctl start            # включить demo
t1ctl restart          # перезапустить demo (нужен после calib save)
t1ctl stock            # вернуть штатный автозапуск Hiwonder

t1ctl mode forbid      # запретить движение
t1ctl mode allow       # слежение за человеком
t1ctl mode manual      # ручное управление с пульта

t1ctl calib            # калибровка датчиков, см. отдельный раздел

t1ctl --help
t1ctl --version
```

Подкоманды `calib`: `status`, `corner`, `drive`, `side`, `lidar`, `camera`, `accept`, `reject`, `save`, `abort` (подробнее в разделе [Калибровка датчиков](#калибровка-датчиков)).

Поля статуса:

| Поле | Значения | Что означает |
|------|----------|--------------|
| `demo` | `active` / `inactive` / `failed` | состояние `mentorpi-t1.service` |
| `stock` | `inactive` / `active` | состояние вендорского `start_node.service` |
| `chassis` | `active` / `inactive` | контейнер поднят и шасси отвечает в `/vehicle/status` |
| `mode` | `follow` / `manual` / `forbidden` | режим движения; у `forbidden` печатается ещё строка `reason` |
| `remote controller` | `active` / `inactive` | есть ли связь с пультом |
| `lidar`, `camera`, `imu`, `odometry` | `active` / `degraded` / `inactive` | датчик даёт данные и его система координат (TF) на месте |
| `platform model` | `active` / `degraded` / `inactive` | загружена модель робота (URDF) |
| `calibration` | `file` / `factory` / `unused` | откуда взяты позы датчиков: из файла, из модели, либо файл есть, но не применился |
| `dds buffers` | `active` / `degraded` | хватает ли сетевых буферов под поток с камеры |
| `version` | например `2.0.0` | версия `t1ctl` |

## Как устроен стек

### Путь команды скорости

До гусениц команда идёт одним и тем же путём в обоих режимах — меняется только источник:

```
follow: motion_control → /pnc/desired_twist       ┐
                                                  ├→ control_mux
manual: pad_teleop     → /control/manual_cmd_vel  ┘      │
                                                         ↓
                                                  /vehicle/cmd_vel
                                                         ↓
                                                  platform_adapter
                                                         ↓
                                         /hiwonder_controller/cmd_vel
                                                         ↓
                                    вендорский драйвер → UART → гусеницы
```

Правила публикации:

- `/vehicle/cmd_vel` пишет только `control_mux` — он и выбирает источник по текущему режиму.
- На вендорский топик шасси `/hiwonder_controller/cmd_vel` пишет только `platform_adapter`. По умолчанию он шлёт нули: команды нет, команда протухла или движение запрещено.
- Топик `/cmd_vel` не используется вообще.
- Нули в топике — это «не ехать», а не физическая остановка: контроллер шасси останавливается не по нулевой скорости, а по команде инициализации, поэтому штатная остановка стека идёт через скрипт `t1-stop`.

### Пакеты

| Пакет | Роль |
|-------|------|
| `src/mentorpi_bringup` | `stage1.launch.py` — запуск всего графа и слоёв датчиков |
| `src/mentorpi_control` | режим (`control_state`), пульт (`linux_joy`, `pad_teleop`), выбор источника скорости (`control_mux`) |
| `src/mentorpi_platform` | `platform_adapter` — мост на вендорское шасси |
| `src/motion_control` | закон слежения за человеком и объезд по виртуальному бамперу → `/pnc/desired_twist`; охрана `obstacle_guard` → `/control/motion_restriction` |
| `src/mission_control` | статус слежения (`FOLLOWING` / `HOLD` / `INACTIVE`), скорость не считает |
| `src/mentorpi_perception` | детекции + облако точек → положение ближайшего человека |
| `src/mentorpi_person_detect` | распознавание человека на роботе (YOLO11n, включено по умолчанию) |
| `src/mentorpi_localization` | ориентация по IMU → `/imu_odom` |
| `src/mentorpi_description` | модель робота (URDF/xacro) |
| `src/mentorpi_calibration` | файл калибровки и утилита `calib` |
| `src/mentorpi_msgs` | типы сообщений и сервис смены режима |
| `src/hiwonder_controller`, `src/ros_robot_controller` | вендорские драйверы шасси: кинематика гусениц, одометрия, обмен по UART |
| `host/t1ctl` | CLI оператора (C++17, без ROS) |
| `host/systemd`, `host/sudoers.d`, `host/sysctl.d` | автозапуск и настройки хоста Pi |
| `Makefile`, `mk/`, `docker/` | сборка под arm64, выкладка на Pi, первичная настройка |

### Основные топики

| Топик | Тип | Кто публикует |
|-------|-----|---------------|
| `/control/state`, `/control/status` | `mentorpi_msgs/ControlState`, `ControlStatus` | `control_state` (режим и то, что читает `t1ctl`) |
| `/control/manual_cmd_vel` | `geometry_msgs/Twist` | `pad_teleop` (только в manual) |
| `/pnc/desired_twist` | `geometry_msgs/Twist` | `motion_control` |
| `/vehicle/cmd_vel` | `geometry_msgs/Twist` | `control_mux` |
| `/vehicle/status` | `mentorpi_msgs/ChassisStatus` | `platform_adapter` |
| `/hiwonder_controller/cmd_vel` | `geometry_msgs/Twist` | `platform_adapter` |
| `/pnc/follow_person/status` | `mentorpi_msgs/FollowPersonStatus` | `mission_control` |
| `/perception/nearest_person` | `mentorpi_msgs/NearestPerson` | `mentorpi_perception` |
| `/control/motion_restriction` | `mentorpi_msgs/MotionRestriction` | `obstacle_guard` (стоп по предмету, нет цели, нет команды; `reason`) |
| `/pnc/obstacle_avoidance/status` | `mentorpi_msgs/ObstacleAvoidanceStatus` | `motion_control` (свободно / объезд / стоп / нет скана) |
| `/scan`, `/aurora/points2`, `/imu`, `/odom_raw` | данные датчиков | драйверы лидара, камеры, IMU, шасси |

Смена режима — сервис `/control/set_mode`; `t1ctl mode` дёргает именно его.

## Слежение за человеком

Как это работает по шагам:

1. Нода `person_detect_pi` распознаёт человека на RGB-картинке камеры и публикует рамки в `/perception/detections_2d` (YOLO11n на Pi, примерно 2 кадра в секунду).
2. `mentorpi_perception` берёт рамку, сопоставляет её с облаком точек глубинной камеры и считает, где человек относительно робота. Результат — `/perception/nearest_person`.
3. `motion_control` по дистанции и направлению считает скорость и публикует `/pnc/desired_twist`.
4. В режиме follow `control_mux` пропускает эту скорость на шасси.

Детектор включён по умолчанию и стартует вместе с demo; отдельной команды переключения нет.

Если человека не видно, `mentorpi_perception` какое-то время продолжает предсказывать его положение по последней скорости (режим «coasting»). В это время робот не едет — трогается он только по свежим детекциям.

## Просмотр с датчиков

Смотрите датчики на 3D-сцене rviz по VNC:

1. Подключитесь к рабочему столу Pi по VNC.
2. Убедитесь, что demo активен (`t1ctl status` → `demo active`).
3. Дважды щёлкните ярлык **MentorPi rviz** на столе — откроется окно с моделью робота, TF, лидаром `/scan` и облаком `/aurora/points2`.

Если контейнер demo не running, ярлык ничего не откроет — сначала `t1ctl start`. Fixed frame в конфиге — `odom`.

## Калибровка датчиков

### Зачем

Лидар, глубинная камера и IMU закреплены на корпусе, и софт должен знать, где именно они стоят. Пока позы заводские, одна и та же стена в `/scan` и в облаке точек `/aurora/points2` оказывается в разных местах, и робот неправильно считает, где человек.

### Как устроен процесс

Три шага: замер → черновик → файл.

1. Команда замера (`corner`, `drive`, `side`) или ручной ввод (`lidar`, `camera`) кладёт предложение в черновик.
2. `accept` оставляет предложение в черновике, `reject` выбрасывает. Делать это надо **после каждого** шага.
3. `save` записывает черновик в файл, `restart` перезапускает стек.

Позы становятся живыми только после перезапуска: стек читает файл один раз при старте. То есть цепочка всегда такая — **accept → save → restart**. Без `save` и `restart` картинка в rviz не изменится.

### Что нужно перед началом

1. Поднятый demo-режим: `t1ctl start`, в статусе `demo active`.
2. Ровный пол и прямой угол комнаты (две стены под ~90°).
3. Свободно около метра вперёд и место развернуться на месте.

### Команды

```
t1ctl calib                                             # что применено сейчас (то же, что calib status)
t1ctl calib corner [--timeout <сек>]                    # свести камеру с лидаром по углу комнаты, ~5 с
t1ctl calib drive [--timeout <сек>]                     # найти разворот лидара проездом, по умолчанию 60 с
t1ctl calib side --side left|right [--timeout <сек>]    # проверить, не зеркалит ли камера лево/право
t1ctl calib lidar --height <м> --pitch <град> --roll <град>   # ручной замер лидара
t1ctl calib camera --height <м>                         # ручной замер камеры
t1ctl calib accept                                      # оставить предложение в черновике
t1ctl calib reject                                      # выбросить последнее предложение
t1ctl calib save                                        # записать черновик в файл (дальше нужен t1ctl restart)
t1ctl calib abort                                       # удалить черновик целиком, файл не трогает
```

Числа пишутся **без единиц**: `0.18`, а не `0.18m` и не `18`. Высота — в метрах, наклон — в градусах. У `calib lidar` обязательны все три флага.

Порога «откалибровано» нет — решает оператор, глядя на сцену в rviz по VNC.

Сейчас **не** калибруются: оптические оси камеры и лидара, положение IMU (`x`/`y`/`z`/`yaw` остаются заводскими, `drive` только сверяет знак вращения), внутренние параметры камеры, коэффициенты одометрии.

### Обычный порядок работ

```bash
t1ctl calib corner                                   # автоматический замер по углу
t1ctl calib accept

t1ctl calib lidar --height 0.18 --pitch 0 --roll 0   # ручные замеры рулеткой
t1ctl calib accept
t1ctl calib camera --height 0.145
t1ctl calib accept

t1ctl calib save
t1ctl restart
```

Как мерить рулеткой — [docs/SD/SD012/measure.md](docs/SD/SD012/measure.md).

Файл с результатом: `/home/pi/mentorpi_t1_ws/config/platform/t1/sensor_calibration.yaml` на хосте Pi (он же виден внутри контейнера).

### Замер по углу: `calib corner`

Поставьте робота **перед прямым углом** так, чтобы обе стены попадали и в глубинную камеру, и в лидар. Не перед одной плоской стеной. Пока идёт набор данных (~5 с), робот не двигать и пульт не трогать.

Что получится: наклоны (`roll`, `pitch`) камеры и IMU, сведение камеры с лидаром по `x`, `y`, `yaw`, а если в кадр попал пол — ещё и высота камеры. Если пола не видно, команда так и напишет: высоту введите руками через `calib camera --height`.

Отказ `calib corner failed` — файл не меняется. Причины: видна одна стена, стены не под 90°, мало точек, молчит датчик. Переставьте робота и повторите.

### Замер проездом: `calib drive`

Показывает, куда лидар смотрит относительно корпуса (его разворот, `yaw`).

Нужен режим **manual** (`t1ctl mode manual` или Select на пульте) и свободный метр впереди. Ехать надо **сразу** после ввода команды. Пока идёт замер, в кадре ничего не должно двигаться — сами перед лидаром не ходите.

Замер двухфазный, подсказки печатаются в терминале:

1. `drive forward` — ехать **только прямо**, без дуги, примерно 0.5–1 м. Счётчик `turn` пока стоит на нуле. Фаза не закончится, пока не наберётся около 0.3 м хода.
2. Когда подсказка сменится на `now turn in place` — остановиться и **развернуться на месте** на 45–90°.

```bash
t1ctl calib drive --timeout 60
```

В выводе растут счётчики `travel` и `turn`. В конце печатаются поправка лидара по `x`, `y`, `yaw`, ошибка до и после, а также `imu yaw sign` — слово `ok` или `mismatch` (это знак вращения, не угол).

Слишком короткий проезд или отсутствие разворота дадут `travel too short` / `no rotation segment`, черновик при этом не меняется. Если от прошлого прогона остался черновик, сначала сделайте `reject`.

### Проверка лево/право: `calib side`

Проверяет, не перепутаны ли у камеры стороны. Поставьте коробку **явно слева или справа** от робота, так чтобы её видели и лидар, и камера. Робот стоит, пульт не трогаем.

```bash
t1ctl calib side --side left
```

Команда напечатает, с какой стороны предмет по вашим словам, по лидару и по облаку точек, и скопирует кадр на хост Pi в `/tmp/t1ctl-side_frame.png`. Если камера зеркалит, в черновик ляжет поправка `camera_transverse_mirror` — дальше обычные `accept` и `save`.

### Ручной замер рулеткой

Робот стоит неподвижно на ровном полу. Высота — это **вертикаль от пола** (от плоскости гусениц), а не расстояние вдоль оптической оси и не высота от крышки корпуса.

**Лидар.** Мерить до середины тёмного кольца — это плоскость, в которой он сканирует, — а не до крышки или кронштейна. `pitch`: положите уровень на крышку вдоль корпуса, нос выше кормы — плюс. `roll`: то же поперёк, правый борт ниже — плюс. Если скан горизонтальный, оба нуля. Разворот лидара сюда не входит, его даёт `calib drive`.

**Камера.** Мерить до центра переднего окна глубины (ИК), а не до RGB-объектива. Даже если камера задрана вверх, высота всё равно меряется по вертикали от пола.

Значения, снятые на этом роботе 27.08.2026:

| Датчик | До какой точки мерить | Высота | Наклон |
|--------|----------------------|--------|--------|
| лидар LD19 | середина оптического кольца | 0.180 м | плоскость параллельна полу: `pitch 0`, `roll 0` |
| камера Aurora 930 | середина окна глубины | 0.145 м | считает `calib corner` |

### Как понять, что получилось

Откройте rviz по VNC (см. [Просмотр с датчиков](#просмотр-с-датчиков)) и смотрите на две вещи:

- Стена в облаке точек `/aurora/points2` должна лежать **на лучах** `/scan`, а не отдельным слоем рядом.
- Кольцо `/scan` должно быть плоским и на высоте лидара относительно `base_footprint` (около 18 см).

Кадр `depth_camera_link` в TF может выглядеть «заваленным» — так устроена оптическая система координат камеры, перпендикуляр к полу по ней проверять нельзя.

### Если что-то пошло не так

| Симптом | Что делать |
|---------|------------|
| После `accept` в rviz ничего не изменилось | Не было `save` или `restart`. Цепочка: accept → save → restart |
| В статусе `calibration unused` | Файл есть, но не применился. Причину смотрите в `reason` у `t1ctl calib` |
| `error: calib … failed`, стек лежит | `t1ctl start`; файл калибровки при этом не портится |
| `one wall` / `walls not orthogonal` | Робот стоит не перед настоящим прямым углом — переставить и повторить `corner` |
| `imu not still` | Робота трясло во время `corner` — он должен стоять неподвижно |
| `travel too short` / `no rotation segment` / `scene changed` | Проехать длиннее прямо, развернуться после смены подсказки, не ходить перед лидаром |
| `not enough data` сразу при живых датчиках | На Pi старая сборка или датчики недоступны — `make deploy`, потом `t1ctl restart` |
| `no pending` / `no draft` / `no changes` | Нечего принимать или сохранять — сначала должен пройти успешный замер |
| Камера «уехала» за корпус | `reject` или `abort`, не сохранять |
| Хочется начать заново | `t1ctl calib abort` убирает черновик. Чтобы вернуть заводские позы — удалить `sensor_calibration.yaml` и `t1ctl restart` |

## Сборка и выкладка

Собирать надо на машине разработки с **Linux x86_64** (Ubuntu 22.04) и **Docker Engine**, **не на Pi**.

Зависимости хоста (один раз):

```bash
sudo apt install -y sshpass file qemu-user-static binfmt-support cmake python3-pip
python3 -m pip install --user pre-commit
```

`pre-commit` ставится через pip: apt-пакет Ubuntu 22.04 (2.17) не читает манифест хука clang-format из `.pre-commit-config.yaml`. Команда попадает в `~/.local/bin`; если её нет в `PATH`, перелогиньтесь.

Без зарегистрированного qemu для arm64 `make build` откажет до запуска Docker:

```
error: docker cannot run linux/arm64 (no qemu-aarch64 binfmt on this host)
install: sudo apt install qemu-user-static binfmt-support
```

Первый раз — в таком порядке:

```bash
make build       # собрать ROS-пакеты и t1ctl под arm64 в build-arm64/
make deploy      # скопировать на Pi и перезапустить контейнер mentorpi-t1
make provision   # первичная настройка Pi: образ, контейнер, автозапуск
```

При изменениях кода достаточно `make build && make deploy`.

`make` без цели печатает список целей и ничего не собирает:

```
Targets:
  help              this list (default; no docker/colcon)
  env               builder images (overlay + t1ctl)
  env-overlay       image mentorpi-overlay-builder:arm64
  env-t1ctl         image mentorpi-t1ctl-builder:arm64
  build             overlay + t1ctl (linux/arm64) into build-arm64/
  overlay           ROS overlay only
  t1ctl             t1ctl only
  deploy            copy overlay + t1ctl to Pi; restart mentorpi-t1
  provision         Pi image/container + our unit (not docker rm MentorPi)
  clean             remove build-arm64/ (not docker images)
  env-clean         remove builder image tags

Env: PI_HOST PI_PASSWORD PI_STAGING CONTAINER FORCE_REBUILD HTTP_PROXY HTTPS_PROXY
```

Кратко по целям:

- **`make build`** — собирает overlay (пакеты из `src/`) и `t1ctl` под `linux/arm64` в `build-arm64/`. Отдельно: `make overlay`, `make t1ctl`. Сборочные образы создаются целью `make env` и переиспользуются; если хост ходит в сеть через прокси, `HTTP_PROXY` и `HTTPS_PROXY` из окружения передаются в `docker build` этих образов. На Pi нельзя ставить `t1ctl`, собранный нативно на машине разработки — только arm64 из `build-arm64/` (`make deploy` проверяет архитектуру и откажется).
- **`make deploy`** — копирует overlay в `/home/pi/mentorpi_t1_ws/`, ставит `t1ctl` в `/usr/local/bin`, systemd-юнит, sudoers и настройки сети. Если контейнер `mentorpi-t1` уже есть — останавливает стек, подменяет в контейнере каталог `install/` и перезапускает, чтобы ноды взяли новую сборку. Контейнер `MentorPi` не трогает. По умолчанию `PI_HOST=pi@192.168.88.56`, пароль берётся из `PI_PASSWORD`.
- **`make provision`** — первичная настройка или ремонт стенда. Собирает на Pi образ `mentorpi-t1` поверх вендорского, создаёт контейнер с той же сетью, привязками устройств и каталогом калибровок, включает автозапуск `mentorpi-t1.service` и выключает вендорский `start_node.service`. Вендорский контейнер `MentorPi` не удаляет. Если overlay ещё не выложен, сам вызовет `make deploy`. После смены вендорского образа: `FORCE_REBUILD=1 make provision`.

## Локальная сборка для разработки

Собрать и запустить ROS-пакеты на машине с ROS 2 Humble:

```bash
source /opt/ros/humble/setup.bash
colcon build --base-paths src --packages-up-to mentorpi_bringup
source install/setup.bash
ros2 launch mentorpi_bringup stage1.launch.py
```

Собрать `t1ctl` нативно на машине разработки (это не тот бинарник, что идёт на Pi) и прогнать тесты:

```bash
cmake -S host/t1ctl -B host/t1ctl/build-host
cmake --build host/t1ctl/build-host
ctest --test-dir host/t1ctl/build-host
```

## Проверка кода

Проверки локальные, CI нет. Собирать overlay для них не нужно.

```bash
python3 -m pip install --user pre-commit   # не apt: версия 2.17 не подходит
pre-commit install                         # поставить хук на git commit
pre-commit run --all-files
```

Что проверяется: clang-format (стиль Google, 100 колонок) для C++, black / isort / flake8 (длина строки 120) для Python.

Если хук переформатировал файлы или flake8 нашёл ошибку, коммит не создаётся — добавьте изменения в индекс и повторите. Напрямую black, isort, flake8 и clang-format вызывать не надо, только через `pre-commit`.
