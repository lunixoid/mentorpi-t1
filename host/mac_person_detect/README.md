# mac_person_detect

Нативная нода `person_detect` на macOS ARM (Pixi + RoboStack Humble, PyTorch MPS). Не Docker, `t1ctl` на Mac не ставится.

RGB `/aurora/rgb/image_raw` → YOLO11n track (`yolo11n.pt`, ByteTrack `bytetrack.yaml`, `persist=True`) → `/perception/detections_2d` (`vision_msgs/Detection2DArray`). У каждой подтверждённой детекции `person` поле `Detection2D.id` — десятичная строка ByteTrack id (например `"7"`); рамки без id не публикуются. Keypoints и `PersonArray` не публикуются. Инференс на MPS; NMS torchvision на CPU (ограничение Apple Silicon). Пакет `lap` (>=0.5.12) входит в pixi: ByteTrack импортирует `lap`, в окружении нет `pip`, AutoUpdate Ultralytics не сработает.

Цель `make mac-detect` берёт `host/mac_person_detect/yolo11n.pt` если файл есть локально, иначе имя `yolo11n.pt` (Ultralytics подтянет). Pose `yolo11n-pose.pt` автоматически не выбирается.

При старте нода ищет класс `person` в `model.names`; если в весах нет такого имени — `SystemExit` до spin/RGB.

Переменные: `ROS_DOMAIN_ID` (default `1`), `IMAGE_TOPIC` (`/aurora/rgb/image_raw`), `WEIGHTS` (`yolo11n.pt`), `CONFIDENCE_THRESHOLD` (`0.25`).

A/B весы через `WEIGHTS` (default всё равно `yolo11n.pt`):

```bash
WEIGHTS=yolo11n-pose.pt make mac-detect
WEIGHTS=yolo26n.pt make mac-detect
```

Один раз на Mac, если нет `pixi` в PATH:

```bash
curl -fsSL https://pixi.sh/install.sh | bash
```

## Канал Mac↔Pi

Штатный канал perception: Ethernet `192.168.88.56`, если с Pi проходит ICMP до `ethernet_ping_host` (default `192.168.88.57` — Mac в `192.168.88.0/24`). Иначе — точка доступа робота `192.168.149.1`. На AP нода уходит только после `ethernet_ping_fail_threshold` (default `3`) неудач ping подряд; один потерянный echo кабель не снимает. FastDDS на Mac всегда знает оба unicast-peer; при обрыве кабеля `person_detect` не перезапускать — participant тот же, YOLO не перезагружается.

На улице: подключить Mac к точке робота, затем тот же `make mac-detect`. В логе DDS: `FastDDS peers 192.168.88.56:7660 192.168.149.1:7660 multicast 239.255.0.1:7650 domain 1`. В логе ноды — `/perception/dds_peer` и выбранный IPv4.

Если Mac в Ethernet не `192.168.88.57`, в `src/mentorpi_perception/config/person_perception.yaml` выставить `ethernet_ping_host` на фактический IPv4 Mac и задеплоить overlay.

`make deploy` (default `PI_HOST=pi@192.168.88.56`), SSH, `t1ctl` и URL Foxglove этим SD не выбирают адрес: дефолт по-прежнему Ethernet. На точке укажите хост сами (`ssh pi@192.168.149.1`, `PI_HOST=pi@192.168.149.1 make deploy`, URL из `t1ctl viewer status`).

## Прогон с нуля (T2)

Три терминала. Каждый после старта не закрывать, пока не сказано Ctrl+C. `t1ctl` только на Pi, на Mac не ставить.

Камера Aurora в USB Pi. Mac либо в Ethernet `192.168.88.0/24`, либо на точке робота `192.168.149.1` (не чужая сеть и не раздача с телефона).

Вендорский `.hiwonderrc` ставит `ROS_LOCALHOST_ONLY=1`. Юнит overlay после `source .hiwonderrc` должен делать `export ROS_LOCALHOST_ONLY=0` (уже в `host/systemd/mentorpi-t1.service`), иначе RGB на Mac не уйдёт. Юнит на Pi попадает только через `make deploy` (не `scp`).

### Терминал 1 — хост Pi

На Mac, из корня репозитория (нужен уже собранный `build-arm64/`):

```bash
make deploy
ssh pi@192.168.88.56   # на точке робота: ssh pi@192.168.149.1
```

Пароль: `raspberrypi`. Дальше на Pi:

```bash
t1ctl restart
t1ctl status
```

В статусе должно быть `demo active` и `camera active`. Если `camera` не `active` — дальше не идти.

```bash
docker exec -it -u ubuntu mentorpi-t1 bash -lc '
  source /home/ubuntu/ros2_ws/.hiwonderrc
  export ROS_LOCALHOST_ONLY=0
  source /home/ubuntu/mentorpi_t1_ws/install/setup.bash
  exec bash
'
```

В этом shell:

```bash
ros2 topic hz /aurora/rgb/image_raw
```

Должны идти строки Hz. Затем Ctrl+C. Этот shell оставить открытым.

### Терминал 2 — Mac, детектор

Корень репозитория. Других `person_detect` / `make mac-detect` не должно быть.

В логе должна быть строка `FastDDS peers 192.168.88.56:7660 192.168.149.1:7660 multicast 239.255.0.1:7650 domain 1`.

```bash
cd /Users/roman/Develop/personal/mentorpi-t1
make mac-detect
```

Ждать **две** строки в этом терминале:

1. `person_detect domain=1 ... device=mps`
2. `first RGB ...`

Нет второй строки за ~20 с — RGB с Pi на Mac не доходит, echo не запускать.

Этот процесс не гасить.

### Терминал 3 — Mac, echo

```bash
cd /Users/roman/Develop/personal/mentorpi-t1
make echo-detections
```

Встань перед камерой. В echo должны появиться `Detection2DArray` с `class_id: person` и непустым `id`. Отойди из кадра — `detections: []` (не `PersonArray`).

Ctrl+C echo. Детектор в терминале 2 остановить Ctrl+C, когда проверка кончилась.

Процесс детектора в Activity Monitor — darwin, не Docker linux/arm64.
