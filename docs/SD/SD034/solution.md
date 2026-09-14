# SD034. Миграция разработки на Linux

## Проблема
1. Сборка и деплой (`Makefile`, `mk/*.sh`) жёстко завязаны на macOS: комментарий «GNU Make 3.81 compatible (macOS)», `PATH="/opt/homebrew/bin:/usr/local/bin:${PATH}"`, ожидание `sshpass` именно в `/opt/homebrew/bin`, обходы под «Cursor/macOS» для `DISPLAY`/ssh-agent. Разработка теперь ведётся с Linux-машины (x86_64 Ubuntu), и эти скрипты в текущем виде не отрабатывают: `sshpass` не установлен, Homebrew-путей нет, эмуляция `docker run --platform linux/arm64` не поднята (`exec /bin/uname: exec format error`).
2. Распознавание человека по умолчанию выполняется на отдельном Mac (`host/mac_person_detect`, `make mac-detect`: Pixi + RoboStack Humble + PyTorch MPS). Ноды `person_perception` и `motion_control` по умолчанию ждут `detections_source=mac`, `t1ctl detect mac` — команда по умолчанию, источник сбрасывается на `mac` при каждом `t1ctl restart`. На Linux-машине этот путь неприменим (MPS — бэкенд только Apple Silicon), отдельный Mac рядом с роботом больше не планируется.
3. Сегодняшний обход: разработчик держит Mac специально ради `make mac-detect`, либо каждый раз после `t1ctl restart` вручную переключает источник командой `t1ctl detect offline`.
4. Запасной просмотр датчиков — мост Foxglove (`t1ctl debug on`, порт 8765) — задокументирован и задуман как сценарий именно для Mac («Запасной сценарий (Foxglove на Mac)»); вместе с оверлеем рамок людей (`/perception/persons/overlay`), который по коду существует специально для показа в Foxglove. Основной просмотр (rviz по VNC) от Mac не зависит и его не показывает. Отдельного Mac для этого сценария больше не будет, а подключаться к мосту с Linux-хоста разработчика не планируется.

## Решение
Полностью убрать macOS-специфичный путь: удалить `host/mac_person_detect` и все связанные с ним цели/скрипты/тесты, убрать саму концепцию выбора источника детекции — onboard-распознавание (YOLO11n на Pi 5) остаётся единственным источником и включается автоматически сразу при старте стека. Вместе с ним убрать мост Foxglove и оверлей рамок людей (`t1ctl debug`, порт 8765, `/perception/persons/overlay`) — запасной сценарий просмотра, который существовал ради Mac. Сборочные и деплойные скрипты (`Makefile`, `mk/*.sh`, `README.md`, `AGENTS.md`) переписать под Linux-хост разработчика (без Homebrew-путей и macOS-обходов), сохранив целью деплоя тот же Raspberry Pi 5 (arm64, Docker-контейнер `mentorpi-t1`) без изменений в самом образе робота, кроме удаления mac-веток и Foxglove-моста.

## Сценарии

### Позитивный
1. Разработчик на чистой Linux-машине (x86_64 Ubuntu) клонирует репозиторий и по README/AGENTS.md ставит зависимости хоста: `sshpass`, эмуляцию `linux/arm64` в Docker (binfmt).
2. Выполняет `make build` — overlay и `t1ctl` под `linux/arm64` собираются в Docker (через binfmt-эмуляцию), без обращений к путям Homebrew.
   1. Скрипты находят `sshpass` в обычном `PATH` Linux, а не только в `/opt/homebrew/bin`.
3. Выполняет `make deploy` — overlay и `t1ctl` копируются на Pi, `mentorpi-t1` перезапускается.
4. После `t1ctl start`/`t1ctl restart` стек поднимается в `follow`, onboard-детекция человека (YOLO11n на Pi 5) включена автоматически — без ручного `t1ctl detect offline` и без необходимости держать рядом Mac.

### Негативный №1
1. Пункт 1 позитивного сценария (чистая Linux-машина).
2. Разработчик не установил (или пропустил) binfmt-эмуляцию `linux/arm64`.
   1. `make build`/`docker run --platform linux/arm64 …` падает с понятной ошибкой эмуляции архитектуры, а не тихо собирает бинарник под неверную архитектуру; README называет команду установки binfmt.

### Негативный №2
1. Разработчик по привычке запускает `make mac-detect`, `make pi-detect`, `t1ctl detect mac` или `t1ctl debug on`.
2. Цель/подкоманда отсутствует.
   1. `make` (без аргументов) печатает список актуальных целей без `mac-detect` и `pi-detect`; `t1ctl --help` не содержит подкоманд `detect` и `debug`, портов 8765 и упоминаний `mac`/Foxglove. Основной просмотр датчиков (rviz по VNC) при этом работает как раньше.

## Функциональные требования
- Должна быть возможность собрать overlay + `t1ctl` (`make build`) и задеплоить на Pi (`make deploy`, `make provision`) с Linux-хоста разработчика (x86_64) без обращения к путям Homebrew и без macOS-специфичных обходов в скриптах.
- Репозиторий не должен содержать кода, скриптов и документации, реализующих распознавание человека на Mac (`host/mac_person_detect`, цель `make mac-detect`, `t1ctl detect mac`, ветку `mac` в `detections_source`).
- Onboard-распознавание человека (YOLO11n на Pi 5) должно быть единственным источником детекций и включаться автоматически при старте стека (`t1ctl start` / `t1ctl restart`), без отдельной команды оператора.
- `t1ctl`, `Makefile` и `mk/*.sh` должны сохранять целью деплоя Raspberry Pi 5 (`linux/arm64`, контейнер `mentorpi-t1`) без изменений в образе робота, кроме удаления mac-веток.
- Все цели `make`, не связанные с выбором источника детекции, должны без изменения поведения работать с Linux-хоста разработчика: `help`, `env`, `env-overlay`, `env-t1ctl`, `build`, `overlay`, `t1ctl`, `deploy`, `provision`, `clean`, `env-clean`.
- Цели и команды, существовавшие только ради выбора/переключения источника детекции, удаляются вместе с самой концепцией источника (раз путь инференса один и он включается автоматически): `make mac-detect`, `make pi-detect` целиком (не только ветка `ARGS=mac` — раз выбирать больше не из чего, сама цель теряет смысл), подкоманда `t1ctl detect` со всеми её вариантами (`mac`, `offline`, `status`). Цель `echo-detections` сейчас целиком реализована поверх `host/mac_person_detect` (pixi-окружение, DDS-профиль для канала Mac↔Pi) и теряет свою цель вместе с переносом детекции onboard — удаляется как mac-related; если нужен генерический способ посмотреть `/perception/detections_2d` с Linux-хоста, это отдельное требование, не восстановление этой цели.
- `README.md` и `AGENTS.md` должны описывать только актуальный путь «Linux-хост разработчика → Raspberry Pi 5», без инструкций для Mac.
- Вместе с источником `mac` из `person_perception` и `motion_control` должен уйти весь механизм, который существовал только ради него: выбор DDS-пира для Mac (`ethernet_ping_host`, `ethernet_ping_period_s`, `ethernet_ping_fail_threshold`, `ethernet_ip`, `wifi_ap_ip`, топик `/perception/dds_peer`, фоновый ping-поток в `person_perception`), отдельный вход «внешних» детекций (`detections_topic=/perception/detections_2d`, обработчик `on_mac_detections`) и парный mac-тайм-аут в обоих узлах (`points_timeout_ms` в `person_perception`, `nearest_timeout_ms` в `motion_control` схлопываются в один тайм-аут каждый — офлайн-значение остаётся, ветвление по источнику уходит).
- `src/mentorpi_person_detect/config/person_detect_pi.yaml`: `enabled: false` меняется на включено по умолчанию — это и есть механизм автовключения onboard-детекции при старте стека (см. выше).
- Мост Foxglove и оверлей рамок людей удаляются целиком, а не только «Mac» в названии: `host/t1ctl/src/viewer.{cpp,hpp}`, подкоманда `t1ctl debug` (`on`/`off`/`status`) и её код в `main.cpp`/`ui.cpp`/`units.{cpp,hpp}` (`DebugCommand`, `DebugChange`), встроенный `ros_debug.py` + блок в `CMakeLists.txt`, `src/mentorpi_bringup/launch/foxglove_bridge.launch.py` и `config/foxglove_bridge.yaml`, параметр `publish_overlay` и рисование рамок (`draw_person_boxes`) в `person_perception.cpp`, топик `/perception/persons/overlay`, порт 8765. Основной просмотр датчиков (rviz по VNC, ярлык «MentorPi rviz») не меняется и остаётся единственным способом посмотреть на робота.

## Нефункциональные требования
- Onboard-детекция постоянно нагружает ядра Pi 5 (~63–69% CPU по замерам SD030) — осознанное следствие отказа от Mac-детекции, не предмет оптимизации в этой задаче.
- Исторические документы `docs/SD/SD013`, `docs/SD/SD024`, `docs/SD/SD026` и другие чужие SD, где Mac-детекция упомянута как факт на момент написания, не переписываются (правило «чужие SD не трогать»); комментарии в текущем коде/конфигах, сравнивающие с Mac (например «depth ~2.15 Hz» из SD024, ByteTrack-сравнение из SD028), можно убрать или переписать — это не чужой документ.
- Миграция не меняет протокол и образ на самом Raspberry Pi 5 (`linux/arm64` Docker остаётся) — меняются только инструменты и настройки хоста разработчика.
