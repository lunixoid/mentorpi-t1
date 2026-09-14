# STATUS: Миграция разработки на Linux

sd: SD034
phase: qa-handoff
ba: approved
design: skipped
sa: approved

## Скипы
- BA: —
- Design: экраны и визуальные компоненты не меняются; правки в тексте справки `t1ctl` — не UI-дизайн

## Заметки
- **SD034 ЗАКРЫТ (2026-09-14, решение оператора: «все, работает, закрываем SD и задачи»).** Хост разработки — Linux x86_64; сборка и `make deploy` с этой машины; onboard YOLO — единственный путь детекций, включён сразу; follow на стенде работает; голос SD031 после выкладки жив (ответ в Bluetooth-колонку — стендовый обход, не в репозитории). Задачи T1–T6 закрыты. Канон: [`tech.md`](tech.md). Коммит и релиз за оператором.
- Версии на закрытии: `t1ctl` 2.0.0, `mentorpi_person_detect` 0.4.0, `mentorpi_perception` 0.9.0, `motion_control` 0.5.0, `mentorpi_bringup` 0.10.0.
- Код-ревью `git diff master` 2026-09-14, оператор попросил исправить всё, кроме удаления `docs/reverse-architecture.md` (удалил оператор). Исправлено: `pre-commit` в README, AGENTS и D1.1 ставится через pip (apt 2.17 падает на манифесте clang-format v21, тег `textproto`); `.clang-format` добавлен в индекс через `git add -f` (его скрывал глобальный `~/.gitignore`), коммит за оператором; `HTTP_PROXY`/`HTTPS_PROXY` в `make help`, README и I8; `points_timeout_ms` и `nearest_timeout_ms` стали read-only; убраны Mac-хвосты в комментарии юнита и описании `mentorpi_person_detect`, возвращён `#include <algorithm>`. Версии после ревью: `mentorpi_perception` 0.9.1, `motion_control` 0.5.1 — на стенд попадут со следующим `make build && make deploy`.
- Хвосты вне SD034: ответ в колонку (`playback_device: bt`, `~/.asoundrc`, `libasound2-plugins`) не в git — следующий `make deploy` вернёт WonderEcho; `FORCE_REBUILD=1 make provision` (T5 AC6) не делали, пакет `foxglove_bridge` может остаться в старом образе, мост не запускается. SD033 (голос на ходу) по-прежнему на паузе.
- Заведён 2026-09-14. Запрос оператора: SD033 (голосовые команды) поставлен на паузу, сейчас в приоритете миграция разработки на Linux — полностью удалить инференс человека на Mac и настроить сборку/деплой под Linux-хост разработчика.
- Ветка `feature/migrate_to_linux` уже создана (чистая, только initial commit).
- Дев-машина на момент заведения: `lunix0x-workstation`, Linux 5.15.0-191-generic, x86_64, Docker 24.0.7 + buildx 0.11.2. `sshpass` не установлен. Эмуляция `docker run --platform linux/arm64` не работает (`exec /bin/uname: exec format error`) — нужен qemu/binfmt.
- Ответы оператора на вопросы BA (2026-09-14):
  - убрать концепцию источника детекции (`detections_source`) целиком — offline (onboard YOLO11n на Pi 5) остаётся единственным путём;
  - onboard-детекция включается по умолчанию сразу при старте стека, без ручного `t1ctl detect offline`;
  - подтверждено разделение: мигрирует только машина разработчика (эта, x86_64 Ubuntu), цель деплоя — тот же Raspberry Pi 5 (arm64, Docker) без изменений;
  - установку sshpass и qemu binfmt для linux/arm64 на этой машине включить в задачу.
- Затронутые компоненты (найдено грепом на момент BA, уточнится в СА): `host/mac_person_detect/` (весь пакет), `Makefile` (цель `mac-detect`, help, комментарий про macOS make), `mk/mac-detect.sh`, `mk/echo-detections.sh`, `mk/pi-detect.sh` (ARGS=mac), `mk/deploy.sh`, `mk/provision.sh` (Homebrew PATH, sshpass path, Cursor/macOS комментарии), `host/t1ctl/src/{main,ui,units,units.hpp}` + `ros_detect.py` (`detect mac` подкоманда, DetectCommand::Mac, парсинг source), `host/t1ctl/tests/test_status.cpp` (тесты mac-веток), `src/mentorpi_bringup/launch/stage1.launch.py`, `src/mentorpi_perception/{config/person_perception.yaml,src/person_perception.cpp}`, `src/motion_control/src/motion_control.cpp` (параметр `detections_source`, ветки `mac`/`offline`), `README.md`, `AGENTS.md`.
- Исторические `docs/SD/SD013`, `SD024`, `SD026` и т.п. не трогаем (правило чужих SD).
- На гейте BA оператор расширил scope: убрать `make pi-detect` и всю подкоманду `t1ctl detect` (путь инференса один), мост Foxglove вместе с оверлеем рамок и `t1ctl debug`. BA утверждён 2026-09-14 («ок»).
- Ответы оператора на технические вопросы СА (2026-09-14):
  - топик детекций переименовать в `/perception/detections_2d`;
  - `ROS_LOCALHOST_ONLY=0` оставить, переписать только комментарий юнита;
  - локально можно `make env`/`make build`, нативный `t1ctl_test`, `colcon test` в builder-образе, `pre-commit`; `make deploy`/`make provision` и стенд — только оператор;
  - эмуляция arm64 через apt `qemu-user-static` + `binfmt-support`; туда же `sshpass`, `file`, `cmake`, `pre-commit`.
- СА утверждён 2026-09-14 («ок, пиши tech.md»): [`tech.md`](tech.md), задачи T1–T6.
- Имплементация T1–T6 завершена 2026-09-14. Хостовые AC — суб-агенты; `make deploy` и стенд — оператор (follow, голос, колонка).
- Design-review skipped: UI-макетов не было (`design: skipped`).
- Хост: tinyproxy `127.0.0.1:8888`; для Docker build нужен `HTTP_PROXY=http://172.17.0.1:8888` (в Makefile --build-arg). Apt `pre-commit` 2.17 падает на clang-format hook; проходит `~/.local/bin/pre-commit` 4.6.2.
