# SD023. Makefile: окружение отдельно от проекта

## Проблема

1. Каждый запуск `./scripts/build-arm64.sh` поднимает одноразовый `docker run --rm` и заново ставит apt-пакеты в `ros:humble` (colcon, vision/tf, distro OpenCV) и в `debian:12` (g++/cmake). Это не пересборка кода, а повторная установка окружения; инкремент colcon в `build-arm64/` от этого не спасает.
2. Обход: всё равно гонять полный скрипт и ждать `apt-get` перед каждой выкладкой на Pi.

## Решение

Разделить окружение и проект: два локальных linux/arm64-образа с уже установленными пакетами; ежедневная сборка overlay и `t1ctl` идёт в этих образах без apt. Единственная точка входа оператора — GNU Makefile в корне (`make` печатает help; `make build` собирает оба артефакта; overlay и t1ctl доступны порознь). Образ окружения пересобирается, только если изменился Dockerfile или локального тега нет. Операции бывших корневых `scripts/*.sh` (сборка, deploy, provision, Mac-детект) становятся целями Make.

## Сценарии

### Позитивный

1. На машине разработки нужен arm64 overlay и `t1ctl` после правки кода.
2. Оператор в корне репозитория вызывает `make` без цели.
   1. Печатается список целей, docker и colcon не стартуют.
3. Оператор вызывает `make build`.
   1. Если builder-образов нет или изменился Dockerfile — `docker build` образов окружения (apt только здесь). Затем `docker run` без apt: colcon overlay и cmake `t1ctl` в `build-arm64/`.
4. Повторный `make build` при том же Dockerfile.
   1. Apt внутри `docker run` нет; colcon/cmake инкрементальны по `build-arm64/`.
5. Нужен только overlay или только `t1ctl` — `make overlay` / `make t1ctl`.
6. Выкладка: `make deploy` копирует overlay, перезапускает контейнер `mentorpi-t1` и demo-launch; первичная настройка стенда: `make provision`.

### Негативный №1

1. Пункты позитивного сценария до `make deploy`, но `build-arm64/ros/install/setup.bash` или `t1ctl` нет.
   1. Deploy отказывается со ссылкой на `make build`, на Pi ничего не копирует.

### Негативный №2

1. `make mac-detect` не на Darwin arm64 (или без pixi).
   1. Цель завершается ошибкой, linux/arm64 Docker для детектора не используется.

## Функциональные требования

- Должна быть возможность собрать окружение отдельно от проекта (`make env` / `env-overlay` / `env-t1ctl`).
- Ежедневная сборка проекта не должна выполнять `apt-get` внутри `docker run`.
- `make` без цели должен печатать help и не собирать.
- `make build` должен собирать overlay и `t1ctl`; отдельно — `make overlay` и `make t1ctl`.
- Образ окружения должен пересобираться при изменении Dockerfile или отсутствии локального тега.
- Должны быть цели `deploy`, `provision`, `mac-detect`, `echo-detections`. `make deploy` при наличии контейнера `mentorpi-t1` должен перезапустить контейнер и demo-launch, чтобы новый overlay гарантированно загрузился.
- Артефакты должны остаться в `build-arm64/ros/install` и `build-arm64/t1ctl/bin/t1ctl` (ELF aarch64).

## Нефункциональные требования

- Сборка overlay и `t1ctl` для Pi — на машине разработки через Docker `linux/arm64`, не на Pi.
- Рантайм-образ `mentorpi-t1` на Pi и `docker/mentorpi-t1/Dockerfile` не меняются этой работой.
- Не `docker rm MentorPi`. Агент не запускает сборку и деплой.
- GNU Make 3.81 (`/usr/bin/make` на macOS) должен быть достаточен; отдельный `gmake` не требуется.
- Версии пакетов робота и `t1ctl` не поднимаются: на стенде поведение то же, меняется DX сборки.
