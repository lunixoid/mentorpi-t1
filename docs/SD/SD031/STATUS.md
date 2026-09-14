# STATUS: Голосовое управление

sd: SD031
phase: closed
ba: approved
design: skipped
sa: approved

## Скипы
- BA: —
- Design: экранов нет, UX не нужен (решение оператора 2026-09-13)

## Заметки
- **SD031 ЗАКРЫТ (2026-09-13, решение оператора: «sd031 можно закрывать, проблема скорее всего аппаратная»).** Голосом переключаются все три режима: `voice_command` распознаёт фразу, `command_dispatcher` меняет режим через `/control/set_mode`, нода отправляет ответ в динамик. Ответ из динамика оператор не слышит и причину считает аппаратной. Канон: [`tech.md`](tech.md). Не коммичено, коммит и релиз за оператором.
- Версии на закрытии: `mentorpi_msgs` 1.3.0, `mentorpi_commands` 0.1.0 (новый), `mentorpi_voice` 0.1.1 (новый), `mentorpi_bringup` 0.8.0. `t1ctl` не менялся (1.8.0). Runtime-образ `mentorpi-t1` пересобран: RHVoice, ALSA, `ros-humble-vision-msgs`.
- Открыто на закрытии: ответ в динамик не слышен. Что проверил агент по журналу и `/proc/asound` (стенд, 2026-09-14 01:24–01:30 HKT):
  - на каждый `announce` вывод карты `Device` (WonderEcho: USB `0c76:161f` за одним хабом с CH340 `1a86:7523`) открывается через 35–70 мс, 48 кГц стерео, и работает 0,86–1 с, столько длится фраза;
  - `Speaker Playback Switch` включён; WAV от RHVoice не пустые (24 кГц моно, пик около −10 дБ, средний уровень около −22 дБ);
  - громкость `Speaker` на микшере −31,5 дБ. Её выставляет WirePlumber хоста из сохранённых 30 % (`/home/pi/.local/state/wireplumber/default-routes`, `channelVolumes=0.0266`) при загрузке и при подключении модуля;
  - проверку на громкости 100 % не делали. Следующий шаг: `amixer -c 2 sset Speaker 100%`, вендорский `aplay` на `plughw:CARD=Device`, осмотр динамика модуля. Если причина в громкости, нужен параметр громкости в `voice_command`: в `tech.md` громкость не описана.
- Проверка через Bluetooth-колонку (2026-09-14, после закрытия): оператор — «в колонке есть звук и ответы, все отлично». Журнал после перезапуска: 5 команд на все три режима, 5 ответов `trigger=voice`, `playback failed` 0, свой ответ робот командой не считал, одна неполная фраза `ignored "запрет" reason=no match`. Вывод: код, синтез и политика ответа работают, штатный динамик WonderEcho молчит по аппаратной причине. T5 AC2 и T7 AC2 подтверждены на слух.
- **Стенд сейчас в временной конфигурации:**
  - в работающий контейнер `mentorpi-t1` поставлены `libasound2-plugins` и `alsa-utils`;
  - в `~/.asoundrc` пользователя `ubuntu` устройство `bt`: ALSA `pulse` → PipeWire хоста, выход `bluez_output.B8_87_6E_22_9B_6D.1` (колонка «Станция Мини new»);
  - на хосте профиль колонки переключён в `a2dp-sink` (`pw-cli set-param 71 Profile "{ index: 2, save: false }"`);
  - в установленном `voice_command.yaml`: `playback_device: "bt"`, `mute_tail_ms: 1000`.
  Всё это не в репозитории: `make deploy` вернёт YAML на WonderEcho, `make provision` уберёт пакеты и `~/.asoundrc`, профиль колонки может сброситься при её переподключении.
- **Открыто после закрытия: пока крутятся гусеницы, голосовые команды не проходят.** Наблюдение оператора 2026-09-14: «когда шасси крутятся они шумят и из за этого микрофон ничего не слышит и на команды не реагирует пока не остановится». Значит, голосовой `Режим запрет` во время движения сейчас не срабатывает. Оператор решит это в отдельном SD, в SD031 не исправляется. Известно на момент записи (на ходу не проверялось):
  - `voice_command` берёт только финальный результат Vosk, а Vosk 0.3.45 закрывает фразу по паузе после речи. Настроек этой паузы и максимальной длины фразы в API 0.3.45 нет, есть только промежуточный результат `vosk_recognizer_partial_result`;
  - усиление микрофона на карте `Mic Capture Volume` 412 из 496 (25,75 дБ);
  - запись с микрофона во время езды не снималась. Что именно мешает, не подтверждено: шум не даёт распознавателю закрыть фразу или заглушает саму речь.
- Критерии приёмки на закрытии:
  - подтверждены: T1 AC2, AC3 (голосом, `done success=1`); T3 и T4 AC1 (`colcon test` `mentorpi_voice` 4 из 4); T4 AC2, AC4; T5 AC1, AC4, AC2 программно (ответ проигрывается, но не слышен); T6 AC1–AC3; T7 AC1, AC2 программно (включая повтор текущего режима), AC6 (детекция 8,6–11,1 Гц, ядра 64,7–67,1 % против 63–69 % SD030);
  - не подтверждены: T1 AC1 (прогона `test_command_table` нет), AC4, AC5; T2 AC3; T4 AC5; T5 AC3 (озвучка по кнопке пульта), AC5; T6 AC4; T7 AC3–AC5.
- Хвосты по коду, в SD031 не исправлялись: `command_dispatcher` помнит один ожидающий запрос (при двух командах подряд строка `done` первой теряется, по таймауту пишется `rejected`, хотя запрос мог примениться); проверка multiverse в `docker/mentorpi-t1/Dockerfile` считает включённой закомментированную строку; в логе `capture open failed` печатается имя из параметра, а открывается `plughw:`.
- После перезапуска контура `control_state` стартует в `AutoFollow` (SD011): «робот в `forbidden`» держится только до рестарта.
- Заведён 2026-09-13 по фиче F21 из `docs/SD/SD001/tech.md`. Тексты SD проходят через скилл humanizer-ru (просьба оператора).
- BA утверждён 2026-09-13 («БА принимаем, UX не нужен, приступай к СА»). Перед утверждением оператор сам сократил пункт 2 раздела «Проблема».
- SA утверждён 2026-09-13 (план одобрен, «пиши сразу tech.md, но к реализации не приступай»). Канон: [`tech.md`](tech.md), 7 задач.
- T1 реализован 2026-09-13 (код), QA оператора ещё нет: `NamedCommand` в `mentorpi_msgs` 1.3.0, пакет `mentorpi_commands` 0.1.0, нода `command_dispatcher`. Launch — T6.
- T2 реализован 2026-09-13 (код), QA оператора ещё нет: `docker/overlay-builder/Dockerfile` — apt ALSA/JSON/unzip/curl, `ARG`/`ENV` Vosk, `vosk-linux-aarch64-0.3.45` в `/opt/vosk`, `vosk-model-small-ru-0.22` в `/opt/vosk-models`; sha256 вписаны с первого скачивания. `make env-overlay` / `make overlay` агент не запускал.
- T3 реализован 2026-09-13 (код), QA оператора принят («ок делай t4»).
- T4 реализован 2026-09-13 (код), QA оператора ещё нет: нода `voice_command` (захват ALSA + Vosk + `/commands/named`), YAML I6, CMake I12.2 (`libvosk.so`, модель, RUNPATH `$ORIGIN/..`), `test_vosk_phrases` и WAV-фикстуры `say -v Milena` / `afconvert` 16 кГц моно. Ответ в динамик — T5. `colcon` в сборщике агент не запускал.
- T5 реализован (код): TTS-кеш, озвучка режима, MuteWindow; стендовые AC1–AC4 ждут T7; AC5 (`tts_command:=/nonexistent`) проверяется на стенде.
- T6 реализован (код): Dockerfile RHVoice/ALSA + multiverse, `image_has_voice_stack` в provision, `command_dispatcher` + `voice_command` в stage1, `enable_voice` default true; AC3/AC4 (provision и `enable_voice:=false` на стенде) ждут T7. `make overlay` OK (16 пакетов), `--show-args` показывает `enable_voice` default `true`, `pre-commit` OK (clang-format поправил чужие T3–T5 файлы).
- Технические решения оператора (2026-09-13): Vosk small-ru; libvosk и модель в образе сборщика; ответ синтезируется на Pi (RHVoice); контракт команды это отдельный топик `/commands/named` и новая нода `command_dispatcher`, не `mission_control`.
- Ответы оператора на вопросы BA (2026-09-13):
  - распознавание русское, офлайн на Pi;
  - F21 даёт только контракт «именованная команда запускает поведение»;
  - команды просто переключают режимы forbid, follow, manual, речь и ответы на русском;
  - голос, пульт и `t1ctl` переключают режимы независимо, конфликта нет;
  - ответ в динамик «Режим <название режима на русском>»;
  - слушать сразу после включения питания.
- Правки оператора на гейте BA (2026-09-13): фраза команды и ответ совпадают («Режим запрет», «Режим следование», «Режим ручной»); смену режима кнопкой пульта тоже озвучиваем. Без возражений остались: ответ при повторе текущего режима, без слова-активатора. Смена через `t1ctl` не озвучивается (прочитано буквально, отдельно не подтверждено).
- Референс железа: WonderEcho Pro (вендорские уроки 20.1 и 20.2). Это USB-аудио (микрофон и динамик) и чип CL1302, который сам распознаёт слово-активатор и фиксированный набор команд и отдаёт их по UART через CH341 (`AA 55 00 xx FB`). Прошивку собирают на платформе ChipIntelli, языки только английский и китайский. Распознавание речи в вендорском уроке 1.2.4 облачное (OpenAI).
- Стык под команды уже есть: сервис `/control/set_mode` (`SetControlMode`) у `control_state` принимает любой допустимый режим из любого состояния, `/control/mode_toggle` от пульта не выводит из `Forbidden`, `t1ctl mode forbid|allow|manual`.
- T7 стенд 2026-09-14 (агент): SSH Ethernet `192.168.88.56` OK. `make build` OK. `make provision`: rebuild — `image mentorpi-t1 exists but voice stack (RHVoice/ALSA) is missing`. После: `RHVoice-test` и `libasound.so.2` в контейнере. `make deploy` OK, `t1ctl` 1.8.0.
- T7 As-built 2026-09-13: WonderEcho подключён — USB `0c76:161f` (JMTek USB PnP Audio Device), ALSA-карта `Device` (card 2). `plughw:CARD=Device,DEV=0` capture/playback открывается при работающем PipeWire (wireplumber держит `/dev/snd/controlC2`). `voice_command.yaml`: `capture_device`/`playback_device` = `plughw:CARD=Device,DEV=0`, `capture_channels` = 1.
- T7 AC1 закрыт (As-built + yaml). T7 чекбокс `[ ]`: голосовая приёмка AC2–AC5 — оператор.
- T7 неходовая приёмка 2026-09-13: после `make overlay`+`make deploy` и рестарта — `command_dispatcher started`, `voice_command started` с `capture_device=plughw:CARD=Device,DEV=0`, `tts ready` ×3, `/tmp/mentorpi_voice` — 3 WAV; `capture open failed` после рестарта **0** (до yaml были WARN с `device=default`). Робот в `forbidden`.
- T7 AC6 (2026-09-13, после vision_msgs): `t1ctl detect offline` OK. `/perception/detections_2d_onboard` ≈ 8.6–11.1 Гц (окно 20, типично ~9 Гц; SD030 было 10.2 Гц). Ядра хоста 64.7–67.1 % за 5 с (`/proc/stat`), сумма контейнера `docker stats` 264 %. Рядом с 63–69 % SD030. `ldd` по executable в overlay install: missing libs нет. Робот в `forbidden`, голос включён.
- Регрессия после SD031 provision: runtime-образ FROM stock без `ros-humble-vision-msgs` (пакет был только в overlay-builder). Ноды perception exit 127. Фикс: apt `ros-humble-vision-msgs` в `docker/mentorpi-t1/Dockerfile` + `image_has_vision_msgs` в `provision.sh`. Provision 2026-09-13: rebuild «vision-msgs is missing»; `t1ctl detect offline` → `T1CTL_DETECT_OK=1`, source offline. Робот в `forbidden`.
- 2026-09-13 стенд не отвечал по SSH (Ethernet и Wi-Fi); 2026-09-14 Ethernet восстановлен.
- TTS (ревью): `SIGPIPE` игнорируется, argv до fork, `waitpid` 10 с + SIGKILL. `mentorpi_voice` 0.1.1. Захват тоже открывается как `plughw`, если в параметре `hw:`.
- Детекция offline занимает ядра Pi 5 на 63–69 % (замер SD030).
- Сканер humanizer-ru (`scripts/scan.py`) не запускался: в python3 нет `razdel` и `pymorphy3`. Тексты вычитаны вручную по жёстким запретам.
