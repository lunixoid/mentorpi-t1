# STATUS: rviz2 на рабочем столе VNC

sd: SD032
phase: closed
ba: approved
design: skipped
sa: approved

## Скипы
- BA: —
- Design: экранов приложения нет; ярлык рабочего стола и готовый rviz, отдельный UX не нужен (решение оператора 2026-09-13)

## Заметки
- **SD032 ЗАКРЫТ (2026-09-13, решение оператора: «закрывай sd»).** Основной просмотр — ярлык `MentorPi rviz` на столе VNC (`/home/pi/Desktop/mentorpi-rviz.desktop` → `/usr/local/bin/mentorpi-rviz` → `docker exec` `rviz2 -d demo.rviz`). Foxglove — запасной путь: `t1ctl debug on` поднимает overlay и `foxglove_bridge`, `debug off` гасит оба; `t1ctl viewer` снят. Канон: [`tech.md`](tech.md). Не коммичено, коммит и релиз за оператором.
- Версии на закрытии: `t1ctl` 1.9.0, `mentorpi_bringup` 0.9.0. Runtime-образ `mentorpi-t1` пересобран: `ros-humble-rviz2`, контейнер с `DISPLAY=:0` и `QT_X11_NO_MITSHM=1`.
- Смоук на закрытии (стенд, без запуска окна агентом): ярлык и лаунчер на месте, `Exec=/usr/local/bin/mentorpi-rviz`, в лаунчере нет `t1ctl`/`cmd_vel`; `command -v rviz2` после Humble → `/opt/ros/humble/bin/rviz2`; `pgrep foxglove_bridge` пусто; `t1ctl --version` 1.9.0. Клик по ярлыку — оператор.
- Заведён 2026-09-13: в demo-контейнере `rviz2` не стартует (нет дисплея для Qt). SD031 к этому не относится, закрыт.
- Ответы оператора на вопросы BA (2026-09-13):
  - вход по VNC; на рабочем столе ярлык, который открывает rviz2 с 3D-сценой и нужными топиками;
  - rviz2 только вручную с ярлыка, не вместе со стеком; но `rviz2` в контейнере должен работать;
  - Foxglove — запасной путь; по умолчанию выключен и спрятан за `t1ctl debug on`;
  - иксы пробросить в контейнер; графические приложения сами не поднимаются; если позже поднимутся — не должны блокировать контур;
  - только demo, stock не трогаем.
- As-is, не решение: `t1ctl debug on` уже есть (SD014) и включает overlay `/perception/persons/overlay`, не мост Foxglove. Мост сейчас стартует со стеком (`viewer_bridge:=true`). В BA запасной Foxglove тоже за этой командой — на гейте поправить имя, если имелось другое.
- BA утверждён 2026-09-13 («ок, давай СА без ux»). Имя `t1ctl debug on` для запасного Foxglove оператор не поправил.
- SA утверждён 2026-09-13 (план в UI Plan + «ок пиши план»). Канон: [`tech.md`](tech.md), 5 задач. Уточнения к плану: 3D-сцена rviz при `debug off`; на диаграмме VNC нет debug (отдельный путь Foxglove).
- Имплементация T1–T5 субагентами composer-2.5; код родитель не пишет. Сборка и деплой разрешены оператором. Окно rviz запускает оператор. Смоук родителя: ярлык в `~/Desktop` на Pi.
- 2026-09-13: `make build` + `make deploy` + `make provision`. `t1ctl` 1.9.0, `mentorpi_bringup` 0.9.0. Образ пересобран из‑за `ros-humble-rviz2`; контейнер с `DISPLAY=:0`. Ярлык `/home/pi/Desktop/mentorpi-rviz.desktop` → `/usr/local/bin/mentorpi-rviz`.
