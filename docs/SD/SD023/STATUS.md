# STATUS: Makefile сборка окружения

sd: SD023
phase: qa-handoff
ba: approved
design: skipped
sa: approved

## Скипы
- BA: —
- Design: нет UI — меняется только DX сборки на машине разработки
- Design-review: skipped (нет UI)

## Заметки
- **SD023 ЗАКРЫТ.** Пользователь принял. Точка входа — `make`; окружение (два builder-образа) отдельно от проекта (`make overlay` / `make t1ctl` / `make build`). Операторские `scripts/*.sh` удалены.
- `make deploy` при живом `mentorpi-t1`: `systemctl stop` → замена `install/` → `docker restart` → `t1ctl restart`. Image на Pi не пересобирается; `MentorPi` не удаляется.
- T1–T6 сделаны. Версии пакетов робота и `t1ctl` не поднимались (DX сборки на Mac).
