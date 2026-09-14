# STATUS: Pre-commit format lint

sd: SD018
phase: qa-handoff
ba: approved
design: skipped
sa: approved

## Скипы
- BA: —
- Design: skipped — нет UI; меняется только локальный Git-хук разработчика
- Design-review: skipped — UI не менялся

## Заметки
- СА утверждён в Plan UI; канон в [tech.md](tech.md) (T1–T4)
- T1: `.clang-format` Google 100, `.pre-commit-config.yaml` (clang-format, black, isort, flake8)
- T2: 35 файлов C++ переформатированы
- T3: Python black/isort/flake8 зелёные; flake8 `--extend-ignore=E203`, E402 только `person_detect.py`
- T4: README и AGENTS.md
- CI нет; версии пакетов не поднимались
