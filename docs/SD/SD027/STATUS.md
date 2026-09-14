# STATUS: Окна свежести nearest и облака в offline

sd: SD027
phase: closed
ba: skipped
design: skipped
sa: skipped

## Скипы
- BA: оператор уточнил — `1000` мс только в offline; Mac остаётся `300` / `700`. Новых экранов нет
- Design: `t1ctl detect` без смены вывода
- SA: fast path, Plan не собирали
- Design-review: skipped (дизайна не было)

## Заметки
- **SD027 ЗАКРЫТ (2026-09-10, решение оператора).** После питания и `t1ctl restart`
  окна Mac: `nearest_timeout_ms=300`, `points_timeout_ms=700`. `t1ctl detect offline`
  ставит `detections_source=offline` на `person_perception` и `motion_control` и
  включает окна 1000 мс; `detect mac` возвращает Mac-окна. Гейт `!coasting` не менялся.
  Задачи T1–T3 закрыты. Канон: [`tech.md`](tech.md). Закрытые SD не патчили.
- Версии на закрытии: `mentorpi_perception` 0.6.1, `mentorpi_bringup` 0.6.2,
  `motion_control` 0.4.1, `t1ctl` 1.7.1. Не коммичено — коммит, `make build` /
  `make deploy` и релиз за оператором.
- **Хвост за пределами SD027:** разнести инференс Pi с колбэком RGB, чтобы рамка
  шла ~10 Гц во время NCNN. Отдельного SD не заводили.
- Каталог: F11 в [`docs/SD/SD001/tech.md`](../SD001/tech.md).
