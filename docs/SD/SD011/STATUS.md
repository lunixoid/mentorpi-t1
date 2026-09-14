# STATUS: Режимы управления

sd: SD011
phase: qa-handoff
ba: approved
design: approved
sa: approved

## Скипы
- BA: —
- Design: —

## Заметки
- Источник требований: F06 в `docs/SD/SD001/tech.md`.
- Каркас режимов частично сделан попутно в SD004: топик режима, значения enum, toggle с пульта, обнуление на `Forbidden` в адаптере.
- Утверждённое решение SA: `Forbidden` в первой реализации включается только явной командой оператора; асинхронный auto-`Forbidden` по деградации mode-subsystem в этот SD не входит.
- Дизайн утверждён пользователем; макеты CLI — в `docs/SD/SD011/design/`.
- `tech.md` утверждённым планом записан в `docs/SD/SD011/tech.md`.
- **T1 done (QA):** boot `state: 2` + пустой `reason`, `set_mode(0)` → `operator`, toggle не снимает запрет, `set_mode(2)` возвращает follow.
- **T2 done (QA):** mux выбирает источник, логирует рёбра `state` и `stop_request`.
- **T3 done (implementation):** read-side `t1ctl status`: `forbidden`/`reason`, без ложного `follow` при отсутствии `/control/status`.
- **T4 done (implementation):** `t1ctl mode forbid|allow|manual` через in-container `ros_mode.py` + `SetControlMode`; отказ без fallback publish и без ложного `follow`.
- **T5 done (implementation):** `control_mux` в `stage1.launch.py`, `t1ctl` 1.4.0, patch ROS (`mentorpi_msgs`/`mentorpi_control` 0.1.1, `mentorpi_bringup` 0.1.4), `./scripts/test-host.sh` и `./scripts/test-ros.sh`.
- Design-review: сверка CLI с макетами `docs/SD/SD011/design/` — в чеклисте handoff (T3/T4).
- Закрыт: пользователь подтвердил закрытие SD011; `F06` отмечена выполненной в каталоге `SD001`. Релиз / тег / деплой — пользователь.
