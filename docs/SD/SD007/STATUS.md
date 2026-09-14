# STATUS: Модель платформы T1

sd: SD007
phase: qa-handoff
ba: approved
design: skipped
sa: approved

## Скипы
- BA: —
- Design: пользователь подтвердил skip; отдельного UI-дизайна не было; Foxglove workflow использует существующий viewer (SD005)

## Заметки
- Фича выделена из `SD001` как реализация `F07`
- Объём включает визуализацию модели платформы в Foxglove по аналогии с `F03`
- Для отладки достаточно согласованных габаритов и положений сенсоров; детальный продуктовый 3D-меш не требуется
- Ошибка модели не блокирует demo-контур и диагностируется через `t1ctl status`
- В BA явно перечислен полный сенсорный состав из `SD001`: ToF-лидар MS200, глубинная камера Deptrum, IMU и энкодеры гусениц как источник одометрии
- План утверждён и перенесён в `docs/SD/SD007/tech.md`
- **T1 done:** self-contained `mentorpi_description` (tank URDF/Xacro, MS200/IMU/Deptrum frames, mesh-ресурсы). Pose Deptrum preliminary (`depth_cam_x=0.10`, y=0, z=0.05, rpy=0) — не калибровано.
- **T2 done:** отказоустойчивый `robot_model_layer.launch.py` в `stage1` (`enable_robot_model` default true).
- **T3 done:** единый источник lidar TF — URDF при active model; `lidar_frame_tf_fallback` только при `enable_robot_model:=false` или недоступной модели.
- **T4 done:** `t1ctl status` — `platform model` (active/degraded/inactive), unified probe `T1CTL_MODEL*`, адресные hints.
- **T5 done:** `t1ctl viewer status` + Foxglove workflow (RobotModel `/robot_description`, fixed frame `odom`); `docs/SD/SD005/ops.md` обновлён.
- **T6 done (implementation):** статический аудит T1–T5, дополнены unit-тесты `test_status.cpp`, as-built контракт и финальный QA-чеклист в `tech.md`. Build/deploy, стендовый QA, прогон `t1ctl_test` и код-ревью — за пользователем.
- Закрыт: пользователь принял T1–T6 и подтвердил закрытие SD007; `F07` отмечена выполненной в каталоге `SD001`
