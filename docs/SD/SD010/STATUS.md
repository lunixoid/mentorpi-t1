# STATUS: IMU и одометрия гусениц

sd: SD010
phase: qa-handoff
ba: approved
design: skipped
sa: approved

## Скипы
- BA: —
- Design: отдельного UI нет; Foxglove и `t1ctl` используют существующие поверхности (SD005/SD006/SD008)
- Design-review: skipped (отдельного UI-дизайна не было)

## Заметки
- Фича выделена из `SD001` как реализация `F04`
- Соседние SD уже дают сырой `/odom_raw` и TF `odom → base_footprint` (SD005/SD003) и кадр `imu_link` в модели (SD007); живые `/imu` и продуктовый контракт F04 ими не закрыты
- BA: отсутствие IMU/одометрии не блокирует `demo`-контур, только явно диагностируемый degraded
- BA: отдельные строки `t1ctl status` для IMU и одометрии
- BA: визуализация IMU в Foxglove (ориентация и ускорение)
- BA: поза этапа 1 follow — `/odom_raw` + TF `odom → base_footprint`; `/odom` (фьюжн) остаётся **F15**
- BA: IMU-одометрия готовится как высокочастотный поток; продуктовое использование — в **F15**
- Дизайн пропущен по решению пользователя
- План утверждён и перенесён в `docs/SD/SD010/tech.md`
- **T1 done (QA):** `imu_layer.launch.py` → vendor `hiwonder_peripherals/launch/imu_filter.launch.py`; цепочка `/ros_robot_controller/imu_raw` → `imu_calib` → `/imu_corrected` → `imu_filter` → `/imu`; remap overlay не нужен. TF `imu_link` от URDF при активной модели, иначе `imu_link_tf_fallback`. `publish_tf` фильтра = false. `mentorpi_bringup` **0.1.3**.
- **T2 done (QA):** пакет `mentorpi_localization` **0.1.0**, нода `imu_odometry` → `/imu_odom` (~50 Hz), без TF; поза (0,0,0). Follow и `platform_adapter` не читают поток.
- **T3 done (QA):** `t1ctl status` строки `imu` и `odometry`. t1ctl **1.3.0**: probe `/imu` 5 с + dual QoS (apt `complementary_filter` публикует RELIABLE); 1.2.1 давал ложный `(no /imu)` сразу после restart.
- **T4 done (QA):** `t1ctl viewer status` cheat-sheet `imu`, `imu odom`, `imu tf`; блок FOXGLOVE DESKTOP (Plot/IMU: orientation + linear_acceleration `/imu`). `docs/SD/SD005/ops.md` — шаги проверки IMU.
- **T5 done (QA):** as-built и QA-чеклист в `docs/SD/SD010/tech.md`; F04 → SD010 в `docs/SD/SD001/tech.md`.
- **Образ IMU-фильтра:** vendor-образ не содержит `imu_complementary_filter`. Overlay ставит apt `ros-humble-imu-complementary-filter` (`docker/mentorpi-t1/Dockerfile`); `provision-pi.sh` пересобирает image, если ноды нет в `/opt/ros/humble`. Не source-сборка `CCNYRoboticsLab/imu_tools`.
- **Технический долг (образ):** в контейнере `mentorpi-t1` нет `ss` (`iproute2`); probe `T1CTL_PORT` возвращал 0, блок FOXGLOVE DESKTOP в `t1ctl viewer status` не печатался. В T4 `iproute2` установлен в контейнер вручную, в образ не зафиксирован.
- **Найдено вне scope (platform model):** `t1ctl status` показывает `platform model: degraded` — существовало до SD010. Probe ждёт TF `base_footprint → depth_cam_frame`, URDF даёт `depth_cam_link` / `depth_camera_link`; кадра `depth_cam_frame` в дереве нет. Исправление — отдельная задача (не SD010).
- Финальный QA: инструкция `docs/SD/SD010/ops.md`. Пользователь на стенде подтвердил `/imu`, `/imu_odom`, TF `base_footprint → imu_link`, `/odom_raw` и TF `odom → base_footprint`.
- Закрыт: пользователь принял T1–T5 и подтвердил закрытие SD010; `F04` отмечена выполненной в каталоге `SD001`
