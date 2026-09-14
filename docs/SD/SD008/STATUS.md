# STATUS: Подключение depth-камеры Deptrum

sd: SD008
phase: qa-handoff
ba: approved
design: skipped
sa: approved

## Скипы
- BA: —
- Design: пользователь подтвердил skip; отдельного UI-дизайна нет; Foxglove и `t1ctl` используют существующие поверхности (SD005/SD006)
- Design-review: skipped (отдельного UI-дизайна не было)

## Заметки
- Фича выделена из `SD001` как реализация `F05`
- Объём BA расширен: статическая TF по аналогии с лидаром, визуализация в Foxglove (2D и 3D), статус в `t1ctl`
- Потоки камеры: всё, что отдаёт штатный драйвер; ожидаются цвет и глубина
- Камера не блокирует запуск `demo`-контура; отсутствие данных и TF диагностируется как degraded
- Статическая TF камеры: как у лидара — запасной кадр относительно базы при выключенной или недоступной модели платформы; при активной модели запасная связь не дублирует модель
- BA утверждён; дизайн пропущен по решению пользователя
- План утверждён и перенесён в `docs/SD/SD008/tech.md`
- **T1 done (QA):** `camera_layer.launch.py` → `deptrum-ros-driver-aurora930/aurora930_launch.py`; USB `3251:1930`; overlay `hiwonder_peripherals/depth_camera.launch.py` не используем (там только Dabai). Топики: `/aurora/rgb/image_raw`, `/aurora/depth/image_raw`, `/aurora/points2`. URDF optical `depth_camera_link`; fallback `base_footprint -> depth_camera_link`; `rgb_camera_link` только от драйвера. `provision-pi.sh` копирует isolated prefix + OpenCV 4.10 из RW `MentorPi` в overlay **image** и пересоздаёт `mentorpi-t1`, если контейнер на старом слое. Unit source deptrum `local_setup.bash`. ROS apt-ключ вендорится в `docker/mentorpi-t1/ros.key` (публичный GPG, не секрет).
- **Ограничение стенда (питание):** при питании Raspberry Pi 5 от Anker A1695 Aurora 930 и лидар не работают одновременно: по отдельности оба сенсора работают, при активной камере лидар перестаёт работать. Разнесение по USB-контроллерам не помогло. Вероятен недостаточный бюджет питания, но электрическими измерениями это не подтверждено; совместный режим пока не поддерживается.
- **T2 done (QA):** `t1ctl status` строка `camera` (active/degraded/inactive) в unified probe: `T1CTL_CAMERA`, `T1CTL_CAMERA_COLOR`, `T1CTL_CAMERA_DEPTH`, `T1CTL_CAMERA_TF`. Цвет: compressed если топик есть, иначе `/aurora/rgb/image_raw`. Глубина: `/aurora/points2` или `/aurora/depth/image_raw`. TF: `base_footprint -> depth_camera_link`. Версия t1ctl **1.1.0**.
- **T3 done (QA):** `t1ctl viewer status` cheat-sheet `camera 2d` (compressed or raw), `camera 3d` `/aurora/points2`, `camera tf` `base_footprint -> depth_camera_link`; блок FOXGLOVE DESKTOP с Image + PointCloud2. Без live echo камеры в viewer-probe и без нового whitelist. `docs/SD/SD005/ops.md` — шаги 2D/3D и фактические топики T1.
- **T4 done (QA):** as-built и QA-чеклист в `docs/SD/SD008/tech.md`; F05 → SD008 в `docs/SD/SD001/tech.md`.
- Закрыт: пользователь принял T1–T4 и подтвердил закрытие SD008; `F05` отмечена выполненной в каталоге `SD001`
