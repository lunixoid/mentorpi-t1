# STATUS: F11 слежение за целью (motion control)

sd: SD020
phase: qa-handoff
ba: approved
design: skipped
sa: approved

## Скипы
- BA: —
- Design: нет UI — только ROS-топик /pnc/desired_twist и параметры; экраны/t1ctl не меняются

## Заметки
- **SD020 ЗАКРЫТ.** Реализация T1–T4 проверена и корректна: гейт desired==vehicle 2021/2021, vehicle==hiwonder 4041/4041, stop_request 0%, реверса нет, насыщение трака 0 нарушений, закон vs спека err=0.0000.
- QA на стенде (бэг f11_follow_20260904_182253 + видео f11_run1.mp4): робот доехал до стоящего впереди человека, НО перцепшн отдаёт цель как x<0 (позади) при человеке спереди — инверсия оси X кадра камеры. Дефект ВЫШЕ F11 (перцепшн/TF), контракт I1 нарушен на входе. Заведён отдельный виток **SD021**.
- T4 реализован: `motion_control` в `stage1.launch.py` с параметрами I3, bringup `0.3.0` + `exec_depend`, README — реальный `/pnc/desired_twist`. `control_mux`/`odom_publisher`/`t1ctl` без правок. Сборку агент не запускал.
- design-review skipped: UI не менялся.
- T3 реализован: `stub_graph` больше не публикует `/pnc/desired_twist`; `/control/motion_restriction` (`stop_request=false`) остаётся. `mentorpi_stubs` 0.2.0.
- T2 реализован: нода `motion_control` (подписки I1, watchdog D4, гейт D3, `/pnc/desired_twist` с таймера).
- T1 реализован: пакет `motion_control` 0.1.0, header-only `compute_follow_twist`, CTest без ROS. Хостовый clang++ тест: ok.
- measure.md — операторская методика замера скоростей. Замер 1 снят (бэги на pi@192.168.2.2, контейнер mentorpi-t1, /home/ubuntu/mentorpi_t1_ws/bags/).
- Итог Замера 1: max_angular=2.0 подтверждён; путевая max_linear=0.375 м/с (1.0м/2.67с, секундомер). Odom-скорость ≈ верна (run2 0.376 ≈ 0.375; run1 0.44 +17%); прежний вывод «odom +35% дистанции» снят — то была вся прокрутка, не масштаб.
- Odom СЛЕП к физич. уводу по курсу 20-30°/м (angular.z≡0, Δyaw=0) → проскальзывание траков. Это остаётся.
- Следствие для F11: контур сервить на /perception/nearest_person (range+пеленг atan2(y,x)), НЕ на odom по курсу. Войдёт в системный анализ (tech.md).
- Ограничение платформы: путевой потолок ~0.37 м/с < нормального шага (1.1-1.4). max_target_speed снижен 0.4→0.3. Робот держит только медленно идущего.
- Замер 2 (скорость ухода человека) — ПРОПУЩЕН по решению пользователя. Допущение: человек стоит или идёт очень медленно (до ~0.5 м/с кратковременно). max_target_speed=0.3 (сустейн), 0.5 и выше — деградация (робот отстаёт, до-закрывает на остановке), потолок 0.37.
- Параметры для F11 зафиксированы: max_linear≈0.37, max_angular=2.0, max_target_speed=0.3, standoff=0.5. BA готов к утверждению.
- F11 из docs/SD/SD001/tech.md. Разблокирована: входы (F06 /control/state, F08+F09 /perception/nearest_person) и выход (control_mux→platform_adapter) готовы.
- F11 заменяет публикацию нулей на /pnc/desired_twist в stub_graph (D4 из SD019). stub_graph продолжает публиковать /control/motion_restriction (F12/F13 не готовы).
- Ограничение на время F11: охраны столкновений с предметами нет (F12/F13), тесты в свободном пространстве.
