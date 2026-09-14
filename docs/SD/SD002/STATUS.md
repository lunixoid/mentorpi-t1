# STATUS: Bringup и автозапуск после питания

sd: SD002
phase: qa-handoff
ba: approved
design: approved
sa: approved

## Скипы
- BA: —
- Design: —
- Design-review: принят при закрытии SD (CLI на стенде; линейка `=---` снята по запросу)

## Заметки
- Закрыт: пользователь принял после QA на стенде
- Фича F01 из каталога SD001; T1–T7 сделаны; per-task QA снят
- Стенд: контейнер `mentorpi-t1` создан, `MentorPi` не удаляли; running один контейнер на режим
- QA: `setup.zsh` в unit (не `setup.bash` из zsh); `t1ctl stock` делает `docker start MentorPi`; линейка убрана из `t1ctl`
- Режим пульта/джойстика — F24
