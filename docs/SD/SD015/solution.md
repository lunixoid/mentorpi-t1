# SD015. Исследование моделей детекции человека

Каталог: F08. Пайплайн: [SD013](../SD013/solution.md), задержка: [SD014](../SD014/solution.md).
Это не бизнес-анализ: инвентаризация готовых моделей, которые стыкуются с текущим контуром без смены ROS-контракта.

На стенде не гонялось. Цифр FPS/mAP с Aurora нет. SLA не задаём.

## Зачем

После SD013/SD014 гипотезы людей есть, но детекция нестабильна: в одном и том же кадре (одни и те же люди) модель то находит, то нет. Сопровождение во времени (F09) это не лечит — сначала должен стабильно появиться bbox класса человек.

Сейчас на Mac стоит официальный **Ultralytics YOLO11n-pose** (`yolo11n-pose.pt`). Keypoints в топик не идут; нужна только рамка. Pose выбрали в SD013 как «единственный класс — человек», не как лучший детектор bbox.

## Что нельзя менять

Горячий путь as-built:

1. Pi: `/aurora/rgb/image_raw` (640×400, ~10 Гц) → LAN → Mac.
2. Mac: `person_detect` — `ultralytics.YOLO(weights).predict(...)` на MPS → `/perception/detections_2d` (`vision_msgs/Detection2DArray`, класс `person`, stamp как у RGB).
3. Pi: `person_perception` — низ центра рамки + `/aurora/points2` + TF → `/perception/persons`, `/perception/nearest_person`.

Уже есть смена весов без правки кода: `WEIGHTS` / `--weights` в `scripts/run-mac-person-detect.sh`. Условие: объект грузится через `YOLO()`, у результата есть `.boxes`, класс называется `person` **или** `cls_id == 0` (fallback в `_is_person`).

Не входят: новые топики, сжатие RGB, инференс на Pi, Hailo, геометрия range, F09, Twist.

Бюджет задержки SD014 (100 мс на весь контур) остаётся ограничением; на M3 замеров новых моделей нет.

## Почему pose плохо стыкуется с этой задачей

Пайплайну нужен **детектор человека**, не скелет. YOLO11n-pose учится на COCO Keypoints: bbox и 17 суставов вместе. Люди без видимых keypoints, сильно обрезанные кадром или с ракурса вне COCO, модель часто не отдаёт как инстанс.

Стенд: камера Aurora **0.145 м** от пола ([SD012/measure.md](../SD012/measure.md)) — вид снизу, крупный человек вблизи, ноги/торс обрезаны краем кадра. COCO — обычные фото с уровня груди/глаз.

Зафиксированный прецедент: [ultralytics#17757](https://github.com/ultralytics/ultralytics/issues/17757) — YOLO8/11-pose не находит людей, которых detect находит (потолок/стена, дальние, нетипичный ракурс). Порог keypoints не помогает: инстанса нет.

Мерцание вокруг порога `conf=0.25` (default SD013, не SLA) тоже возможно: score прыгает около отсечки. Это не доказано на стенде; проверяется сравнением detect vs pose на одном RGB.

NMS на CPU (`PYTORCH_ENABLE_MPS_FALLBACK=1`) может добавлять дрожание рамок; не объясняет полное исчезновение человека, если score стабильно выше порога.

## Критерий «применимо»

| Уровень | Что меняется | Примеры |
|---------|----------------|---------|
| **A. Drop-in** | Только файл/`WEIGHTS` | `yolo11n.pt`, `yolo26n.pt`, чужой `.pt` с `names` ⊇ `person` |
| **B. Одна строка predict** | Тот же `YOLO()`, плюс `classes=[0]` или имя класса | COCO detect, фильтр person до публикации |
| **C. Тот же ROS, другой конструктор** | `YOLOE` / `RTDETR` вместо `YOLO()` | open-vocab, transformer |
| **D. Не подходит** | Другой runtime, другой выход, другая геометрия рамки | OpenVINO, CoreML-only, детектор голов, Roboflow Inference |

Дальше — только A и B, плюс пометка C/D почему нет.

## Кандидаты A/B (официальные Ultralytics)

Все грузятся `YOLO("….pt")`, MPS, `.boxes`. Скачивание с релизов Ultralytics, как сейчас pose.

### 1. `yolo11n.pt` — detect, тот же nano

- Задача: bbox, 80 классов COCO; `person` = класс 0. Фильтр уже есть в `person_detect.py`.
- Параметры: 2.6M / 6.5B FLOPs vs pose 2.9M / 7.4B — тот же класс размера.
- Зачем: убрать зависимость bbox от keypoints; тот же ракурс, на который detect учился лучше pose ([#17757](https://github.com/ultralytics/ultralytics/issues/17757)).
- Риск: остальные 79 классов на NMS (стул/рюкзак рядом с человеком). Смягчение уровня B: `classes=[0]` в `predict` — правка одной строки, ROS не трогаем.
- Смена пайплайна: нет (A), либо B.

### 2. `yolo26n.pt` — detect, NMS-free

- Семейство января 2026; документация: [YOLO26](https://docs.ultralytics.com/models/yolo26). Dual-head, инференс без NMS (e2e).
- COCO mAP 40.9 vs YOLO11n 39.5 (таблица Ultralytics, не Aurora).
- Зачем: то же, что п.1, плюс нет torchvision NMS на CPU — кандидат, если мерцание от NMS.
- Риск: в `pixi.toml` сейчас `ultralytics>=8.3.0` без lock в git; YOLO26 нужен пакет, который эти веса знает. Это зависимость Mac-окружения, не ROS.
- Intel [person-detection](https://huggingface.co/Intel/person-detection) — тот же YOLO26, но **OpenVINO** (CPU/GPU/NPU Intel). На Mac MPS не кладём.

### 3. Крупнее официальные detect: `yolo11s.pt` / `yolo26s.pt`

- Больше ёмкости (YOLO11s: 9.4M / 21.5B FLOPs, mAP 47.0).
- Зачем: nano может не держать домен (низкая камера, indoor).
- Риск: уложится ли инференс в бюджет 100 мс вместе с сетью — **не измерено**. На M3 24 ГБ запас по compute есть; проверять на стенде, не по таблице T4.
- Pose-крупнее (`yolo11s-pose.pt`) ту же ошибку задачи не снимает: всё ещё keypoints.

### 4. Official pose оставить и «дотюнить порог»

- Не новая модель. Имеет смысл только как контроль: если detect на том же кадре стабилен, а pose нет — гипотеза про pose подтверждается.
- Поднимать/опускать `CONFIDENCE_THRESHOLD` без смены весов — уже можно скриптом; отдельной моделью не считается.

## Кандидаты A: person-only веса (не COCO-80)

Идея: голова сети только «человек», NMS не конкурирует с chair/dog. Домен датасета важнее факта «один класс».

### 5. CrowdHuman → YOLOv8n (yakhyo)

- Репозиторий: [yakhyo/yolov8-crowdhuman](https://github.com/yakhyo/yolov8-crowdhuman). Веса: `yolov8n_best.pt` в [Releases](https://github.com/yakhyo/yolov8-crowdhuman/releases/download/weights/yolov8n_best.pt). Заявлен как person, грузится `YOLO()`.
- CrowdHuman — толпа, улица, много мелких людей, часто ещё класс **head**.
- Зачем: узкий класс «человек», много окклюзий.
- Почему слабо для стенда: датасет не indoor и не камера у пола. Мелкий пешеход ≠ человек в 1–3 м перед Aurora.
- Риск для геометрии: если в чекпоинте класс 0 = `head`, `_is_person` пропустит головы (fallback `cls_id == 0`). Низ рамки головы — шея, не стопы; range и nearest сломаются **без смены пайплайна**. Перед прогоном смотреть `model.names`.
- SD013 явно не брал CrowdHuman-форки. Здесь это снова кандидат только как файл весов, не как смена архитектуры.

Другие CrowdHuman на Ultralytics HUB (например [crowdhumanyolov8n640](https://platform.ultralytics.com/vincent-huard/crowdhuman/crowdhumanyolov8n640)): mAP50 ~62% на CrowdHuman val — метрика чужого датасета, не стенда. Те же оговорки про `names` и домен.

### 6. VisDrone / «beach» person (дрон, высота)

- Пример: fine-tune YOLOv8n-person от VisDrone ([описание](https://huggingface.co/Shashank022002/beach-person-detector-yolov8m)). Люди мелкие, вид сверху.
- Для камеры 0.145 м — чужой домен, скорее хуже COCO detect. Не первый A/B.

### 7. Свой fine-tune на кадрах Aurora

- Не готовые веса, готовый **метод**: `YOLO("yolo11n.pt").train(...)` на Mac MPS ([Train / MPS](https://docs.ultralytics.com/modes/train)). На выходе `.pt` → тот же `WEIGHTS`.
- Лучший домен (наш ракурс, свет, 640×400). Нужна разметка. В это исследование как продукт не входит; запасной путь, если A/B на официальных весах не хватит.

Community Ultralytics: дообучение только на «своих людях» без COCO person **ухудшает** общий detect ([тред](https://community.ultralytics.com/t/helping-with-elevating-yolov11s-performance-in-human-detection-task/775)). Если дообучать — мешать кадры стенда с COCO person, не заменять COCO целиком.

## Не берём (ломают контракт или домен рамки)

| Решение | Почему мимо |
|---------|-------------|
| Детекторы **голов** (CrowdHuman head, [Owen718](https://github.com/Owen718/Head-Detection-Yolov8)) | Рамка не туловище; сэмпл низа bbox в `person_perception` рассчитан на тело |
| Roboflow Inference | Отклонён в SD013; другой сервис, не `YOLO().predict` |
| Intel person-detection / OpenVINO | Не MPS, не текущий Python-процесс |
| CoreML / Vision `VNDetectHumanRectanglesRequest` | Смена runtime (ANE), не drop-in Ultralytics |
| YOLOE / YOLO-World (`set_classes(["person"])`) | Другой класс API (`YOLOE`), плюс текст-эмбеддинг; выигрыш vs COCO person для «просто человек» сомнителен |
| `RTDETR()` | Не `YOLO()`; правка конструктора. Имеет смысл только если A/B исчерпаны |
| Nav-YOLO, YOLO-GSD, MR-YOLO (статьи 2024–2025) | Форки архитектуры, нет стабильного `.pt` под наш `predict`; часто CUDA/Jetson |
| Трекер (ByteTrack / F09) | Стабильный id, не появление bbox. Вне этого SD |
| Инференс на Pi, Hailo, вендорский YOLO | Граница SD013 |

## Порядок проверки на стенде (для СА, не сейчас)

Смена только `WEIGHTS` (и при необходимости `classes=[0]`). Один и тот же человек перед камерой, overlay debug, смотреть кадр-к-кадру пропадания рамки. Цифр цели нет — сравниваем с текущим `yolo11n-pose.pt`.

1. Контроль: текущий `yolo11n-pose.pt`, `conf=0.25`.
2. **`yolo11n.pt`** (detect nano) — главная гипотеза «pose vs detect».
3. Если 2 лучше, но ещё дыры: `classes=[0]` в predict.
4. **`yolo26n.pt`**, если пакет Ultralytics его открывает — гипотеза NMS.
5. Если nano мало: `yolo11s.pt` / `yolo26s.pt`, глядя на WARN >100 мс в `person_perception`.
6. CrowdHuman `.pt` — только после проверки `model.names` (нет `head` как класса 0).
7. Свой fine-tune — если 2–6 не закрывают домен низкой камеры.

Не смешивать в одном прогоне смену модели и смену порога/imgsz.

## Вывод для системного анализа

Первый осмысленный шаг — **не CrowdHuman и не новая архитектура**, а официальный **detect** того же nano (`yolo11n.pt`) вместо **pose**. Текущий выбор YOLO11n-pose оптимизировал «один класс в чекпоинте», а не устойчивость bbox. Person-only веса с чужих датасетов (толпа, дрон) для Aurora у пола — второй ряд и с риском сломать геометрию, если в модели голова, а не тело.

Пайплайн Pi↔Mac и контракт `/perception/*` не менять. Допустимые правки Mac: путь весов, при необходимости `classes=` и версия `ultralytics` под YOLO26.
