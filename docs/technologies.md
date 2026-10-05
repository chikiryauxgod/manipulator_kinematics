# Каталог возможного развития после учебной работы

Статус: отложенные направления. Этот каталог сохраняет обсуждённые технологии, но не требует их установки или реализации для сдачи. Текущая задача описана в [требованиях](requirements.md), обязательный стек — C++17, CMake, консоль и математическая библиотека.

Ближайшее возможное аппаратное расширение — [МК и Modbus RTU по RS-232](embedded.md). Оно не требует остального каталога.

## Направления

| Направление | Технологии-кандидаты | Условие возвращения к теме |
| --- | --- | --- |
| Робототехническая платформа | ROS 2, DDS/QoS, MoveIt 2, ros2_control, URDF/Xacro, SRDF, TF2, RViz2 | Понадобятся сцена, траектории и интеграция робота |
| Пользовательский интерфейс | TypeScript, Three.js, HTTP/OpenAPI, WebSocket | Появится задача удалённого управления/визуализации |
| Внешняя телеметрия и задания | MQTT 5, Eclipse Paho, брокер | Появится внешний потребитель или диспетчер |
| Связь ноутбук–МК | Modbus RTU по RS-232; libmodbus на ноутбуке — кандидат | Этап МК-2: удалённый расчёт IK, отдельная карта регистров |
| Промышленная периферия | Modbus TCP/RTU, RS-485 | Выбрано устройство с документированной картой регистров |
| SCADA/MES | OPC UA, open62541, OPC UA Robotics, PubSub | Появится промышленная система интеграции |
| Управление приводами | EtherCAT, CAN/CAN FD, CANopen/CiA 402 | Выбраны приводы и временные требования |
| Заводские контроллеры | PROFINET/PROFIdrive, EtherNet/IP/CIP Motion, SRCI | Подтверждена экосистема ПЛК/робота |
| Прочие промышленные сети | PROFIBUS, CC-Link IE, POWERLINK, Sercos, J1939, HART | Понадобится конкретное совместимое оборудование |
| Периферия и электроника | IO-Link, RS-232/UART/USB/TCP/UDP, SPI/I²C/GPIO/PWM, энкодеры | Проектируется аппаратная часть |
| Встроенное ПО | MCU, FreeRTOS/Zephyr, micro-ROS | Потребуется собственный контроллер |
| Аппаратное время | PREEMPT_RT, PTP, аппаратная синхронизация | Установлены измеримые бюджеты цикла |
| Функциональная безопасность | Safety PLC, аппаратные разрешения, PROFIsafe/FSoE/CIP Safety | Отдельный проект реального оборудования |
| Физическая симуляция | Gazebo; Isaac Sim/MuJoCo под специализированные задачи | Нужна физика, контакты или обучение |
| Восприятие | OpenCV/PCL, RGB-D, GenICam/GigE Vision/USB3 Vision, hand–eye | Появятся камеры и цель по наблюдениям |
| Силовое взаимодействие | Датчики силы/момента, impedance/admittance control | Появится контактная задача |
| ML | Python, PyTorch, ONNX Runtime, CUDA/TensorRT по аппаратуре | Определены данные и метрики модели |
| Сценарии и парк роботов | Автоматы/Behavior Trees, VDA 5050 для мобильных роботов | Появится диспетчеризация/мобильная база |
| Серверная инфраструктура | gRPC/Protobuf, Kafka/AMQP, SQLite/PostgreSQL | Есть конкретная задача API или хранения |
| Эксплуатация | rosbag2/MCAP, метрики, логи, трассировка, CI, systemd/контейнеры | Масштаб приложения оправдывает инструменты |
| Доступ | TLS/mTLS, ACL, роли, сертификаты, разделение сетей | До включения сетевого управления |

## Архитектурный принцип будущих интеграций

Математическое ядро сохраняет независимость от транспорта. MQTT/OPC UA/HTTP передают прикладные команды; драйверы Modbus и других протоколов реализуют конкретные возможности устройств. Для физического движения вводятся отдельные планирование, исполнение и проверка состояния. Решение обратной кинематики само по себе не является траекторией или командой приводу.

Паспорт будущего адаптера должен определить роль, версию, устройство, capabilities, единицы, карту данных, дедлайны, повторы, восстановление и проверку фактического результата. MQTT QoS не заменяет идемпотентность прикладной команды; Modbus client и Modbus server — разные роли. Универсальный send(bytes) не заменяет предметный контракт.

## Справочные источники

- [ROS 2 RMW](https://docs.ros.org/en/jazzy/Installation/RMW-Implementations.html), [MoveIt](https://moveit.picknik.ai/main/index.html), [Gazebo](https://gazebosim.org/docs/harmonic/ros_installation/).
- [MQTT](https://mqtt.org/), [Modbus](https://www.modbus.org/modbus-specifications), [OPC UA Robotics](https://reference.opcfoundation.org/specs/OPC-40010-1/full).
- [EtherCAT](https://www.ethercat.org/en/technology.html), [CiA 402](https://www.can-cia.org/can-knowledge/cia-402-series-canopen-device-profile-for-drives-and-motion-control), [PROFINET](https://www.profibus.com/profinet-technology), [EtherNet/IP](https://www.odva.org/technology-standards/key-technologies/ethernet-ip/).
- [GenICam](https://www.emva.org/standards-technology/genicam/), [VDA 5050](https://www.vda.de/en/news/publications/publication/vda-5050).

Версии и совместимость проверяются при возвращении к направлению. Наличие технологии в каталоге не означает её поддержку проектом.
