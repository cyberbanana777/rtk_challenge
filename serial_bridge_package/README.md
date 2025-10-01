# ROS2 Serial Bridge Package

Пакет для двусторонней связи между ROS2 и микроконтроллерами через последовательный порт с поддержкой одометрии, телеметрии и управления движением.

## 📁 Структура пакета

```
serial_bridge_package/
├── launch/                          # Файлы запуска
│   └── serial_bringup.launch.py    # Основной launch-файл
├── rviz/
│   └── show_all_stats.rviz         # Конфигурация RViz для визуализации
├── serial_bridge_package/          # Исходный код
│   ├── serial_bridge_node.py       # Мост между ROS и UART
│   ├── twist_to_command.py         # Конвертер Twist в команды
│   └── feedback_processor.py       # Обработчик телеметрии
├── package.xml                     # Метаданные пакета
├── setup.py                        # Скрипт установки
└── test/                          # Тесты
```

## 🚀 Узлы (Nodes)

### 1. `serial_bridge_node`
**Мост между ROS и последовательным портом**
- Двунаправленная связь с устройствами UART (ESP32 и др.)
- Буферизация и парсинг сообщений в формате `$данные#`
- Параметры настройки порта (port, baudrate, timeout)

**Топики:**
- Публикует: `low_level/serial/feedback` (сырые данные с устройства)
- Подписывается: `low_level/serial/cmd` (команды на устройство)

### 2. `twist_to_command`
**Конвертер сообщений Twist в протокол устройства**
- Преобразует `geometry_msgs/Twist` в формат `$linear;angular#`
- Использует стандартный топик `/cmd_vel`
- Извлекает только линейную скорость X и угловую Z

**Топики:**
- Подписывается: `/cmd_vel` (стандартный топик управления)
- Публикует: `low_level/esp32_input` → `low_level/serial/cmd`

### 3. `feedback_processor`
**Обработчик телеметрии и данных с датчиков**
- Парсит данные в формате `$data1;...;data13#`
- Публикует одометрию, IMU, температуру, напряжение, нагрузку
- Отправляет трансформации через TF2

**Публикуемые топики:**
- `/odom` - одометрия
- `/imu/data` - данные инерциальной системы
- `/sensors/wheel/{left,right}/temperature` - температура моторов
- `/sensors/wheel/{left,right}/voltage` - напряжение
- `/sensors/wheel/{left,right}/load` - нагрузка
- `/sensors/wheel/{left,right}/position` - позиция вала

**TF трансформации:**
- `odom` → `base_link`
- Публикация позиции колес

## 📋 Форматы данных

### Исходящие команды (ROS → Устройство)
```
$linear_velocity;angular_velocity#
```
Пример: `$0.5;-0.3#`

### Входящие данные (Устройство → ROS)
```
$x_pos;y_pos;theta;vx;vth;left_pos;right_pos;left_load;right_load;left_temp;right_temp;left_volt;right_volt#
```

## 🛠 Установка и запуск

### Сборка пакета
```bash
cd ~/mirea_jetbots/low_level_driver_dynamixel/src
colcon build --packages-select serial_bridge_package
source install/setup.bash
```

### Запуск всего стека
```bash
ros2 launch serial_bridge_package serial_bringup.launch.py
```

### Запуск отдельных узлов
```bash
# Мост Serial-ROS
ros2 run serial_bridge_package serial_bridge_node

# Конвертер команд
ros2 run serial_bridge_package twist_to_command

# Обработчик телеметрии
ros2 run serial_bridge_package feedback_processor
```

## ⚙️ Настройка параметров

### Параметры serial_bridge_node:
- `port`: последовательный порт (по умолчанию `/dev/ttyUSB0`)
- `baudrate`: скорость (по умолчанию 115200)
- `timeout`: таймаут чтения (по умолчанию 1 сек)

### Параметры feedback_processor:
- `frame_id`: система координат одометрии (по умолчанию `odom`)
- `child_frame_id`: система координат робота (по умолчанию `base_link`)
- `left/right_wheel_frame_id`: системы координат колес

## 📊 Визуализация в RViz

Для просмотра всех данных используйте конфигурацию:
```bash
ros2 run rviz2 rviz2 -d install/serial_bridge_package/share/serial_bridge_package/rviz/show_all_stats.rviz
```

**Отображаемые элементы:**
- Позиция и ориентация робота (Odometry)
- TF трансформации
- Данные с датчиков (через отдельные топики)

## 🔧 Требования

- ROS2 Humble или новее
- Python 3.8+
- Библиотека `pyserial`
- Права доступа к последовательному порту

## Лицензия

Copyright (c) 2025 Алиса Зенина и Александр Грачев РТУ МИРЭА (Россия)

Данное программное обеспечение распространяется под [лицензией MIT](LICENSE).
Разрешается свободное использование, копирование, модификация и распространение при условии сохранения уведомления об авторских правах и текста лицензии.