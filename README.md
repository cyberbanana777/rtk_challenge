## Полезные ссылки
[Официальная документация к моторам](https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/)

## Подробное описание модулей репозитория
1. [Описание ROS2 пакета для реализации сообщения по serial-порту](serial_bridge_package/README.md)
2. [Описание прошивки для ESP32](esp_firmware/README.md)

## 📨 Структура сообщений

### Сообщение, полученное от ESP32

Данное сообщение содержит текущее состояние робота.

**Формат:**
`
$<x_position>;<y_position>;<omega_angle>;<x_real_linear_velocity>;<z_real_angular_velocity>;<left_wheel_position>;<right_wheel_position>;<left_wheel_load>;<right_wheel_load>;<left_wheel_temperature>;<right_wheel_temperature>;<left_wheel_voltage>;<right_wheel_voltage>;#
`

**Описание полей:**

| Поле | Описание | Единица измерения |
|:---|:---|:---|
| `x_position` | Позиция по оси X | метры (м) |
| `y_position` | Позиция по оси Y | метры (м) |
| `omega_angle` | Угол ориентации (рыскание) | радианы (рад) |
| `x_real_linear_velocity` | Реальная линейная скорость | м/с |
| `z_real_angular_velocity` | Реальная угловая скорость | рад/с |
| `left_wheel_position` | Позиция левого колеса (энкодер?) | импульсы |
| `right_wheel_position` | Позиция правого колеса (энкодер?) | импульсы |
| `left_wheel_load` | Нагрузка на левый двигатель | %? |
| `right_wheel_load` | Нагрузка на правый двигатель | %? |
| `left_wheel_temperature` | Температура левого двигателя | °C |
| `right_wheel_temperature` | Температура правого двигателя | °C |
| `left_wheel_voltage` | Напряжение левого двигателя | Вольты (В) |
| `right_wheel_voltage` | Напряжение правого двигателя | Вольты (В) |

---

### Сообщение, отправляемое на ESP32

Данное сообщение задает целевые скорости для движения робота.

**Формат:**
`$<x_linear_velocity>;<z_angular_velocity>#`
text

**Описание полей:**

| Поле | Описание | Единица измерения |
|:---|:---|:---|
| `x_linear_velocity` | Заданная линейная скорость | м/с |
| `z_angular_velocity` | Заданная угловая скорость | рад/с |


## Структура конечной системы
![graph](docs/graph.png)