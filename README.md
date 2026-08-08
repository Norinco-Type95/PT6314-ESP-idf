# PT6314 ESP-IDF Component

原生 ESP-IDF 的 Princeton PT6314 点阵字符 VFD 控制器驱动。驱动使用
ESP-IDF GPIO API 和 FreeRTOS 静态互斥锁，不依赖 Arduino Framework、Arduino
兼容层、`String` 或 `Print`。

当前版本已经在 ESP32-C3、ESP-IDF v6.0.2 和 20×2 VFM202MDA VFD 上完成实机
验证，包括初始化、双行显示、DDRAM、亮度、清屏、原始字符码和连续刷新。

## 特性

- 原生 C API 和 `esp_err_t` 错误返回
- GPIO 引脚、逻辑列边界、行模式和初始亮度可配置
- PT6314 三线串行写入：SCK、STB、SI/SO
- DDRAM 定位和连续字符写入
- 8 个 5×8 CGRAM 自定义字符槽
- 25%、50%、75%、100% 四档硬件亮度
- 原始 8 位字符码透传，不强制绑定某一 CGROM 版本
- 静态 FreeRTOS mutex，无动态内存分配
- 多字节、字符串和 CGRAM 操作期间保持完整事务锁

## 目录结构

```text
pt6314/
├── include/
│   └── pt6314.h
├── .gitattributes
├── .gitignore
├── CMakeLists.txt
├── LICENSE
├── pt6314.c
└── README.md
```

## 添加到 ESP-IDF 工程

将整个目录复制到工程的 `components/pt6314`：

```text
your_project/
├── components/
│   └── pt6314/
├── main/
└── CMakeLists.txt
```

应用组件的 `CMakeLists.txt` 添加依赖：

```cmake
idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES pt6314
)
```

## 最小示例

```c
#include "esp_err.h"
#include "pt6314.h"

void app_main(void)
{
    const pt6314_config_t config = {
        .clk_pin = GPIO_NUM_2,
        .data_pin = GPIO_NUM_3,
        .stb_pin = GPIO_NUM_7,
        .columns = 20,
        .lines = 2,
        .brightness = 100,
    };

    ESP_ERROR_CHECK(pt6314_init(&config));
    ESP_ERROR_CHECK(pt6314_clear());
    ESP_ERROR_CHECK(pt6314_set_cursor(0, 0));
    ESP_ERROR_CHECK(pt6314_write_string("HELLO"));
}
```

`pt6314_init()` 会复制配置内容。所有 API 必须从 FreeRTOS task 上下文调用，不能
在 ISR 中调用。初始化成功后单次 API 调用由内部 mutex 保护；`set_cursor()` 后再
调用 `write_string()` 属于两个独立事务，复杂页面仍建议由一个 Display Manager
任务统一串行化。

`columns` 仅用于 `pt6314_set_cursor()` 的软件边界检查，`lines` 会写入 Function
Set 的行模式位。物理 GRID 数、扫描占空比和面板连接仍由 PT6314 的 DLS、
DS1/DS0、RL 等硬件绑定位决定，必须与实际 VFD 模组匹配。

## 主要 API

| API | 用途 |
| --- | --- |
| `pt6314_init()` | 配置 GPIO 并执行初始化序列 |
| `pt6314_clear()` | DDRAM 填充空格并回到地址 0 |
| `pt6314_home()` | 地址和显示移位归零，不清除 DDRAM |
| `pt6314_display_on/off()` | 开关显示，保留 DDRAM 内容 |
| `pt6314_set_cursor()` | 设置零基列、行位置 |
| `pt6314_set_brightness()` | 设置四档硬件亮度 |
| `pt6314_write_byte()` | 写入一个原始字符码 |
| `pt6314_write_bytes()` | 持锁串行写入一组原始字节；失败时可能只写入前缀 |
| `pt6314_write_string()` | 写入 NUL 结尾的单字节字符串 |
| `pt6314_create_char()` | 写入一个 5×8 CGRAM 字符 |
| `pt6314_send_raw()` | 发送底层 Instruction/Data 帧 |

亮度参数沿用已验证 Arduino 驱动的百分比阈值：

- 0～25：25%
- 26～50：50%
- 51～75：75%
- 76～255：100%

亮度 0 不代表熄灭；需要熄灭时使用 `pt6314_display_off()`。

## 串行协议与时序

每次写入由一个完整的 16-bit 帧组成：

```text
STB LOW
  ├── Start Byte: 0xF8（Instruction）或 0xFA（Data）
  └── 8-bit Instruction/Data，D7 first
STB HIGH
```

SCK 空闲为高电平，SI 在 SCK 上升沿被 PT6314 锁存。实现继续使用已经验证的
GPIO bit-bang 边沿顺序，没有改成硬件 SPI，也没有在一个 STB 周期内批量发送
多个数据字节。

驱动采用 2 µs 软件相位裕量：

| 参数 | PT6314 V1.5 最小值 | 当前实现下限（不含 GPIO 开销） |
| --- | ---: | ---: |
| SCK 周期 | 500 ns | 约 6 µs |
| SCK 高/低脉宽 | 各 200 ns | 高约 4 µs、低 2 µs |
| 数据建立/保持 | 各 100 ns | 约 4 µs / 2 µs |
| STB↓ 到首个 SCK↓ | 100 ns | 约 4 µs |
| 最后 SCK↑ 到 STB↑ | 500 ns | 约 4 µs |
| 帧间 STB 高电平 | 500 ns | 2 µs |
| Start/Data 字节边界 `tWAIT` | 表 11.3 为 1 ns，读时序图为 1 µs | 约 6 µs |

FreeRTOS 抢占只会延长脉冲；数据手册没有给出这些写入时序的最大间隔，因此驱动
不会为一个约 100 µs 的帧关闭中断。

## DDRAM、CGRAM 与字符集

- 一行模式 DDRAM 地址范围为 `0x00..0x4F`。
- 两行模式地址范围为第一行 `0x00..0x27`、第二行 `0x40..0x67`。
- 一个 PT6314 提供 24 个 GRID 输出；40 列两行需要数据手册描述的外部 GRID 扩展。
- CGRAM 提供 8 个槽，每槽 8 行；每行有效数据为 bit 4..0。
- `pt6314_create_char()` 后地址仍指向 CGRAM，调用者应再次调用
  `pt6314_set_cursor()` 后再写显示文字。
- PT6314-001、-002、-007、-008 的内置 CGROM 不同。驱动不做字符重映射，
  UTF-8 多字节文本也不会自动转换。

## 电气注意事项

裸芯片或自制控制板还应确认以下硬件条件：

- 串行模式下将 `IFSEL=LOW`、`/CS=LOW`，`/RESET` 在正常运行时保持高电平。
- DLS、DS1/DS0、RL 等绑定位按实际面板配置；其他未用数字输入按数据手册要求
  固定为 H 或 L，不要随意悬空。ESP32-C3 与 PT6314 必须共地。
- 数据手册 V1.5 对 `TEST` 正常电平的不同章节存在矛盾；使用成品模组时保留其
  已验证接法，自制板应结合芯片版本、参考原理图和实测确认。
- SCK、STB、SI/SO 三个 GPIO 应由本组件独占，不能同时被其他外设驱动。

PT6314 数据手册的逻辑电气参数以 VDD1=5 V 为条件：

- SI、STB 等普通逻辑输入的保证高电平为至少 3.5 V。
- SCK 的保证高电平为至少 4.0 V。
- 串行输入边沿要求上升和下降时间小于 15 ns。

因此，ESP32-C3 的 3.3 V GPIO 直接连接虽然可能在个别样机上正常工作，但不在
PT6314 最坏条件保证范围内。正式硬件建议使用能够接受 3.3 V 输入并输出 5 V、
且边沿足够快的逻辑缓冲器（例如合适的 AHCT 系列器件）。不建议使用为 I²C
设计的慢速 BSS138 双向电平转换模块。

当前驱动只写 SI/SO。如果以后实现读取，必须增加 5 V 到 3.3 V 的输入保护或
双向电平转换，不能将 PT6314 的 5 V 输出直接接入 ESP32-C3。

## 设计边界

- 当前为单实例驱动。
- 没有 `deinit()`；初始化成功后不能在运行期更换 GPIO 或显示几何。
- 不支持 ISR 调用。
- 不负责 BLE、Wi-Fi、页面切换或上层显示管理。
- 不维护软件 framebuffer；显示内容直接写入 PT6314 DDRAM。
- 接口没有 ACK 或显示读回；`ESP_OK` 仅表示本地 GPIO 帧发送完成。
- `pt6314_send_raw()` 可改变底层状态。Raw Function Set 若改变行模式，可能与
  初始化时配置的几何边界不一致。

## 构建

本组件已使用 ESP-IDF v6.0.2 和 ESP32-C3 工具链完整构建。放入工程后运行：

组件使用静态 FreeRTOS mutex，因此工程配置必须启用
`CONFIG_FREERTOS_SUPPORT_STATIC_ALLOCATION=y`（ESP-IDF 常规配置通常已启用）。

```text
idf.py set-target esp32c3
idf.py build
```

## License

本项目使用 MIT License，完整文本见 [`LICENSE`](LICENSE)。
