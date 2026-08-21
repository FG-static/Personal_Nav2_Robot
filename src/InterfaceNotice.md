# Vision 通讯协议说明

## 1. 文档范围

本文只说明 Vision 与下位机之间的二进制通讯规则，包括帧边界、结构体对齐、联合体序列化、字节序、收发流程和对接约束。

本文不定义任何具体业务字段、业务含义、控制量或状态值。双方需要另外维护各方向的 Payload 字段表，但字段表必须遵守本文约定。

## 2. 传输方式

- 传输接口：USB CDC（虚拟串口）。
- 传输内容：原始二进制字节，不是字符串。
- 通讯方向：双向通讯；两个方向可以使用不同的 Payload 和不同的固定帧长。
- 帧边界：当前实现使用固定帧长以及帧头、帧尾共同判断一帧是否合法。

发送端不得把二进制数据转换成十六进制字符串，也不得自动追加换行符、空格或字符串结束符 `\0`。

## 3. 帧格式

一帧数据由以下三部分组成：

| 字节位置 | 长度 | 内容 | 说明 |
| --- | ---: | --- | --- |
| `0` | 1 字节 | `0xAA` | 固定帧头 |
| `1 ... N` | N 字节 | Payload | 双方约定的业务数据区 |
| `N + 1` | 1 字节 | `0x55` | 固定帧尾 |

因此：

```text
总帧长 = 1 字节帧头 + Payload 长度 + 1 字节帧尾
```

当前协议没有长度字段、命令字、版本号、转义机制或 CRC 校验。帧头、帧尾只用于边界检查，不能替代数据完整性校验。

## 4. 数据表示约定

双方必须完全一致地约定每个字段的顺序、类型、长度和含义。不得只依赖 C/C++ 类型在各自平台上的默认大小。

### 4.1 固定宽度类型

线上的字段优先使用以下类型：

| 线上类型 | 长度 | 说明 |
| --- | ---: | --- |
| `uint8_t` / `int8_t` | 1 字节 | 无符号/有符号 8 位整数 |
| `uint16_t` / `int16_t` | 2 字节 | 无符号/有符号 16 位整数 |
| `uint32_t` / `int32_t` | 4 字节 | 无符号/有符号 32 位整数 |
| `float` | 4 字节 | IEEE 754 单精度浮点数 |

不建议直接把 `int`、`long`、裸 `enum` 或 C/C++ `bool` 放进线上协议，因为它们的宽度或表示方式可能随编译器、ABI 和平台变化。

- 布尔量在线上定义为 `uint8_t`，`0` 表示 false，`1` 表示 true；其他值视为非法或保留值。
- 枚举在线上定义为固定宽度整数，例如 `uint8_t`；接收后再转换为本地枚举。
- 如果必须使用 `float`，双方必须确认都是 IEEE 754 32 位单精度格式。

### 4.2 字节序

多字节整数和 `float` 均采用小端序：低有效字节先发送，高有效字节后发送。

例如，`uint16_t` 数值 `0x1234` 在线上的两个字节为：

```text
34 12
```

非小端平台或 Python 等上位机程序必须显式指定小端序，不能使用本机默认字节序。

### 4.3 结构体对齐

协议结构体必须按 1 字节对齐，禁止编译器在字段之间自动插入填充字节：

```cpp
#pragma pack(push, 1)

typedef struct
{
    uint8_t Frame_Header;

    // 以下仅为协议写法示例，不代表实际业务字段。
    uint8_t Field_U8;
    int16_t Field_I16;
    float Field_F32;
    uint8_t Flag_U8;

    uint8_t Frame_Tail;
} Protocol_Frame_t;

#pragma pack(pop)
```

必须用编译期断言锁定帧长，避免后续修改字段或更换编译器后静默改变线上格式：

```cpp
static_assert(sizeof(Protocol_Frame_t) == EXPECTED_FRAME_SIZE,
              "Protocol frame size mismatch");
```

`EXPECTED_FRAME_SIZE` 必须填写双方协议表中确认的总帧长，不能为了让编译通过而直接改成 `sizeof(Protocol_Frame_t)`。

## 5. 联合体的作用

联合体让“字段形式的结构体”和“连续原始字节数组”共享同一块内存：

```cpp
typedef union
{
    Protocol_Frame_t Data;
    uint8_t Raw[sizeof(Protocol_Frame_t)];
} Protocol_Frame_u;
```

两种访问方式的含义如下：

- `Data`：便于程序按字段赋值或读取。
- `Raw`：便于 USB 接口按连续字节发送或接收。

联合体不负责压缩、编码、校验或大小端转换。它只是同一段内存的两种视图，因此结构体字段顺序、字段宽度、1 字节对齐和字节序必须预先统一。

## 6. 发送流程

标准发送流程如下：

1. 清零发送联合体。
2. 写入固定帧头 `0xAA`。
3. 按协议字段表写入 Payload。
4. 写入固定帧尾 `0x55`。
5. 一次性发送 `Raw` 的全部字节，发送长度必须为 `sizeof(Raw)`。

```cpp
Protocol_Frame_u tx = {};

tx.Data.Frame_Header = 0xAAU;

// 按双方确认的字段表填写 Payload。
tx.Data.Field_U8 = field_u8;
tx.Data.Field_I16 = field_i16;
tx.Data.Field_F32 = field_f32;
tx.Data.Flag_U8 = flag ? 1U : 0U;

tx.Data.Frame_Tail = 0x55U;

USB_Transmit_Data(tx.Raw, sizeof(tx.Raw));
```

必须检查底层发送函数的返回值。如果 USB 仍在发送上一帧并返回 busy，本帧不能视为已成功发送，应根据应用需求进行重试、排队或丢弃统计。

## 7. 接收流程

标准接收流程如下：

1. 检查接收指针是否有效。
2. 检查本次接收长度是否恰好等于约定帧长。
3. 把接收到的字节复制到接收联合体的 `Raw`。
4. 检查帧头是否为 `0xAA`、帧尾是否为 `0x55`。
5. 检查 Payload 中布尔量、枚举值和数值范围是否合法。
6. 所有检查通过后，才更新业务层数据和在线状态。

```cpp
void Protocol_Receive_Callback(const uint8_t *buffer, uint16_t length)
{
    static Protocol_Frame_u rx = {};

    if (buffer == nullptr)
    {
        return;
    }

    if (length != sizeof(rx.Raw))
    {
        return;
    }

    memcpy(rx.Raw, buffer, sizeof(rx.Raw));

    if (rx.Data.Frame_Header != 0xAAU ||
        rx.Data.Frame_Tail != 0x55U)
    {
        return;
    }

    // 在此检查 Payload 合法性；全部通过后再交给业务层。
}
```

## 8. 当前实现的分包与粘包约束

当前接收器是严格定长接收器，不包含缓存拼帧或帧头搜索逻辑：

- 少于一帧：直接丢弃，不等待下一批数据补齐。
- 多于一帧：直接丢弃，不从中拆出多帧。
- 帧前存在无关字节：直接丢弃，不向后搜索 `0xAA`。
- 一帧被拆成多次回调：每一段都会因长度不符而被丢弃。
- 多帧合并到一次回调：整批数据会因长度不符而被丢弃。

因此，对接端必须遵守：

- 每次 USB 写操作只发送一帧。
- 每次写操作发送完整帧，长度必须与该方向约定帧长完全一致。
- 不要把多帧拼接后一次发送。
- 不要在帧前后附加换行、调试文本或其他字节。

如果对接环境不能保证以上边界，需要先升级接收器，增加环形缓冲、帧头搜索、定长取帧和异常重同步；这属于协议实现升级，双方必须同步版本后再使用。

## 9. Python 对接模板

Python 端应使用 `struct` 模块并显式指定小端、标准字段宽度和无自动对齐：

```python
import struct

FRAME_HEADER = 0xAA
FRAME_TAIL = 0x55

# “<”表示小端。此处 B、h、f、B 只对应前文的通用示例字段，
# 不代表实际业务协议；正式对接时必须替换为双方确认的 Payload 格式。
PAYLOAD_FORMAT = "<BhfB"
PAYLOAD_SIZE = struct.calcsize(PAYLOAD_FORMAT)
FRAME_SIZE = 1 + PAYLOAD_SIZE + 1


def build_frame(*fields) -> bytes:
    payload = struct.pack(PAYLOAD_FORMAT, *fields)
    return bytes([FRAME_HEADER]) + payload + bytes([FRAME_TAIL])


def parse_frame(frame: bytes):
    if len(frame) != FRAME_SIZE:
        raise ValueError("frame length mismatch")
    if frame[0] != FRAME_HEADER or frame[-1] != FRAME_TAIL:
        raise ValueError("frame boundary mismatch")
    return struct.unpack(PAYLOAD_FORMAT, frame[1:-1])
```

常用 Python `struct` 格式字符：

| C/C++ 线上类型 | Python 格式字符 | 长度 |
| --- | --- | ---: |
| `uint8_t` | `B` | 1 字节 |
| `int8_t` | `b` | 1 字节 |
| `uint16_t` | `H` | 2 字节 |
| `int16_t` | `h` | 2 字节 |
| `uint32_t` | `I` | 4 字节 |
| `int32_t` | `i` | 4 字节 |
| `float` | `f` | 4 字节 |

不要使用 Python 原生布尔格式作为未约定的线上类型；布尔量按 `B` 打包为 `0` 或 `1`。

## 10. 双方对接前必须确认的内容

每个通讯方向都应单独确认以下项目：

- Payload 中每个字段的名称、顺序、固定宽度类型、单位、比例和合法范围。
- 总帧长，以及 C/C++ 的 `static_assert` 和 Python 的 `struct.calcsize()` 是否一致。
- 多字节字段是否按小端序处理。
- 布尔量是否只使用 `0` 和 `1`。
- 枚举值是否使用固定宽度整数，未定义值如何处理。
- 浮点数是否为 IEEE 754 32 位单精度。
- 每次底层写操作是否只包含一帧完整数据。
- 错误长度、错误帧头、错误帧尾和非法 Payload 是否都会被拒绝。
- 双方是否使用同一组已知字节流完成过发送、抓包和解析对照测试。

## 11. 协议能力边界

当前协议简单、开销小，但需要明确以下边界：

- 没有 CRC，不能可靠发现 Payload 位错误。
- 没有长度字段，不支持同一接收器内的可变长度帧。
- 没有命令字，不适合在同一方向混合多种帧类型。
- 没有版本号，修改字段顺序、类型或长度时无法自动识别版本不兼容。
- 没有转义机制，当前依赖固定长度确定帧尾位置，Payload 中出现 `0xAA` 或 `0x55` 本身不构成错误。

如果后续要扩展协议，应由双方共同定义新版本，建议至少加入版本、命令、Payload 长度和 CRC；不能由单方直接改变现有帧结构。

