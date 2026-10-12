# Antigravity Global Workspace Rules (Stage 1: 2-Axis Visual Servoing)

## 1. System Overview & Target
- **Objective**: Track a `"cell phone"` (COCO class label) using Raspberry Pi 5 (Hailo-8 YOLO) and keep it at the virtual screen center `(0, 0)` by driving a 2-axis Pan-Tilt mount (Feetech STS3215 servos) via STM32 NUCLEO-F401RE.
- **Scope Restriction (Stage 1 Only)**: Focus strictly on the upper 2-axis (Pan: ID `1`, Tilt: ID `2`). Do NOT include BNO085 IMU SPI code or lower 3-axis parallel kinematics in Stage 1.

## 2. Immutable Data Contract (8-Byte UART Packet)
Python (`rpi5_vision/uart_tx.py`) and C (`stm32_fw/include/protocol.h`) MUST share this exact binary layout (Little-Endian, 8 Bytes total):
- `Byte 0`: `0xFF` (Header 1)
- `Byte 1`: `0xFE` (Header 2)
- `Byte 2`: `uint8_t status` (`0x00`: Target Lost, `0x01`: Detected, `0x02`: Kalman Coasting)
- `Byte 3..4`: `int16_t err_x` (Predicted pixel error X from center `960`, range: `-960` to `+960`)
- `Byte 5..6`: `int16_t err_y` (Predicted pixel error Y from center `540`, range: `-540` to `+540`)
- `Byte 7`: `uint8_t checksum` = `(status + (uint8_t)((uint16_t)err_x & 0xFF) + (uint8_t)(((uint16_t)err_x >> 8) & 0xFF) + (uint8_t)((uint16_t)err_y & 0xFF) + (uint8_t)(((uint16_t)err_y >> 8) & 0xFF)) & 0xFF`

## 3. Hardware Pinout & DMA Mapping Contract (STM32 NUCLEO-F401RE)
- `USART1` (Pi 5 GPIO14/15 <-> STM32): `PA9` (TX), `PA10` (RX) @ `115200 bps`, 8N1.
  - **CRITICAL DMA MAPPING**: `USART1_RX` MUST use **`DMA2_Stream2` (or `DMA2_Stream5`), `DMA_CHANNEL_4`** in `DMA_CIRCULAR` mode. (Never use `DMA1` for `USART1` on STM32F401).
- `USART2` (ST-LINK Debug Virtual COM Port): `PA2` (TX), `PA3` (RX) @ `115200 bps`, 8N1.
- `USART6` (Feetech STS3215 Bus Driver): `PC6` (TX), `PC7` (RX) @ `1000000 bps (1Mbps)`, 8N1.
  - **CRITICAL DMA MAPPING**: `USART6_TX` MUST use **`DMA2_Stream6`, `DMA_CHANNEL_5`** in `DMA_NORMAL` mode. Both `DMA2_Stream6_IRQn` and `USART6_IRQn` MUST be enabled in NVIC so `HAL_UART_TxCpltCallback` fires after `UART_FLAG_TC`.
- `GPIO_Output` (Half-Duplex Direction Control): `PA4` (Set High before `USART6` TX, Set Low inside `HAL_UART_TxCpltCallback`).
- `GPIO_Output` (Heartbeat / Status LED): `PA5` (On-board LD2 green LED).
- `GPIO_Input` (Servo ID Commissioning Button): `PC13` (On-board B1 blue user button, Active-Low, Pull-Up/None).
- `TIM2` (Hard Real-Time Control Loop): Exactly `500 Hz` (`2.0 ms` period, APB1 Timer Clock = `84 MHz`, `Prescaler = 84 - 1`, `Period = 2000 - 1`, `TIM2_IRQn` enabled).

## 4. Strict Embedded & Coding Guardrails (NEVER VIOLATE)
1. **Complete ISR Vector Table**: You MUST implement `SysTick_Handler` (calling `HAL_IncTick()`), `TIM2_IRQHandler`, `DMA2_Stream6_IRQHandler`, and `USART6_IRQHandler`. Never leave interrupt handlers unlinked.
2. **Zero Blocking in ISRs**: Never call `HAL_Delay()`, `printf()`, or blocking `HAL_UART_Transmit()` inside `TIM2_IRQHandler` or any DMA callback.
3. **Volatile & Atomic Enforcement**: Every global variable/struct shared between the `USART1` parser and `TIM2_IRQHandler` MUST be declared `volatile`. Updating multi-byte shared variables (`g_vision_pkt`, `g_new_pkt_flag`, `g_comm_watchdog`) in the background parser MUST be wrapped in a critical section (`__disable_irq()` / `__enable_irq()`).
4. **Byte-Slip & Sticky-0xFF Immunity**: Never cast raw DMA buffer bytes directly to `VisionPacket_t*`. Parse byte-by-byte using the State Machine (`WAIT_FF` -> `WAIT_FE` -> `PAYLOAD`). If `0xFF` arrives while in `WAIT_FE`, stay in `WAIT_FE`.
5. **22-Byte Sync Write Exactness**: The Feetech STS3215 `SYNC WRITE (0x83)` packet for 2 servos is strictly **22 bytes** (`Length = 0x12`). Moreover, the TX buffer (`static uint8_t tx_buf[22]`) MUST be declared `static` or global so it remains valid in memory during asynchronous DMA transfer.
6. **No Dynamic Allocation**: Never use `malloc()` or `free()` in `stm32_fw/`.
7. **Verification First**: Run `python3 rpi5_vision/test_pipeline.py` (including C parser cross-verification via `gcc`/`ctypes` if host compiler is available) and `pio run` (if PlatformIO CLI is available) in the Antigravity terminal before marking tasks complete.