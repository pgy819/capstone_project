# Antigravity Global Workspace Rules (Stage 1: 2-Axis Visual Servoing)

## 1. System Overview & Target
- **Objective**: Track a `"cell phone"` (COCO class label) using Raspberry Pi 5 (Hailo-8 YOLO) and keep it at the virtual screen center `(0, 0)` by driving a 2-axis Pan-Tilt mount (Feetech STS3215 servos) via STM32 NUCLEO-F401RE.
- **Scope Restriction (Stage 1 Only)**: Focus strictly on the upper 2-axis (Pan: ID `2`, Tilt: ID `1` — team assignment). Do NOT include BNO085 IMU SPI code or lower 3-axis parallel kinematics in Stage 1.
- **Languages**: real-time path C++20 on the Pi (`rpi/`), C on the STM32 (`stm32_fw/`). Analysis tools in Python.

## 2. Data Contract (packet v2, 16-Byte frames, CRC-16)
Single source of truth: `stm32_fw/include/link_proto.h` + `stm32_fw/src/link_proto.c`, compiled by BOTH the
firmware and the Pi runtime (`rpi/CMakeLists.txt`). Never re-declare the layout elsewhere.
- Downlink AIM (Pi -> STM32, every camera frame): `AA 55 | 0x21 | seq | mode | flags | err_pan i16 | err_tilt i16 | rate_pan i16 | rate_tilt i16 | crc16`
  - `mode`: 0 HOLD / 1 FOLLOW / 2 PREDICT / 3 JOG (rate_* = commanded rate) / 4 HOME. `flags`: bit0 OPERATOR_HOLD, bit1 ESTOP_REQ (latched), bit2 ESTOP_CLEAR (software ESTOP only).
  - `err_*`: target angle off the camera axis, +right / +up, unit 1/16 servo count (2*pi/65536 rad). `rate_*`: servo count/s.
- Uplink STATUS (STM32 -> Pi, 50 Hz): `AA 56 | 0x81 | seq_echo | guard | flags | pan_goal u16 | tilt_goal u16 | rx_ok u16 | rx_err u16 | crc16`
  - status `flags`: bit0 ESTOP_B1, bit1 COMM_LOST, bit2 ESTOP_SW, bit3 OPERATOR_HOLD, bit4/5 Pan/Tilt answered the boot READ, bit6 boot check done.
- CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) over bytes 2..13, little-endian. Parser = sliding-window frame finder (resynchronises after any slip).
- The v1 8-byte packet (`FF FE status err_x err_y sum`) is retired.

## 3. Hardware Pinout & DMA Mapping Contract (STM32 NUCLEO-F401RE)
Physical wiring is inherited from the team firmware (hyunjinscode `ball5_2_best.ino`) and is FIXED (mechanics built).
- `USART1` (Feetech STS3215 Bus, FE-URT-1 adapter): `PA9` (TX), `PA10` (RX) @ `1000000 bps`, 8N1.
  - `USART1_TX` uses **`DMA2_Stream7`, `DMA_CHANNEL_4`**, `DMA_NORMAL`. `DMA2_Stream7_IRQn` and `USART1_IRQn` enabled so `HAL_UART_TxCpltCallback` fires after `UART_FLAG_TC`.
- `USART6` (Pi 5 link): `PC6` (TX -> Pi GPIO15 RXD), `PC7` (RX <- Pi GPIO14 TXD) @ `115200 bps`, 8N1, common GND.
  - `USART6_RX` uses **`DMA2_Stream1`, `DMA_CHANNEL_5`**, `DMA_CIRCULAR`; `USART6_TX` (uplink) uses **`DMA2_Stream6`, `DMA_CHANNEL_5`**, `DMA_NORMAL`. Never DMA1 for USART1/6 on F401.
- `USART2` (ST-LINK Debug Virtual COM Port): `PA2` (TX), `PA3` (RX) @ `115200 bps`, 8N1.
- `GPIO_Output` (Half-Duplex Direction Control): `PA4` (High before bus TX, Low in `HAL_UART_TxCpltCallback`); harmless with the auto-direction adapter.
- `GPIO_Output` (Heartbeat / Status LED): `PA5` (On-board LD2 green LED).
- `GPIO_Input`: `PC13` (On-board B1, Active-Low): held at boot = servo ID commissioning 1 -> 2 (Pan); pressed while running = ESTOP (latched until RESET). Software ESTOP: link flag `ESTOP_REQ`.
- Boot: both servos are written to operating mode 0 (position) — the team firmware used wheel mode.
- `TIM2` (Hard Real-Time Control Loop): Exactly `500 Hz` (`2.0 ms` period, APB1 Timer Clock = `84 MHz`, `Prescaler = 84 - 1`, `Period = 2000 - 1`, `TIM2_IRQn` enabled).
- NVIC: bus USART1/DMA2_S7 = 1, TIM2 = 2, Pi link USART6/DMA2_S1/DMA2_S6 = 3.

## 4. Strict Embedded & Coding Guardrails (NEVER VIOLATE)
1. **Complete ISR Vector Table**: You MUST implement `SysTick_Handler` (calling `HAL_IncTick()`), `TIM2_IRQHandler`, `DMA2_Stream7_IRQHandler`, `DMA2_Stream1_IRQHandler`, `DMA2_Stream6_IRQHandler`, `USART1_IRQHandler` and `USART6_IRQHandler`. Never leave interrupt handlers unlinked.
2. **Zero Blocking in ISRs**: Never call `HAL_Delay()`, `printf()`, or blocking `HAL_UART_Transmit()` inside `TIM2_IRQHandler` or any DMA callback.
3. **Volatile & Atomic Enforcement**: Every global variable/struct shared between the link parser (main loop) and `TIM2_IRQHandler` MUST be declared `volatile`. Updating multi-byte shared variables (`g_vision_pkt`, `g_new_pkt_flag`, `g_comm_watchdog`) in the background parser MUST be wrapped in a critical section (`__disable_irq()` / `__enable_irq()`).
4. **Byte-Slip Immunity**: Never cast raw DMA buffer bytes to a struct. Feed bytes one by one to `Link_ParseByte` (sliding window, `AA AA 55` safe, CRC-16 checked); after the CRC, also apply the angle/rate sanity gate in `protocol.c`.
5. **22-Byte Sync Write Exactness**: The Feetech STS3215 `SYNC WRITE (0x83)` packet for 2 servos is strictly **22 bytes** (`Length = 0x12`). Moreover, the TX buffer (`static uint8_t tx_buf[22]`) MUST be declared `static` or global so it remains valid in memory during asynchronous DMA transfer.
6. **No Dynamic Allocation**: Never use `malloc()` or `free()` in `stm32_fw/`.
7. **Verification First**: `stm32_fw/host_test` (test_guard.c, test_link.c), `rpi` `rcws_tests`, and `pio run` (if PlatformIO CLI is available) before marking tasks complete.