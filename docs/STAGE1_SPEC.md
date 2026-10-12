# Stage 1 Detailed Engineering Specification

## Module A: Raspberry Pi 5 Vision & Kalman Predictor (`rpi5_vision/`)

### 1. Coordinate System & Error Definition (`tracker_node.py`)
- Virtual Reference Resolution: `1920 x 1080` (FHD)
- Virtual Origin `(0, 0)`: Screen center `(960, 540)`
- Target Filter: Only accept detections where `label == "cell phone"` and `confidence >= 0.45`.
- Coordinate Normalization Rule: Hailo GStreamer bounding boxes (`bbox.xmin()`, `bbox.ymin()`, `bbox.width()`, `bbox.height()`) return normalized values in `[0.0, 1.0]`.
- Raw Pixel Error Calculation:
  - `raw_ex = int((xmin + width / 2.0) * 1920.0) - 960`
  - `raw_ey = int((ymin + height / 2.0) * 1080.0) - 540`
- Target Locking: If multiple `"cell phone"` boxes exist, select the one with the minimum Euclidean distance to the previous frame's predicted position `(pred_ex, pred_ey)` (or `(0, 0)` if previously lost).

### 2. 2D Constant-Velocity Kalman Filter & Lead Compensation (`kalman_filter.py`)
- State Vector: `X = [Ex, Ey, Vx, Vy]^T` (4 states: pixel error in px and pixel velocity in px/s)
- Measurement Vector: `Z = [raw_ex, raw_ey]^T` (2 measurements)
- Measurement Matrix `H`: `[[1, 0, 0, 0], [0, 1, 0, 0]]`
- Covariance & Noise Matrices (Explicit Initialization in `np.float32`):
  - Initial State Covariance `P0`: `np.diag([100.0, 100.0, 1000.0, 1000.0])`
  - Process Noise Covariance `Q`: `np.diag([2.0, 2.0, 50.0, 50.0])`
  - Measurement Noise Covariance `R`: `np.diag([25.0, 25.0])`
- Dynamic `dt`: Measure elapsed time `dt = np.clip(time.monotonic() - self.last_time, 0.005, 0.100)` on every frame and update state transition matrix `F`:
  - `F = [[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]]`
- Coasting & First-Detection Logic:
  - When `"cell phone"` is detected:
    - If `self.is_tracking == False` (first detection after reset/lost): initialize state directly to `X = [raw_ex, raw_ey, 0.0, 0.0]^T`, `P = P0`, `self.is_tracking = True`, `coast_counter = 0`, `status = 0x01` (prevents initial velocity spike from `(0,0)`).
    - Else: call `predict(dt)` then `correct([raw_ex, raw_ey])`, set `coast_counter = 0`, `status = 0x01`.
  - When `"cell phone"` is missed:
    - Increment `coast_counter`.
    - If `self.is_tracking == True` and `coast_counter <= 5`: call `predict(dt)` only and use predicted state with `status = 0x02`.
    - Else (`coast_counter > 5` or not tracking): reset `X = [0, 0, 0, 0]^T`, `P = P0`, `self.is_tracking = False`, and output `status = 0x00, pred_ex = 0, pred_ey = 0`.
- Eye-in-Hand Conditional Lead Compensation:
  - `lead_x = 0.045 if (est_ex * est_vx > 0) else 0.015`
  - `lead_y = 0.045 if (est_ey * est_vy > 0) else 0.015`
  - `pred_ex = int(round(est_ex + est_vx * lead_x))`
  - `pred_ey = int(round(est_ey + est_vy * lead_y))`
- Deadzone & Clamping:
  - If `abs(pred_ex) < 6`: `pred_ex = 0`
  - If `abs(pred_ey) < 6`: `pred_ey = 0`
  - Clamp `pred_ex` to `[-960, 960]`, `pred_ey` to `[-540, 540]`.

---

## Module B: STM32 500Hz Visual Servoing Firmware (`stm32_fw/`)

### 0. PlatformIO Configuration (`platformio.ini`)
- Target Environment: `[env:nucleo_f401re]`
- `platform = ststm32`
- `board = nucleo_f401re`
- `framework = stm32cube`
- `build_flags = -mfpu=fpv4-sp-d16 -mfloat-abi=hard -O2 -Wall`

### 1. Ring-Buffer FSM Parser (`protocol.c`)
- `USART1` (`DMA2_Stream2`, `DMA_CHANNEL_4`, Circular mode) writes into `uint8_t dma_rx_buf[64]`.
- Poll `dma_head = (64 - __HAL_DMA_GET_COUNTER(huart1.hdmarx)) & 0x3F` inside the main `while(1)` loop and process each byte up to `dma_head` through `Protocol_ParseByte(uint8_t b)`:
  - `STATE_WAIT_FF`:
    - If `b == 0xFF` -> transition to `STATE_WAIT_FE`.
  - `STATE_WAIT_FE`:
    - If `b == 0xFE` -> transition to `STATE_PAYLOAD` (`payload_idx = 0`).
    - Else if `b == 0xFF` -> **STAY in `STATE_WAIT_FE`**.
    - Else -> transition back to `STATE_WAIT_FF`.
  - `STATE_PAYLOAD`:
    - Collect 6 bytes into `payload[0..5]` (`status`, `ex_L`, `ex_H`, `ey_L`, `ey_H`, `checksum`).
    - Verify `payload[5] == ((payload[0] + payload[1] + payload[2] + payload[3] + payload[4]) & 0xFF)`.
    - If valid:
      - Reconstruct `int16_t ex = (int16_t)((uint16_t)payload[1] | ((uint16_t)payload[2] << 8))` and `int16_t ey = (int16_t)((uint16_t)payload[3] | ((uint16_t)payload[4] << 8))`.
      - Enter critical section (`__disable_irq()`), update `g_vision_pkt`, set `g_new_pkt_flag = 1`, reset `g_comm_watchdog = 0`, exit critical section (`__enable_irq()`).
    - Always transition back to `STATE_WAIT_FF`.

### 2. 500Hz Deterministic PD Servo Loop (`main.c` -> `TIM2_IRQHandler`)
- Executed strictly every `2.0 ms` (`500 Hz`):
  1. Copy shared volatile variables locally (`status`, `pkt_err_x`, `pkt_err_y`, `new_flag`) and clear `g_new_pkt_flag = 0` atomically.
  2. Increment `g_comm_watchdog`.
  3. Safety Hold & First-Frame Reset:
     - If `g_comm_watchdog > 250` (`500 ms` timeout) OR `status == 0x00`:
       - Set `d_ex_lpf = 0.0f`, `d_ey_lpf = 0.0f`, `is_first_track_frame = 1`.
       - Hold current `f_pan_step` and `f_tilt_step`.
  4. Multi-Rate Incremental PD Calculation (when `status == 0x01` or `0x02` and `g_comm_watchdog <= 250`):
     - Constants: `KP_PAN = 0.0025f`, `KD_PAN = 0.015f`, `KP_TILT = 0.0025f`, `KD_TILT = 0.015f`
     - Direction Macros: `DIR_PAN = 1.0f`, `DIR_TILT = -1.0f`
     - When `new_flag == 1`:
       - If `is_first_track_frame == 1`:
         - `raw_dex = 0.0f`, `raw_dey = 0.0f` (Prevents derivative kick on re-acquisition)
         - `is_first_track_frame = 0`
       - Else:
         - `raw_dex = (float)(pkt_err_x - prev_err_x)`
         - `raw_dey = (float)(pkt_err_y - prev_err_y)`
       - `prev_err_x = pkt_err_x`, `prev_err_y = pkt_err_y`
       - `d_ex_lpf = 0.6f * d_ex_lpf + 0.4f * raw_dex`
       - `d_ey_lpf = 0.6f * d_ey_lpf + 0.4f * raw_dey`
     - When `new_flag == 0` (sub-ticks between camera frames):
       - `d_ex_lpf *= 0.85f`
       - `d_ey_lpf *= 0.85f`
     - Apply step increments:
       - `f_pan_step  += DIR_PAN  * (KP_PAN  * (float)pkt_err_x + KD_PAN  * d_ex_lpf)`
       - `f_tilt_step += DIR_TILT * (KP_TILT * (float)pkt_err_y + KD_TILT * d_ey_lpf)`
  5. Mechanical Limit Clamping (12-bit step `0..4095`, Center = `2048`):
     - Clamp `f_pan_step` to `[512.0f, 3584.0f]`, `f_tilt_step` to `[1536.0f, 2560.0f]`.
  6. Transmit **22-Byte** `SYNC WRITE (0x83)` packet to `USART6` via DMA (`STS_SyncWrite_PosSpeed((uint16_t)(f_pan_step + 0.5f), 3400, (uint16_t)(f_tilt_step + 0.5f), 3400)`).

### 3. Feetech STS3215 Protocol & ID Commissioning (`feetech_sts.c`)
- **22-Byte `SYNC WRITE` Packet Format**:
  - Must use a **`static uint8_t tx_buf[22]`** buffer so memory stays valid during DMA transfer.
  - Check `if (huart6.gState != HAL_UART_STATE_READY) return;` before modifying `tx_buf`.
  - Layout: `[0xFF, 0xFF, 0xFE, 0x12, 0x83, 0x2A, 0x06, 0x01, Pan_L, Pan_H, 0x00, 0x00, Spd1_L, Spd1_H, 0x02, Tilt_L, Tilt_H, 0x00, 0x00, Spd2_L, Spd2_H, Checksum]`
  - `Checksum = ~(tx_buf[2] + tx_buf[3] + ... + tx_buf[20]) & 0xFF`.
- **Half-Duplex Direction Pin (`PA4`) Timing**:
  - Set `PA4` HIGH before calling `HAL_UART_Transmit_DMA(&huart6, tx_buf, 22)`.
  - Inside `HAL_UART_TxCpltCallback`, if `huart->Instance == USART6`, set `PA4` LOW.
- **One-Time ID Assignment Function (`STS_Assign_ID(uint8_t old_id, uint8_t new_id)`)**:
  - Triggered during boot in `main()` if `PC13` (User Button B1) stays LOW for 50ms debounce.
  - Uses blocking `HAL_UART_Transmit(&huart6, ...)` with manual `PA4` toggling and `HAL_Delay(15)` between EEPROM unlock (`0x37=0`), ID write (`0x05=new_id`), and EEPROM lock (`0x37=1`), then blinks `PA5` (LD2) at 10Hz for 2 seconds.