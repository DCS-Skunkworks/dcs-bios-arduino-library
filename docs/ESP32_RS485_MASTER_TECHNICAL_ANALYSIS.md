# ESP32-S3 RS485 Master Technical Analysis

## Executive Summary

This document details all attempts to port the DCS-BIOS RS485 Master from AVR (Arduino Mega) to ESP32-S3. Despite multiple implementation approaches, we have not achieved full functionality. This document serves as a comprehensive reference to prevent repeated failed approaches.

---

## 1. Hardware Configuration

### Target Hardware
- **Board**: Waveshare ESP32-S3-RS485-CAN
- **MCU**: ESP32-S3
- **RS485 Pins**: TX=GPIO17, RX=GPIO18, DE/RE (TX Enable)=GPIO21
- **PC Serial**: **Hardware CDC (HW CDC)** - NOT TinyUSB
- **RS485 Transceiver**: Onboard, directly connected to UART1

### Communication Parameters
- **Baud Rate**: 250,000 bps
- **Data Format**: 8N1 (8 data bits, no parity, 1 stop bit)
- **Protocol**: Half-duplex RS485

### Important Hardware Note
The ESP32-S3 has two USB serial options:
1. **TinyUSB (USB OTG)** - Software USB stack
2. **Hardware CDC (HW CDC)** - Native USB-serial built into the chip

**The user is using HW CDC**, which means `Serial` maps to the hardware USB-CDC peripheral, not a software USB stack. This may have different buffering and timing characteristics than TinyUSB or traditional UART.

---

## 2. AVR Reference Implementation Analysis

### Architecture Overview (src/internal/DcsBiosNgRS485Master.cpp.inc)

The AVR implementation uses **hardware interrupts** for all UART operations:

```
┌─────────────────────────────────────────────────────────────────┐
│                        AVR MEGA 2560                            │
├─────────────────────────────────────────────────────────────────┤
│  UART0 (PC Connection)           UART1/2/3 (RS485 Buses)        │
│  ┌─────────────────────┐         ┌─────────────────────┐        │
│  │ ISR(USART0_RX_vect) │         │ ISR(USARTn_RX_vect) │        │
│  │ - Fires on EVERY    │         │ - Receives slave    │        │
│  │   byte from PC      │         │   responses         │        │
│  │ - Immediately puts  │         │ - Feeds message     │        │
│  │   byte into export  │         │   buffer            │        │
│  │   data buffer       │         └─────────────────────┘        │
│  └─────────────────────┘         ┌─────────────────────┐        │
│  ┌─────────────────────┐         │ ISR(USARTn_UDRE_vect│        │
│  │ ISR(USART0_UDRE_vect│         │ - TX buffer empty   │        │
│  │ - TX buffer empty   │         │ - Sends next byte   │        │
│  │ - Sends slave resp  │         │   to slaves         │        │
│  │   to PC             │         └─────────────────────┘        │
│  └─────────────────────┘         ┌─────────────────────┐        │
│                                  │ ISR(USARTn_TX_vect) │        │
│                                  │ - TX complete       │        │
│                                  │ - Switches to RX    │        │
│                                  └─────────────────────┘        │
├─────────────────────────────────────────────────────────────────┤
│                      Main Loop                                  │
│  - Only handles state machine logic                             │
│  - Timeouts (1ms poll, 5ms RX)                                  │
│  - NEVER reads/writes UART directly (ISRs do that)              │
└─────────────────────────────────────────────────────────────────┘
```

### Critical AVR Characteristics

1. **True Concurrency via Hardware Interrupts**
   - `ISR(USART0_RX_vect)` fires on EVERY byte from PC, regardless of main loop state
   - ISRs preempt main code instantly (within clock cycles)
   - No byte is ever missed because hardware triggers interrupt

2. **Byte-by-Byte Transmission**
   - `ISR(USARTn_UDRE_vect)` fires when TX buffer is empty
   - Sends exactly one byte per interrupt
   - State machine advances one step per byte

3. **TX Complete Detection**
   - `ISR(USARTn_TX_vect)` fires when last bit has been physically transmitted
   - This is when the line switches from TX to RX mode
   - Critical for half-duplex timing

4. **State Machine States**
   ```
   IDLE → POLL_ADDRESS_SENT → POLL_MSGTYPE_SENT → POLL_DATALENGTH_SENT
                                                          ↓
   IDLE ← RX_WAIT_CHECKSUM ← RX_WAIT_DATA ← RX_WAIT_MSGTYPE ← RX_WAIT_DATALENGTH

   IDLE → TX_ADDRESS_SENT → TX_MSGTYPE_SENT → TX → TX_CHECKSUM_SENT → IDLE
   ```

### Key Code Sections

**PC RX ISR (Line 285-296):**
```cpp
void __attribute__((always_inline)) inline MasterPCConnection::rxISR() {
    volatile uint8_t c = *udr;  // Read byte from UART register
    #ifdef UART1_TXENABLE_PIN
    uart1.exportData.put(c);    // Immediately buffer for RS485
    #endif
    // ... repeat for UART2, UART3
}
```

**RS485 TX Complete ISR (Line 155-175):**
```cpp
void __attribute__((always_inline)) inline RS485Master::txcISR() {
    switch (state) {
        case POLL_DATALENGTH_SENT:
            rx_start_time = micros();
            state = RX_WAIT_DATALENGTH;
            clear_txen();  // Switch to RX mode AFTER last bit sent
        break;
        // ...
    }
}
```

---

## 3. Implementation Attempts

### Attempt 1: Arduino HardwareSerial

**Approach:**
```cpp
HardwareSerial RS485Serial(1);
RS485Serial.begin(250000, SERIAL_8N1, RX_PIN, TX_PIN);
```

**Result:** FAILED - Inputs and outputs did not work

**Analysis:**
- Arduino's HardwareSerial does not support RS485 half-duplex mode
- No automatic TX enable control
- No way to detect TX complete for line turnaround

---

### Attempt 2: ESP-IDF UART Driver with RS485 Mode

**Approach:**
```cpp
uart_set_mode(uartNum, UART_MODE_RS485_HALF_DUPLEX);
uart_set_pin(uartNum, TX_PIN, RX_PIN, RTS_PIN, UART_PIN_NO_CHANGE);
```

**Result:** PARTIAL - Inputs worked, outputs failed

**Observations:**
- Slave responses were received and forwarded to PC (inputs worked)
- PC export data was not reaching slaves (outputs failed)
- Suspected: Data from PC was being lost during RS485 operations

**Analysis:**
- ESP-IDF's RS485 mode handles DE/RE pin automatically
- However, we were using `uart_wait_tx_done()` which BLOCKS
- During blocking, PC serial data was not being read
- At 250kbaud, 25 bytes arrive per millisecond
- A 10ms block = 250 bytes lost

---

### Attempt 3: Dual-Core FreeRTOS Implementation

**Approach:**
- Core 0: High-priority task constantly reading PC Serial
- Core 1: RS485 state machine
- Lock-free FIFO for inter-core communication

**Result:** FAILED - Same issues

**Analysis:**
- Added complexity without solving fundamental problem
- FreeRTOS task switching still not as fast as hardware ISR
- Context switch overhead (~1-4µs) vs ISR (~0.1µs)

---

### Attempt 4: Simplified AVR Mirror (No FreeRTOS)

**Approach:**
- Single-core, single-task implementation
- Mirror AVR state machine exactly
- Use library's existing RingBuffer
- Poll PC serial at start and end of loop

**Result:** PARTIAL - Inputs worked but with lag, outputs failed

**Observations:**
- Inputs worked but slower than AVR
- Outputs still failed completely

---

### Attempt 5: Aggressive PC Serial Reading (Current)

**Approach:**
- Remove ALL blocking `uart_wait_tx_done()` calls
- Add intermediate TX_WAIT states that poll for completion
- Call `pcConnection.rxProcess()` at EVERY state transition
- Non-blocking TX completion check: `uart_wait_tx_done(uartNum, 0)`

**Result:** COMPLETE FAILURE - Inputs no longer work at all

**Observations:**
- Made things WORSE
- Even inputs that previously worked now fail
- Something fundamental is broken

---

## 4. Hypotheses

### Hypothesis A: HW CDC Buffering Behavior

**Theory:**
Hardware CDC may buffer data differently than UART or TinyUSB. When we call `Serial.available()` and `Serial.read()`, we may not be getting data in real-time.

**Evidence:**
- User specifically mentioned using HW CDC, not TinyUSB
- HW CDC has its own USB endpoint and buffering
- May have different latency characteristics

**Test:**
- Check ESP32-S3 HW CDC documentation for buffering behavior
- Try using `Serial.setRxBufferSize()` if available
- Try using lower-level USB CDC API

---

### Hypothesis B: UART Driver Event Queue

**Theory:**
ESP-IDF UART driver uses an event queue internally. Without processing events, the driver may behave unexpectedly.

**Evidence:**
- We pass `0, NULL, 0` to `uart_driver_install()` (no event queue)
- Some UART features may require event processing
- RS485 mode may have specific event requirements

**Test:**
```cpp
QueueHandle_t uart_queue;
uart_driver_install(uartNum, 256, 256, 10, &uart_queue, 0);
// Process events in loop
```

---

### Hypothesis C: RS485 Mode TX Complete Timing

**Theory:**
`uart_wait_tx_done(uartNum, 0)` may not accurately reflect when the TX enable line should be switched.

**Evidence:**
- ESP-IDF RS485 mode manages DE/RE automatically
- We're checking TX complete but driver may already have switched
- Or driver may not switch until we explicitly tell it

**Test:**
- Manual TX enable control instead of relying on driver
- Check actual GPIO state during operation
- Add oscilloscope/logic analyzer to verify timing

---

### Hypothesis D: Non-Blocking TX Check Broken

**Theory:**
`uart_wait_tx_done(uartNum, 0)` with timeout=0 may not work correctly.

**Evidence:**
- After adding non-blocking TX checks, everything broke
- ESP-IDF docs may have specific requirements

**Test:**
- Revert to blocking TX wait but with shorter timeout
- Check return value of `uart_wait_tx_done()`
- Use alternative TX complete detection

---

### Hypothesis E: State Machine Logic Error

**Theory:**
Adding intermediate TX_WAIT states broke the state machine flow.

**Evidence:**
- Original AVR doesn't have these states
- State transitions may be incorrect
- May be stuck in a wait state

**Test:**
- Add debug output to see current state
- Verify state transitions match expected flow
- Check if stuck in any state

---

### Hypothesis F: PC Serial Read Overhead

**Theory:**
Calling `pcConnection.rxProcess()` too frequently may be causing issues.

**Evidence:**
- Added ~15+ calls per loop iteration
- Each call involves `Serial.available()` and potentially `Serial.read()`
- May be introducing timing issues or overhead

**Test:**
- Reduce rxProcess calls to key points only
- Profile time spent in rxProcess

---

## 5. Proposed Solutions to Try

### Solution 1: Use ESP32 UART Interrupts (Most Promising)

**Rationale:**
The fundamental difference between AVR and ESP32 implementations is that AVR uses hardware interrupts. ESP32 UART driver supports interrupts via `uart_isr_register()`.

**Implementation:**
```cpp
static void IRAM_ATTR uart_isr_handler(void *arg) {
    // Read RX FIFO directly
    // Put bytes into buffer
    // This fires on every byte, like AVR
}

// Register ISR
uart_isr_register(uartNum, uart_isr_handler, NULL, ESP_INTR_FLAG_IRAM, NULL);
uart_enable_rx_intr(uartNum);
```

**Challenges:**
- Need to understand ESP32 UART register layout
- ISR must be in IRAM
- May conflict with ESP-IDF UART driver

---

### Solution 2: Manual TX Enable Control

**Rationale:**
Don't rely on ESP-IDF RS485 mode. Control DE/RE pin manually like AVR does.

**Implementation:**
```cpp
// Don't use UART_MODE_RS485_HALF_DUPLEX
// Manually control TX enable pin
digitalWrite(TXENABLE_PIN, HIGH);  // TX mode
uart_write_bytes(...);
uart_wait_tx_done(...);
digitalWrite(TXENABLE_PIN, LOW);   // RX mode
```

**Advantages:**
- More control over timing
- Matches AVR behavior more closely

---

### Solution 3: Different Serial API for HW CDC

**Rationale:**
HW CDC may have specific requirements or better APIs.

**Investigation:**
```cpp
// Check what Serial actually is on ESP32-S3 with HW CDC
// HWCDC class in esp32-hal-cdc.h

// Possible alternatives:
USBSerial.begin(250000);  // If available
Serial0.begin(250000);    // Physical UART0 if broken out
```

**Test:**
- Verify `Serial` class type at runtime
- Check if HW CDC has different read methods
- Try `Serial.readBytes()` vs `Serial.read()`

---

### Solution 4: Revert and Add Debug Logging

**Rationale:**
We need visibility into what's actually happening.

**Implementation:**
```cpp
// Revert to Attempt 4 (simplified AVR mirror) that partially worked
// Add state logging
Serial.printf("S:%d E:%d M:%d\n", state, exportData.getLength(), messageBuffer.getLength());
```

**Caution:**
- Debug output on same Serial may interfere
- May need separate debug UART
- Or use GPIO toggling for oscilloscope

---

### Solution 5: Check USB CDC on Second Core

**Rationale:**
ESP32-S3 USB handling may run on a specific core. Our code may be interfering.

**Test:**
```cpp
// Pin task to Core 1, leave Core 0 for USB
xTaskCreatePinnedToCore(rs485Task, "RS485", 4096, NULL, 5, NULL, 1);
```

---

### Solution 6: Use ESP-IDF UART Events Properly

**Rationale:**
The UART driver may require event queue processing for proper operation.

**Implementation:**
```cpp
QueueHandle_t uart_queue;
uart_driver_install(uartNum, 256, 256, 20, &uart_queue, 0);

void loop() {
    uart_event_t event;
    if (xQueueReceive(uart_queue, &event, 0)) {
        switch (event.type) {
            case UART_DATA:
                // Process received data
                break;
            case UART_TX_DONE:
                // TX complete, switch to RX
                break;
        }
    }
}
```

---

## 6. Data Flow Diagrams

### Working AVR Data Flow

```
PC ──250kbaud──► UART0 RX ──ISR──► exportData buffer ──► RS485 TX ──► Slaves
                                   (never misses byte)

Slaves ──► RS485 RX ──ISR──► messageBuffer ──ISR──► UART0 TX ──► PC
           (ISR fires           (ISR fires
            per byte)            when ready)
```

### Current ESP32 Data Flow (Broken)

```
PC ──250kbaud──► USB CDC ──???──► Serial buffer ──poll──► exportData ──► RS485 TX
                                  (may overflow)  (may miss)

Slaves ──► UART1 RX ──poll──► messageBuffer ──poll──► Serial.write() ──► PC
           (poll only when
            main loop runs)
```

### Key Difference
- AVR: ISRs fire **regardless** of main loop state
- ESP32: Everything depends on main loop **polling**

---

## 7. Timing Analysis

### Byte Timing at 250kbaud
- 1 byte = 10 bits (8 data + 1 start + 1 stop)
- Time per byte = 10 / 250000 = 40µs

### Worst Case PC Data Burst
- DCS-BIOS exports can be 1000+ bytes
- At 40µs/byte = 40ms of continuous data
- During this time, we MUST NOT block

### ESP32 Main Loop Timing
- If loop takes 1ms (a reasonable estimate)
- We miss 1000µs / 40µs = 25 bytes per iteration
- With 128-byte buffer, overflow in ~5ms

### AVR ISR Timing
- ISR entry: ~4 clock cycles at 16MHz = 0.25µs
- ISR execution: ~20 cycles = 1.25µs
- Total: ~1.5µs per byte
- At 40µs/byte, plenty of time between bytes

---

## 8. Register-Level Comparison

### AVR UART Registers
```
UDRn  - Data register (read RX, write TX)
UCSRnA - Status (RXC=data ready, TXC=TX complete, UDRE=TX buffer empty)
UCSRnB - Control (RXCIE, TXCIE, UDRIE = interrupt enables)
UCSRnC - Frame format
```

### ESP32 UART Registers (simplified)
```
UART_FIFO_REG      - Read/write FIFO
UART_STATUS_REG    - TX/RX FIFO counts
UART_INT_RAW_REG   - Interrupt status
UART_INT_ENA_REG   - Interrupt enables
UART_RS485_CONF_REG - RS485 configuration
```

---

## 9. What We Know Works

1. **ESP-IDF UART driver** - Correctly sends/receives on RS485 bus
2. **RS485 half-duplex mode** - TX enable pin is controlled automatically
3. **Slave polling** - When we poll slaves, they respond
4. **Slave response forwarding** - Responses can be sent to PC (when inputs work)

## 10. What We Know Fails

1. **PC data reception during RS485 TX** - Bytes are missed
2. **Export data reaching slaves** - Even when buffered, doesn't transmit correctly
3. **Non-blocking TX wait** - Using timeout=0 broke everything
4. **Aggressive polling** - More polling made things worse

---

## 11. Recommended Next Steps

### Priority 1: Revert and Stabilize
Revert to the version where inputs worked (Attempt 4) so we have a baseline.

### Priority 2: Add Visibility
Add debug output (on separate UART if possible) to see:
- Current state
- Buffer fill levels
- Byte counts

### Priority 3: Investigate HW CDC
Research ESP32-S3 Hardware CDC specifically:
- Buffering behavior
- Interrupt support
- Best practices

### Priority 4: Implement True Interrupts
If polling cannot work, implement ESP32 UART interrupts to match AVR behavior.

### Priority 5: Logic Analyzer
If available, capture actual signals:
- USB CDC data
- RS485 bus
- TX enable pin
- Timing relationships

---

## 12. Code Repository State

### Current Branch
`claude/esp32-rs485-master-support-0aNci`

### Relevant Commits
1. Initial ESP-IDF implementation
2. Simplified AVR mirror
3. RingBuffer template fix
4. Non-blocking aggressive polling (BROKEN)

### Files Modified
- `src/DcsBios.h` - Conditional compilation for ESP32
- `src/internal/ESP32RS485/DcsBiosESP32RS485Master.h` - Header
- `src/internal/ESP32RS485/DcsBiosESP32RS485Master.cpp.inc` - Implementation
- `examples/ESP32S3_RS485Master/ESP32S3_RS485Master.ino` - Example

---

## 13. Questions for User/Tester

1. When inputs worked, how fast was the response? Instant or noticeable delay?
2. Does the ESP32 show any error messages in serial monitor?
3. Is there a logic analyzer available to capture the RS485 bus?
4. Can you test with TinyUSB instead of HW CDC to compare?
5. What happens if you slow the baud rate to 115200?

---

## 14. References

- [ESP-IDF UART Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/uart.html)
- [ESP32-S3 Technical Reference Manual](https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf)
- [ESP32 Hardware CDC](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/usb_cdc.html)
- [DCS-BIOS Developer Guide](https://github.com/DCS-Skunkworks/dcs-bios/blob/master/Scripts/DCS-BIOS/doc/developerguide.adoc)

---

*Document created: 2026-02-02*
*Last updated: 2026-02-02*
*Status: Implementation attempts ongoing*
