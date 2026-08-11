# Bare-Metal USB CDC HS — STM32F446 + USB3300

## Overview

This project implements a lightweight **USB CDC ACM device** for the STM32F446 using the integrated **USB OTG HS controller with an external USB3300 ULPI High-Speed PHY**.

The driver is intentionally written **bare-metal**, without HAL, CubeMX, or a USB middleware stack.

The implementation provides:

- USB 2.0 High-Speed operation

- CDC ACM / Virtual COM Port

- EP0 control endpoint using DMA

- EP1 Bulk IN using DMA

- EP1 Bulk OUT using DMA

- EP2 Interrupt IN CDC notification endpoint

- Circular RX and TX buffers

- DMA-safe staging buffers

- RX back-pressure when the application buffer is full

- TX ownership protection until USB DMA completion

- Configurable USB serial number

- Automatic STM32 UID-based serial number by default

- Explicit ULPI / USB OTG HS initialization

- Direct register-level interrupt handling

Only two driver files are intended to be integrated into the application:

```
usb\_cdc\_hs.c  
usb\_cdc\_hs.h
```

The current public data API is deliberately simple:

```
uint16\_t USB\_CDC\_Read(uint8\_t \*dest, uint16\_t maxLen);  
uint32\_t USB\_CDC\_Write(uint8\_t \*src, uint16\_t len);
```

The circular-buffer implementation remains private to the driver.


# 1. USB Endpoint Architecture

The USB device uses three endpoint numbers:

```
                     USB DEVICE  
                         │  
              ┌──────────┴──────────┐  
              │                     │  
             EP0                   CDC  
          Control pipe              │  
              │          ┌──────────┼──────────┐  
              │          │          │          │  
              │         EP1        EP1        EP2  
              │          IN        OUT         IN  
              │        Bulk       Bulk      Interrupt  
              │          │          │          │  
              ▼          ▼          ▼       CDC notification  
           Control      TX         RX
```

## EP0 — Control

EP0 is the mandatory USB default control pipe.

It handles:

- USB enumeration

- Device descriptor

- Configuration descriptors

- String descriptors

- Device address

- Device configuration

- CDC class requests

- `GET\_LINE\_CODING`

- `SET\_LINE\_CODING`

- `SET\_CONTROL\_LINE\_STATE`

- Endpoint stall management

EP0 IN and OUT use DMA.

The EP0 maximum packet size is 64 bytes.

## EP1 IN — CDC Bulk

EP1 IN is the application → host data path.

The application uses:

```
USB\_CDC\_Write(src, len);
```

The internal flow is:

```
Application  
    │  
    ▼  
USB\_CDC\_Write()  
    │  
    ▼  
TX circular buffer  
    │  
    ▼  
private TX peek  
    │  
    ▼  
512-byte DMA staging buffer  
    │  
    ▼  
USB OTG HS DMA  
    │  
    ▼  
EP1 IN  
    │  
    ▼  
Host
```

The USB DMA does not operate directly on the circular buffer.

Instead, the driver copies a contiguous block into an aligned DMA staging buffer.

## EP1 OUT — CDC Bulk

EP1 OUT is the host → application data path.

```
Host  
  │  
  ▼  
EP1 OUT  
  │  
  ▼  
USB OTG HS DMA  
  │  
  ▼  
EP1 RX DMA staging buffer  
  │  
  │ transfer complete  
  ▼  
RX circular buffer  
  │  
  ▼  
USB\_CDC\_Read()  
  │  
  ▼  
Application
```

The USB DMA writes to a dedicated linear staging buffer. The received packet is then copied into the RX circular buffer.

This isolates the USB DMA transfer from circular-buffer wrap-around.

## EP2 IN — CDC Notification

EP2 IN is the CDC notification endpoint.

It is included because it is part of the CDC ACM implementation structure and provides a place for CDC Serial-State notifications.

The current application does not generate notifications, so EP2 remains NAKed.

This is intentional and leaves the descriptor/hardware structure ready for future CDC notification support.


# 2. Public Application API

The application should not know that the driver uses circular buffers internally.

The intended abstraction is:

```
                 APPLICATION  
                      │  
             ┌────────┴────────┐  
             │                 │  
             ▼                 ▼  
      USB\_CDC\_Read()    USB\_CDC\_Write()  
             │                 │  
             ▼                 ▼  
          EP1 OUT            EP1 IN
```

## Reading

Use:

```
uint16\_t len = USB\_CDC\_Read(dest, sizeof(dest));
```

The function returns the number of bytes copied.

The application does not need to know:

- the current circular-buffer read index

- the current write index

- whether the ring wraps

- how RX DMA is armed

- where the DMA staging buffer is located

`USB\_CDC\_Read()` transparently handles all of this.

If more data remains than fits in `dest`, call `USB\_CDC\_Read()` again.

Typical usage:

```
while (1)  
\{  
    uint16\_t len = USB\_CDC\_Read(dest, sizeof(dest));  
  
    if (len == 0)  
        break;  
  
    /\* Process dest\[0 .. len-1\] \*/  
\}
```

## Writing

Use:

```
USB\_CDC\_Write(src, len);
```

The internal TX circular buffer, DMA staging buffer, transfer scheduling and transfer-completion handling are hidden from the application.


# 3. RX Buffer Ownership

A central design principle of this driver is:

> **USB DMA must never be allowed to reuse RX memory while the application is still using it.**

Internally, the driver uses two private operations:

```
static circBufferAddress peek\_circBufferRx(uint16\_t len);  
static void commit\_circBufferRx(uint16\_t len);
```

These functions are implementation details and are deliberately not exposed in `usb\_cdc\_hs.h`.

The safe ownership sequence is:

```
peek  
  │  
  ▼  
copy/process  
  │  
  ▼  
commit  
  │  
  ▼  
DMA may reuse the released space
```

`USB\_CDC\_Read()` implements this sequence internally.

The application therefore cannot accidentally re-arm the RX DMA before its copy has completed.


# 4. RX Circular Buffer and Wrap-Around

The RX data is stored internally in a circular buffer.

The application sees a linear byte stream.

For example, if the internal ring contains:

```
             read  
              ↓  
+--------------------------------+  
| A A A A A A |       | B B B B |  
+--------------------------------+  
                         ↑  
                        write
```

and the application requests 10 bytes:

```
USB\_CDC\_Read(dest, 10);
```

the driver can internally perform:

```
first contiguous block  
        │  
        ├── copy 6 bytes  
        └── commit 6  
  
second contiguous block  
        │  
        ├── copy 4 bytes  
        └── commit 4
```

The application receives:

```
dest\[0..9\] = A A A A A A B B B B
```

It never needs to know that the circular buffer wrapped.

This is an important API property:

> **Circular-buffer wrap-around is a driver implementation detail, not an application responsibility.**


# 5. RX Back-Pressure

The RX circular buffer uses an N−1 scheme so that the empty condition is unambiguous.

Before arming EP1 OUT DMA, the driver verifies that enough free space exists for another USB packet.

If the buffer cannot accommodate another complete packet:

```
EP1 OUT DMA  
     │  
     ▼  
not armed  
     │  
     ▼  
USB endpoint remains NAKed  
     │  
     ▼  
Host retries
```

Once the application consumes data through:

```
USB\_CDC\_Read()
```

the driver releases the consumed space and can re-arm EP1 OUT DMA.

This provides natural USB back-pressure instead of overwriting unread application data.


# 6. TX Buffer Ownership

The TX path uses the same ownership principle.

The internal sequence is:

```
peek → DMA → XFRC → commit
```

The driver determines which TX data can be transmitted but does not release that data before the USB transfer has completed.

The data remains owned by the USB transfer until the EP1 IN transfer-complete (`XFRC`) event.

Only after the transfer completes is the TX circular-buffer read pointer advanced.

This prevents the application from reusing memory that USB DMA is still reading.


# 7. DMA Staging Buffers

Dedicated linear staging buffers isolate USB DMA from the circular buffers.

## RX

```
USB DMA  
   │  
   ▼  
linear RX staging buffer  
   │  
   ▼  
memcpy  
   │  
   ▼  
RX circular buffer
```

## TX

```
TX circular buffer  
   │  
   ▼  
memcpy  
   │  
   ▼  
linear TX staging buffer  
   │  
   ▼  
USB DMA
```

This avoids requiring USB DMA to understand circular-buffer wrap-around.

It also provides a clean ownership boundary between the USB hardware and application data.

All buffers directly accessed by USB DMA are explicitly 4-byte aligned.


# 8. EP0 DMA

EP0 is DMA based but retains normal USB control-transfer semantics.

For example, a CDC `SET\_LINE\_CODING` request follows the control-transfer sequence:

```
SETUP packet  
     │  
     ▼  
EP0 OUT DMA  
     │  
     ▼  
7-byte line-coding data  
     │  
     ▼  
transfer complete  
     │  
     ▼  
update lineCoding  
     │  
     ▼  
STATUS IN ZLP
```

EP0 therefore is not treated like a normal bulk endpoint.

The driver handles the SETUP, DATA and STATUS stages explicitly.


# 9. CDC Descriptor Structure

The configuration contains two CDC interfaces.

## Interface 0 — CDC Communication

Interface 0 contains the CDC functional descriptors:

```
Interface 0  
    │  
    ├── Header Functional Descriptor  
    ├── Call Management Descriptor  
    ├── ACM Descriptor  
    ├── Union Descriptor  
    └── EP2 IN Interrupt Notification
```

EP2 is currently unused but remains part of the CDC implementation.

## Interface 1 — CDC Data

Interface 1 contains:

```
EP1 OUT — Bulk  
EP1 IN  — Bulk
```

For High Speed:

```
Maximum Bulk Packet Size = 512 bytes
```

For Full Speed:

```
Maximum Bulk Packet Size = 64 bytes
```

The driver therefore provides both the normal configuration descriptor and the other-speed configuration descriptor.


# 10. USB Serial Number

The driver supports two serial-number sources.

## Default — STM32 UID

If the application does nothing, the driver generates the USB serial number from the STM32 96-bit unique device ID.

The UID is converted into 24 hexadecimal characters and stored in the USB UTF-16LE serial-number descriptor.

## User-defined serial

The application can provide its own serial number:

```
USB\_CDC\_SetSerialNumber("MY\_PRODUCT\_00123");
```

The driver converts the ASCII string to the USB UTF-16LE descriptor format.

The current maximum is 24 characters.

If `USB\_CDC\_SetSerialNumber()` is never called, the STM32 UID remains the default.

This makes the driver suitable for both development boards and manufactured products with assigned serial numbers.


# 11. USB3300 ULPI Initialization

The STM32F446 uses the external USB3300 PHY for High-Speed USB.

The USB OTG HS and ULPI clocks are enabled during initialization.

The ULPI pins are configured for the USB OTG HS alternate function at Very High Speed.

The ULPI signals include:

```
ULPI\_CLK  
ULPI\_DIR  
ULPI\_NXT  
ULPI\_STP  
ULPI\_D0 ... ULPI\_D7
```

The USB3300 reset sequence is also handled during USB initialization.


# 12. USB OTG HS DMA

The OTG HS controller is configured for DMA operation.

DMA-accessed buffers are explicitly aligned:

```
\_\_attribute\_\_((aligned(4)))
```

This applies to the relevant:

- EP0 buffers

- EP1 RX staging buffer

- EP1 TX staging buffer

- USB descriptors

- serial-number descriptor

The driver therefore keeps the memory requirements of the USB DMA explicit rather than relying on incidental linker alignment.


# 13. FIFO Architecture

The STM32F446 OTG HS FIFO RAM provides:

```
4096 bytes  
1024 × 32-bit words
```

The FIFO allocation must remain within the 1024-word hardware limit.

The FIFO RAM is still used by the OTG HS controller even though the CPU no longer manually services the FIFOs for CDC data transfers.

The driver configures the RX FIFO and the required endpoint TX FIFOs during initialization.


# 14. Interrupt Architecture

USB operation is interrupt driven.

The USB core handles events such as:

- USB reset

- enumeration done

- IN endpoint interrupts

- OUT endpoint interrupts

- transfer completion

- wakeup

Transfer-complete events are particularly important because they define DMA ownership boundaries.

For TX:

```
USB DMA active  
      │  
      ▼  
EP1 IN XFRC  
      │  
      ▼  
release/commit TX data
```

For RX:

```
USB DMA active  
      │  
      ▼  
EP1 OUT XFRC  
      │  
      ▼  
copy packet to RX ring  
      │  
      ▼  
notify application
```


# 15. ISR vs Application Responsibilities

The USB interrupt handlers should perform only the work required to maintain USB and buffer state.

The application should process received data outside interrupt context.

The supplied callback:

```
uint32\_t USB\_CDC\_UserRxCallBack\_EP1(uint16\_t length);
```

is intended to notify the application that new data is available.

A typical callback can simply raise a flag:

```
volatile uint8\_t flag = 0;  
  
uint32\_t USB\_CDC\_UserRxCallBack\_EP1(uint16\_t length)  
\{  
    flag = 1;  
    return EP\_OK;  
\}
```

The main loop then calls:

```
USB\_CDC\_Read()
```

to retrieve the data.

This keeps application processing out of the USB ISR.


# 16. Example Application Flow

A simple echo application can use:

```
while (1)  
\{  
    if (flag)  
    \{  
        flag = 0;  
  
        while (1)  
        \{  
            uint16\_t len = USB\_CDC\_Read(dest, sizeof(dest));  
  
            if (len == 0)  
                break;  
  
            if (USB\_CDC\_Write(dest, len) != EP\_OK)  
            \{  
                (void)USB\_CDC\_Write(dest, len);  
            \}  
        \}  
    \}  
\}
```

The important point is that the application does not access:

```
circBufferRx  
readPtrRxCbuf  
writePtrRxCbuf  
peek\_circBufferRx()  
commit\_circBufferRx()
```

Those are internal driver mechanisms.


# 17. Initialization Sequence

The example initialization sequence is:

```
SysClockConfig()  
        │  
        ▼  
GPIO\_Config()  
        │  
        ▼  
InterruptGPIO\_Config()  
        │  
        ▼  
SystemClock\_Config()  
        │  
        ▼  
USB\_OTG\_HS\_GPIO\_Init()  
        │  
        ▼  
USB\_OTG\_HS\_Core\_Init()  
        │  
        ▼  
USB\_OTG\_HS\_FIFO\_and\_Interrupts\_Init()  
        │  
        ▼  
USB\_OTG\_HS\_Connect()  
        │  
        ▼  
USB enumeration
```

The FIFO and interrupt configuration is performed before connecting the device to the host.

The USB reset handler then initializes endpoint/DMA state as part of enumeration.


# 18. Reset and Recovery

`USB\_CDC\_ForceResetState()` resets the software state associated with USB CDC transfers.

Reset/recovery handling resets the relevant:

- RX pointers

- TX pointers

- pending transfer state

- endpoint transfer counters

- endpoint states

The relevant USB FIFOs and endpoint interrupts are also handled as part of recovery.

EP1 OUT is re-primed only when the RX buffer has enough free space for the next transfer.

This keeps reset/recovery consistent with the normal RX ownership model.


# 19. Important Design Invariants

The driver relies on the following invariants.

### RX

```
USB DMA writes only to the dedicated RX DMA staging buffer.  
  
The RX circular buffer is not released until the application has  
finished copying/processing the data.  
  
USB\_CDC\_Read() performs the copy before releasing the consumed space.
```

### TX

```
USB DMA reads only from the dedicated TX DMA staging buffer.  
  
TX circular-buffer data remains reserved until the USB transfer completes.  
  
The TX read pointer is advanced only after XFRC.
```

### EP1 OUT

```
EP1 OUT DMA is not armed unless sufficient RX buffer space exists  
for the transfer.
```

### EP0

```
Control transfers are handled according to the USB SETUP/DATA/STATUS  
sequence.
```

### DMA

```
DMA-accessed buffers are explicitly 4-byte aligned.
```

These invariants are the foundation of the driver architecture.


# 20. Data Flow Example — RX

For a 512-byte packet received from the host:

```
Host  
 │  
 │ 512-byte USB Bulk OUT  
 ▼  
EP1 OUT  
 │  
 ▼  
USB DMA  
 │  
 ▼  
RX DMA staging buffer  
 │  
 │ XFRC  
 ▼  
RX circular buffer  
 │  
 ▼  
USB\_CDC\_UserRxCallBack\_EP1()  
 │  
 ▼  
Application  
 │  
 ▼  
USB\_CDC\_Read(dest, ...)  
 │  
 ├── private peek  
 ├── memcpy  
 └── private commit  
 │  
 ▼  
EP1 OUT DMA can reuse released space
```

If the ring wraps, `USB\_CDC\_Read()` repeats the private peek/copy/commit sequence internally. The application still receives one linear block.


# 21. Data Flow Example — TX

```
Application  
 │  
 ▼  
USB\_CDC\_Write()  
 │  
 ▼  
TX circular buffer  
 │  
 ▼  
private TX peek  
 │  
 ▼  
TX DMA staging buffer  
 │  
 ▼  
USB DMA  
 │  
 ▼  
EP1 IN  
 │  
 ▼  
Host  
 │  
 ▼  
XFRC  
 │  
 ▼  
private TX commit
```

The application does not need to wait for or manage USB DMA completion directly.


# 22. Why the Driver Uses Staging Buffers

The staging buffers solve two problems at once.

### Circular-buffer boundaries

A USB High-Speed packet can contain up to 512 bytes, while the ring-buffer read/write pointers can be close to the end of the array.

The staging buffer ensures that every USB DMA transfer is linear and contiguous.

### Ownership

The staging buffers also make USB DMA ownership explicit.

For RX:

```
DMA owns staging buffer  
       │  
       ▼  
transfer complete  
       │  
       ▼  
driver copies to RX ring
```

For TX:

```
driver copies from TX ring  
       │  
       ▼  
DMA owns staging buffer  
       │  
       ▼  
transfer complete
```

This separation makes the concurrency model easier to reason about.


# 23. Generic Driver Philosophy

The driver is intended to be reusable rather than tied to one application.

The public API should therefore expose **USB CDC behavior**, not implementation details.

Prefer:

```
USB\_CDC\_Read()  
USB\_CDC\_Write()  
USB\_CDC\_SetSerialNumber()
```

rather than exposing:

```
peek\_circBufferRx()  
commit\_circBufferRx()  
circBufferRx  
readPtrRxCbuf  
writePtrRxCbuf
```

The latter are implementation mechanisms and should remain private.

The result is a clean boundary:

```
┌──────────────────────────────────────────┐  
│              Application                 │  
│                                          │  
│  USB\_CDC\_Read()                          │  
│  USB\_CDC\_Write()                         │  
│  USB\_CDC\_SetSerialNumber()               │  
│  USB\_CDC\_UserRxCallBack\_EP1()            │  
└──────────────────┬───────────────────────┘  
                   │  
             Public API  
                   │  
┌──────────────────▼───────────────────────┐  
│             USB CDC Driver               │  
│                                          │  
│  Circular buffers                        │  
│  DMA staging buffers                     │  
│  Endpoint state                          │  
│  USB descriptors                         │  
│  DMA scheduling                          │  
│  Interrupt handling                      │  
│  ULPI / OTG HS hardware                  │  
└──────────────────┬───────────────────────┘  
                   │  
                   ▼  
              STM32F446  
                   │  
                   ▼  
               USB3300
```


# 24. Integration

The intended integration is lightweight:

```
usb\_cdc\_hs.c  
usb\_cdc\_hs.h
```

The application provides the platform-specific functions required by the project and may implement:

```
USB\_CDC\_UserRxCallBack\_EP1()
```

The application can then use:

```
USB\_CDC\_Read()  
USB\_CDC\_Write()  
USB\_CDC\_SetSerialNumber()
```

without knowing the internal DMA/circular-buffer implementation.


# 25. Current Limitations / Future Work

### EP2 Serial-State notifications

EP2 is included in the CDC configuration but currently remains unused/NAKed.

Future work could implement CDC Serial-State notifications such as:

- carrier state changes

- line state changes

- UART status notifications

### Application-level protocol

USB CDC provides a byte stream. Any framing, packetization, command protocol, CRC, etc. remains the responsibility of the application.

### Buffer sizing

The RX/TX circular-buffer sizes and DMA staging buffers are compile-time design choices and can be adapted to the application.


# 26. Summary

The core architecture can be summarized as:

```
                       USB3300  
                          │  
                          ▼  
                 STM32F446 OTG HS  
                          │  
          ┌───────────────┼────────────────┐  
          │               │                │  
         EP0             EP1              EP2  
       Control           Data          Notification  
          │          ┌────┴────┐             │  
          │          │         │             │  
          │         IN        OUT            │  
          │          │         │             │  
          ▼          ▼         ▼             ▼  
         DMA        TX        RX          Reserved  
                    │         │  
                    ▼         ▼  
              DMA staging buffers  
                    │         │  
                    ▼         ▼  
              circular buffers  
                    │         │  
                    └────┬────┘  
                         │  
                         ▼  
                    Application  
                         │  
              ┌──────────┴──────────┐  
              │                     │  
              ▼                     ▼  
       USB\_CDC\_Read()       USB\_CDC\_Write()
```

The most important ownership rules are:

```
RX:  peek → copy/process → commit → DMA may reuse  
  
TX:  peek → DMA → XFRC → commit
```

The application-facing interface hides these mechanisms and presents the USB CDC device as a simple byte stream.


## Public API at a glance

```
uint16\_t USB\_CDC\_Read(uint8\_t \*dest, uint16\_t maxLen);  
  
uint32\_t USB\_CDC\_Write(uint8\_t \*src, uint16\_t len);  
  
uint32\_t USB\_CDC\_SetSerialNumber(const char \*serial);  
  
uint32\_t USB\_CDC\_UserRxCallBack\_EP1(uint16\_t length);
```

The first three are driver services; the callback is the application hook used to notify the application that EP1 OUT has received new data.

