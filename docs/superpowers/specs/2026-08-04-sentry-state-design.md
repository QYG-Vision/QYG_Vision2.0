# Sentry State Packed Field Design

## Goal

Replace the adjacent `chassis_state` and visual `mode` bytes in the GD receive
frame with one two-byte `sentry_state` field, without changing the 51-byte
frame length.

## Wire Format

`sentry_state` is an unsigned 16-bit little-endian integer at bytes 36-37 of
the GD frame (one-based byte numbering). Byte 36 carries bits 7-0; byte 37
carries bits 15-8.

```text
bit 15                                  bit 2 bit 1 bit 0
+--------------------------------------------+-------------+
|          sentry status: 14 bits             | visual mode |
+--------------------------------------------+-------------+
```

`visual mode` meanings:

| Bits | Value | Gimbal mode |
| --- | --- | --- |
| 1:0 | `0b00` | `IDLE` |
| 1:0 | `0b01` | `AUTO_AIM` |
| 1:0 | `0b10` | `SMALL_BUFF` |
| 1:0 | `0b11` | `BIG_BUFF` |

The 14-bit status value is `(sentry_state >> 2) & 0x3FFF`.

## Compatibility

The GD frame remains 51 bytes and the following field offsets remain unchanged:
`vyaw` byte 38, `vpitch` byte 42, `vroll` byte 46, and CRC16 byte 50.

This is wire-layout compatible but semantically incompatible with the former
`uint8_t chassis_state` plus `uint8_t mode` representation. The STM32 sender
and vision receiver must switch together. The old `0x11`, `0x12`, and `0x13`
mode values are no longer valid.

## Code Boundaries

`ReceiveFrame` and `GimbalState` keep the raw packed `uint16_t sentry_state`.
Named helpers own all encoding and decoding. Consumers use the typed
`GimbalMode` result rather than duplicating masks or numeric mode constants.
C++ bit-fields are intentionally not used because their serialized layout is
implementation-defined.

## Validation

The protocol loop test must cover the four mode encodings, 14-bit status
extraction, byte-level little-endian layout, parsing, CRC rejection, and the
existing ROS message path.
