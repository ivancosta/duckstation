// SPDX-FileCopyrightText: 2019-2026 Connor McLaughlin <stenzek@gmail.com> and contributors
// SPDX-License-Identifier: CC-BY-NC-ND-4.0
//
// PSXKeyboard
// -----------
// Emulates a BlueRetro adapter in keyboard mode. On real hardware, BlueRetro
// bridges a Bluetooth keyboard onto a controller port's serial (SIO) bus by
// emulating the real Lightspan PSX Keyboard peripheral (the only actual
// retail-ish device that ever used a PSX-side keyboard protocol). It
// identifies as device ID 0x5A96 (low byte 0x96 first, same wire convention
// as a standard digital pad's 0x5A41), then sends PS/2 Scan Code Set 2 make/
// break events. This class reproduces that exact wire format on the
// DuckStation side so a Linux 2.4/PS1 port's drivers/char/psxkbd.c can talk
// to it identically whether it's decoding a real BlueRetro adapter or this
// virtual one.
//
// Frame format (verified against a real BlueRetro<->Lightspan bus trace,
// not guessed — see psx_keyboard.cpp for the source):
//   15 bytes total, always: FF, 0x96, 0x5A, <length>, <up to 11 bytes of
//   Set 2 scancode>, zero-padded out to 15 bytes total. <length> is 0 when no
//   event is pending this poll.
//
// Keyboard implementation by Ivan F. C. Costa

#pragma once

#include "controller.h"

#include <array>
#include <memory>

class PSXKeyboard final : public Controller
{
public:
  // Each enumerator's low 7 bits are the PS/2 Scan Code Set 2 base scancode
  // for that key. Bit 0x80 is used here (not on the wire) to mark keys that
  // need an 0xE0 "extended" prefix — see IsExtended()/BaseScanCode() in the
  // .cpp. Add more keys by adding more lines here plus a matching row in
  // the binding table in the .cpp.
  enum class Key : u8
  {
    A = 0x1C,
    B = 0x32,
    C = 0x21,
    D = 0x23,
    E = 0x24,
    F = 0x2B,
    G = 0x34,
    H = 0x33,
    I = 0x43,
    J = 0x3B,
    K = 0x42,
    L = 0x4B,
    M = 0x3A,
    N = 0x31,
    O = 0x44,
    P = 0x4D,
    Q = 0x15,
    R = 0x2D,
    S = 0x1B,
    T = 0x2C,
    U = 0x3C,
    V = 0x2A,
    W = 0x1D,
    X = 0x22,
    Y = 0x35,
    Z = 0x1A,

    Num1 = 0x16,
    Num2 = 0x1E,
    Num3 = 0x26,
    Num4 = 0x25,
    Num5 = 0x2E,
    Num6 = 0x36,
    Num7 = 0x3D,
    Num8 = 0x3E,
    Num9 = 0x46,
    Num0 = 0x45,

    Grave = 0x0E,
    Minus = 0x4E,
    Equals = 0x55,
    Backslash = 0x5D,
    Backspace = 0x66,
    Tab = 0x0D,
    LeftBracket = 0x54,
    RightBracket = 0x5B,
    Enter = 0x5A,
    CapsLock = 0x58,
    Semicolon = 0x4C,
    Apostrophe = 0x52,
    LeftShift = 0x12,
    Comma = 0x41,
    Period = 0x49,
    Slash = 0x4A,
    RightShift = 0x59,
    LeftCtrl = 0x14,
    LeftAlt = 0x11,
    Space = 0x29,
    Escape = 0x76,

    F1 = 0x05,
    F2 = 0x06,
    F3 = 0x04,
    F4 = 0x0C,
    F5 = 0x03,
    F6 = 0x0B,
    F7 = 0x83,
    F8 = 0x0A,
    F9 = 0x01,
    F10 = 0x09,
    F11 = 0x78,
    F12 = 0x07,
    ScrollLock = 0x7E,

    // Extended keys — real Set 2 wire form is 0xE0 followed by the base
    // code below; bit 0x80 here is just our in-memory "needs E0" marker
    // and is stripped off before anything goes on the wire.
    RightCtrl = 0x80 | 0x14,
    RightAlt = 0x80 | 0x11,
    LeftGui = 0x80 | 0x1F,
    RightGui = 0x80 | 0x27,
    Insert = 0x80 | 0x70,
    Delete = 0x80 | 0x71,
    Home = 0x80 | 0x6C,
    End = 0x80 | 0x69,
    PageUp = 0x80 | 0x7D,
    PageDown = 0x80 | 0x7A,
    Up = 0x80 | 0x75,
    Down = 0x80 | 0x72,
    Left = 0x80 | 0x6B,
    Right = 0x80 | 0x74,
  };

  static const Controller::ControllerInfo INFO;

  explicit PSXKeyboard(u32 index);
  ~PSXKeyboard() override;

  static std::unique_ptr<PSXKeyboard> Create(u32 index);

  ControllerType GetType() const override;

  void Reset() override;
  bool DoState(StateWrapper& sw, bool apply_input_state) override;

  void ResetTransferState() override;
  bool Transfer(const u8 data_in, u8* data_out) override;

  float GetBindState(u32 index) const override;
  void SetBindState(u32 index, float value) override;
  u32 GetButtonStateBits() const override;

  void LoadSettings(const SettingsInterface& si, const char* section, bool initial) override;

private:
  // Idle: waiting for the 0x01 select byte.
  // Selected: waiting for the 0x42 read command; this is also where the
  //   next event (if any) is popped from the queue and the whole 15-byte
  //   response is built in one shot into m_response.
  // Responding: shifting the remaining 14 bytes of m_response out, one per
  //   Transfer() call, tracked by m_response_pos.
  enum class TransferState : u8
  {
    Idle,
    Selected,
    Responding,
  };

  struct KeyEvent
  {
    bool pressed = false;
    u8 keycode = 0; // a PSXKeyboard::Key value (may have the 0x80 extended marker set)
  };

  // Fixed-capacity FIFO. This is deliberately NOT a "latest state" register:
  // every discrete key transition is queued and popped exactly once, in
  // order, even if it is byte-for-byte identical to the entry before it
  // (e.g. two quick taps of the same key). Collapsing adjacent identical
  // entries here is exactly the failure mode that ate repeated letters on
  // real BlueRetro hardware — don't reintroduce it.
  static constexpr u32 MAX_QUEUED_EVENTS = 32;

  void PushEvent(bool pressed, u8 keycode);
  bool PopEvent(KeyEvent* out);

  void BuildResponseFrame();

  std::array<KeyEvent, MAX_QUEUED_EVENTS> m_event_queue{};
  u32 m_queue_head = 0;
  u32 m_queue_count = 0;

  // Tracks whether each possible keycode is currently held, purely so we
  // only enqueue on genuine press/release transitions rather than every
  // time the input system re-asserts an unchanged bind value.
  std::array<bool, 256> m_key_down{};

  // The full 15-byte response for the poll currently in progress, built
  // once (in the Selected state, on seeing 0x42) and then shifted out one
  // byte per subsequent Transfer() call.
  std::array<u8, 15> m_response{}; // matches psxkbd.c's KBD_FRAME_LEN exactly
  u8 m_response_pos = 0;

  TransferState m_transfer_state = TransferState::Idle;
};