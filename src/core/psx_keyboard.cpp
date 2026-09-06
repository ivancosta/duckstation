// SPDX-FileCopyrightText: 2019-2026 Connor McLaughlin <stenzek@gmail.com> and contributors
// SPDX-License-Identifier: CC-BY-NC-ND-4.0
//
// PSXKeyboard
// -----------
// See psx_keyboard.h for the general idea.
//
// WIRE FORMAT — confirmed directly against blackroo's own
// drivers/char/psxkbd.c (KBD_FRAME_LEN/KBD_ID/KBD_READY and the
// psxkbd_frame()/psxkbd_decode() functions), which in turn was verified
// against real hardware via BRMON's `kbd` command before that driver was
// written (see docs/27-KEYBOARD-BRINGUP.md). Independently corroborated by
// a BlueRetro<->Lightspan-keyboard bus trace published by BlueRetro's own
// author while reverse-engineering the real Lightspan PSX Keyboard:
//
//   Make 'Backspace':  TX: 01 42 00×13   RX: FF 96 5A 01 66 00×10
//   Break 'Backspace': TX: 01 42 00×13   RX: FF 96 5A 02 F0 66 00×9
//
// i.e. a fixed 15-byte exchange (KBD_FRAME_LEN), always: FF (dummy), 0x96,
// 0x5A (ID), <length>, then up to 11 bytes of PS/2 Scan Code Set 2 data,
// zero-padded out to 15 bytes total. Make = the base scancode (1 byte).
// Break = 0xF0 then the base scancode (2 bytes). Extended keys (arrows,
// Ins/Del/Home/End/PgUp/PgDn, right-side Ctrl/Alt/Win) get an 0xE0 prefix
// on top of that. At most one key transition is reported per poll;
// <length>=0 means nothing happened this poll, matching psxkbd.c's own
// documented rationale for not de-duplicating identical frames.
//
// Keyboard implementation by Ivan F. C. Costa

#include "psx_keyboard.h"

#include "util/state_wrapper.h"
#include "util/translation.h"

#include "common/assert.h"
#include "common/log.h"

LOG_CHANNEL(Controller);

PSXKeyboard::PSXKeyboard(u32 index) : Controller(index)
{
}

PSXKeyboard::~PSXKeyboard() = default;

std::unique_ptr<PSXKeyboard> PSXKeyboard::Create(u32 index)
{
  return std::make_unique<PSXKeyboard>(index);
}

ControllerType PSXKeyboard::GetType() const
{
  return ControllerType::PSXKeyboard;
}

void PSXKeyboard::Reset()
{
  m_transfer_state = TransferState::Idle;
  m_response.fill(0);
  m_response_pos = 0;
  m_queue_head = 0;
  m_queue_count = 0;
  m_event_queue.fill(KeyEvent{});
  m_key_down.fill(false);
}

bool PSXKeyboard::DoState(StateWrapper& sw, bool apply_input_state)
{
  std::array<KeyEvent, MAX_QUEUED_EVENTS> event_queue = m_event_queue;
  u32 queue_head = m_queue_head;
  u32 queue_count = m_queue_count;
  std::array<bool, 256> key_down = m_key_down;
  std::array<u8, 15> response = m_response;
  u8 response_pos = m_response_pos;

  for (KeyEvent& ev : event_queue)
  {
    sw.Do(&ev.pressed);
    sw.Do(&ev.keycode);
  }
  sw.Do(&queue_head);
  sw.Do(&queue_count);
  for (bool& down : key_down)
    sw.Do(&down);
  for (u8& b : response)
    sw.Do(&b);
  sw.Do(&response_pos);
  sw.Do(&m_transfer_state);

  if (apply_input_state)
  {
    m_event_queue = event_queue;
    m_queue_head = queue_head;
    m_queue_count = queue_count;
    m_key_down = key_down;
    m_response = response;
    m_response_pos = response_pos;
  }

  return !sw.HasError();
}

void PSXKeyboard::ResetTransferState()
{
  m_transfer_state = TransferState::Idle;
  m_response_pos = 0;
}

void PSXKeyboard::PushEvent(bool pressed, u8 keycode)
{
  if (m_queue_count >= MAX_QUEUED_EVENTS)
  {
    // 32 deep at a >=60Hz poll rate is far beyond any human typing rate;
    // if this ever fires, something upstream is spamming SetBindState.
    // Drop the newest rather than evicting the oldest still-pending event.
    return;
  }

  const u32 tail = (m_queue_head + m_queue_count) % MAX_QUEUED_EVENTS;
  m_event_queue[tail] = KeyEvent{pressed, keycode};
  m_queue_count++;
}

bool PSXKeyboard::PopEvent(KeyEvent* out)
{
  if (m_queue_count == 0)
    return false;

  *out = m_event_queue[m_queue_head];
  m_queue_head = (m_queue_head + 1) % MAX_QUEUED_EVENTS;
  m_queue_count--;
  return true;
}

void PSXKeyboard::BuildResponseFrame()
{
  m_response.fill(0);
  m_response[0] = 0xFF;
  m_response[1] = 0x96;
  m_response[2] = 0x5A;

  KeyEvent ev;
  if (!PopEvent(&ev))
  {
    m_response[3] = 0; // nothing pending this poll
    return;
  }

  const bool extended = (ev.keycode & 0x80) != 0;
  const u8 code = ev.keycode & 0x7F;

  u8 n = 0;
  u8* p = &m_response[4];
  if (ev.pressed)
  {
    if (extended)
      p[n++] = 0xE0;
    p[n++] = code;
  }
  else
  {
    if (extended)
      p[n++] = 0xE0;
    p[n++] = 0xF0;
    p[n++] = code;
  }
  m_response[3] = n;

  // TEMP DIAGNOSTIC: using ERROR_LOG (not DEV_LOG) deliberately, so this
  // shows up in DuckStation's log window regardless of your configured log
  // verbosity. Revert to DEV_LOG once things work.
  // ERROR_LOG("PSXKeyboard[{}]: TX {} scancode=0x{:02X}{} len={}", m_index, ev.pressed ? "MAKE" : "BREAK", code,
  //         extended ? " EXTENDED" : "", n);
}

bool PSXKeyboard::Transfer(const u8 data_in, u8* data_out)
{
  switch (m_transfer_state)
  {
    case TransferState::Idle:
    {
      *data_out = 0xFF;
      if (data_in == 0x01)
      {
        m_transfer_state = TransferState::Selected;
        return true;
      }
      return false;
    }

    case TransferState::Selected:
    {
      if (data_in == 0x42)
      {
        BuildResponseFrame();
        m_response_pos = 1;
        *data_out = m_response[1]; // 0x96
        m_transfer_state = TransferState::Responding;
        return true;
      }
      *data_out = 0xFF;
      return false;
    }

    case TransferState::Responding:
    {
      m_response_pos++;
      *data_out = m_response[m_response_pos];
      if (m_response_pos >= 14)
      {
        m_transfer_state = TransferState::Idle;
        return false;
      }
      return true;
    }

    default:
      UnreachableCode();
  }
}

float PSXKeyboard::GetBindState(u32 index) const
{
  if (index > 0xFF)
    return 0.0f;

  return m_key_down[index] ? 1.0f : 0.0f;
}

void PSXKeyboard::SetBindState(u32 index, float value)
{
  if (index > 0xFF)
    return;

  const u8 keycode = static_cast<u8>(index);
  const bool pressed = (value >= 0.5f);

  // Edge-detect on the way in: only enqueue on genuine press/release
  // transitions, not every time the input system re-asserts an unchanged
  // bind value while a key is held.
  if (pressed == m_key_down[keycode])
    return;

  m_key_down[keycode] = pressed;
  PushEvent(pressed, keycode);

  // DIAGNOSTIC:
  // ERROR_LOG("PSXKeyboard[{}]: keycode 0x{:02X} {}", m_index, keycode, pressed ? "DOWN" : "UP");
}

u32 PSXKeyboard::GetButtonStateBits() const
{
  // See the note in the header: this device is modeled as a stream of
  // discrete events, not a steady-state bitmask, so there's nothing
  // meaningful to report here for on-screen overlays / rewind detection.
  return 0;
}

void PSXKeyboard::LoadSettings(const SettingsInterface& si, const char* section, bool initial)
{
  // No per-controller settings — intentionally empty.
}

// ---------------------------------------------------------------------------
// Binding table / ControllerInfo
//
// Reuses DuckStation's ordinary per-controller binding UI: each key below
// shows up in Settings -> Controllers exactly like a DigitalController
// button does. Plain positional initializers (not designated) — see the
// note on this in an earlier revision: MSVC's C++20 designated-initializer
// support has bugs when nested inside another aggregate's brace-init-list.
//
// PrintScreen and Pause are deliberately omitted: both use irregular,
// non-make/break Set 2 sequences (Pause has no break code at all) that
// don't fit this driver's simple make/break model, and neither is likely
// to matter for a shell.
// ---------------------------------------------------------------------------

#define KEY(str_name, disp_name, key)                                                                                  \
  {str_name,                                                                                                           \
   disp_name,                                                                                                          \
   nullptr,                                                                                                            \
   static_cast<u32>(PSXKeyboard::Key::key),                                                                            \
   InputBindingInfo::Type::Button,                                                                                     \
   GenericInputBinding::Unknown}

static const Controller::ControllerBindingInfo s_binding_info[] = {
  // clang-format off
    KEY("A", "A", A), KEY("B", "B", B), KEY("C", "C", C), KEY("D", "D", D),
    KEY("E", "E", E), KEY("F", "F", F), KEY("G", "G", G), KEY("H", "H", H),
    KEY("I", "I", I), KEY("J", "J", J), KEY("K", "K", K), KEY("L", "L", L),
    KEY("M", "M", M), KEY("N", "N", N), KEY("O", "O", O), KEY("P", "P", P),
    KEY("Q", "Q", Q), KEY("R", "R", R), KEY("S", "S", S), KEY("T", "T", T),
    KEY("U", "U", U), KEY("V", "V", V), KEY("W", "W", W), KEY("X", "X", X),
    KEY("Y", "Y", Y), KEY("Z", "Z", Z),

    KEY("Num1", "1", Num1), KEY("Num2", "2", Num2), KEY("Num3", "3", Num3),
    KEY("Num4", "4", Num4), KEY("Num5", "5", Num5), KEY("Num6", "6", Num6),
    KEY("Num7", "7", Num7), KEY("Num8", "8", Num8), KEY("Num9", "9", Num9),
    KEY("Num0", "0", Num0),

    KEY("Enter", "Enter", Enter), KEY("Escape", "Escape", Escape),
    KEY("Backspace", "Backspace", Backspace), KEY("Tab", "Tab", Tab),
    KEY("Space", "Space", Space), KEY("Minus", "- (minus)", Minus),
    KEY("Equals", "= (equals)", Equals), KEY("LeftBracket", "[ (left bracket)", LeftBracket),
    KEY("RightBracket", "] (right bracket)", RightBracket), KEY("Backslash", "\\ (backslash)", Backslash),
    KEY("Semicolon", "; (semicolon)", Semicolon), KEY("Apostrophe", "' (apostrophe)", Apostrophe),
    KEY("Grave", "` (backtick)", Grave), KEY("Comma", ", (comma)", Comma),
    KEY("Period", ". (period)", Period), KEY("Slash", "/ (slash)", Slash),
    KEY("CapsLock", "Caps Lock", CapsLock),

    KEY("F1", "F1", F1), KEY("F2", "F2", F2), KEY("F3", "F3", F3),
    KEY("F4", "F4", F4), KEY("F5", "F5", F5), KEY("F6", "F6", F6),
    KEY("F7", "F7", F7), KEY("F8", "F8", F8), KEY("F9", "F9", F9),
    KEY("F10", "F10", F10), KEY("F11", "F11", F11), KEY("F12", "F12", F12),
    KEY("ScrollLock", "Scroll Lock", ScrollLock),

    KEY("Insert", "Insert", Insert), KEY("Home", "Home", Home),
    KEY("PageUp", "Page Up", PageUp), KEY("Delete", "Delete", Delete),
    KEY("End", "End", End), KEY("PageDown", "Page Down", PageDown),
    KEY("Right", "Right Arrow", Right), KEY("Left", "Left Arrow", Left),
    KEY("Down", "Down Arrow", Down), KEY("Up", "Up Arrow", Up),

    KEY("LeftCtrl", "Left Ctrl", LeftCtrl), KEY("LeftShift", "Left Shift", LeftShift),
    KEY("LeftAlt", "Left Alt", LeftAlt), KEY("LeftGui", "Left Win/Cmd", LeftGui),
    KEY("RightCtrl", "Right Ctrl", RightCtrl), KEY("RightShift", "Right Shift", RightShift),
    KEY("RightAlt", "Right Alt", RightAlt), KEY("RightGui", "Right Win/Cmd", RightGui),
  // clang-format on
};

#undef KEY

// Positional, matching upstream's own DigitalController::INFO / s_none_info
// literals: {type, name, display_name, icon_name, image_name, bindings,
// settings}. If your checkout's ControllerInfo has a different field
// count, add/remove a nullptr/{} here to match.
const Controller::ControllerInfo PSXKeyboard::INFO = {ControllerType::PSXKeyboard,
                                                      "PSXKeyboard",
                                                      TRANSLATE_NOOP("ControllerType", "PSX Keyboard (BlueRetro)"),
                                                      nullptr, // icon_name (font glyph) - none
                                                      nullptr, // image_name (svg resource path) - none
                                                      s_binding_info,
                                                      {}};