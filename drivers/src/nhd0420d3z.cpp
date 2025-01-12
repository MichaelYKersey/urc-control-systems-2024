#include <array>
#include <libhal-util/i2c.hpp>
#include <libhal/error.hpp>
#include <libhal/units.hpp>
#include <nhd0420d3z.hpp>

namespace {
enum commands : hal::byte
{
  prefix = 0xFE,
  display_on = 0x41,
  display_off = 0x42,
  set_cursor = 0x45,
  cursor_home = 0x46,
  underline_cursor_on = 0x47,
  underline_cursor_off = 0x48,
  move_cursor_left_one_place = 0x49,
  move_cursor_right_one_place = 0x4A,
  blinking_cursor_on = 0x4B,
  blinking_cursor_off = 0x4C,
  backspace = 0x4E,
  clear_screen = 0x51,
  set_contrast = 0x52,
  set_backlight_brightness = 0x53,
  load_custom_character = 0x54,
  move_display_one_place_to_the_left = 0x55,
  move_display_one_place_to_the_right = 0x56,
};
}

namespace sjsu::drivers {
hal::byte nhd0420d3z::coordinates_to_position(hal::byte line, hal::byte column)
{
  hal::byte pos = line;
  pos += (column / 2) * display_lines;
  if (column % 2) {
    pos += 0x40;
  }
  return pos;
}
void nhd0420d3z::write_char(char c)
{
  if (m_cursor_line >= display_lines || m_cursor_column >= display_columns)
    return;
  std::array<hal::byte, 1> command = { c };
  hal::write(m_i2c_bus, m_i2c_address, command);
}
void nhd0420d3z::set_cursor_position(hal::byte line, hal::byte column)
{
  std::array<hal::byte, 3> command = { commands::prefix,
                                       commands::set_cursor,
                                       coordinates_to_position(line, column) };
  hal::write(m_i2c_bus, m_i2c_address, command);
  m_cursor_line = line;
  m_cursor_column = column;
}

void nhd0420d3z::move_cursor_right()
{
  m_cursor_line++;
  if (m_cursor_line >= display_columns - 1)
    return;
  constexpr std::array<hal::byte, 2> command = {
    commands::prefix, commands::move_cursor_right_one_place
  };
  hal::write(m_i2c_bus, m_i2c_address, command);
}
void nhd0420d3z::home_cursor()
{
  constexpr std::array<hal::byte, 2> command = { commands::prefix,
                                                 commands::cursor_home };
  hal::write(m_i2c_bus, m_i2c_address, command);
  m_cursor_column = 0;
  m_cursor_line = 0;
}
void nhd0420d3z::clear_screen()
{
  std::array<hal::byte, 2> command = { commands::prefix,
                                       commands::clear_screen };
  hal::write(m_i2c_bus, m_i2c_address, command);
}

nhd0420d3z::nhd0420d3z(hal::i2c& p_i2c, hal::byte p_i2c_address)
  : m_i2c_bus(p_i2c)
  , m_i2c_address(p_i2c_address)
{
}
void nhd0420d3z::display_message(std::string_view str)
{
  home_cursor();
  clear_screen();
  auto strIt = str.begin();
  while (m_cursor_line < display_lines && strIt != str.end()) {
    if (*strIt == '\n') {
      set_cursor_position(0, m_cursor_column + 1);
    } else {
      write_char(*strIt);
    }
    strIt++;
  }
}
void nhd0420d3z::power(bool on)
{
  if (on) {
    constexpr std::array<hal::byte, 2> command = { commands::prefix,
                                                   commands::display_on };
    hal::write(m_i2c_bus, m_i2c_address, command);
  } else {
    constexpr std::array<hal::byte, 2> command = { commands::prefix,
                                                   commands::display_off };
    hal::write(m_i2c_bus, m_i2c_address, command);
  }
}
}  // namespace sjsu::drivers