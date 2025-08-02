# Unit tests for Pulse Position Animation
#
# This file contains comprehensive tests for the PulsePositionAnimation class
# to ensure it works correctly with various parameters and edge cases.
#
# Command to run tests:
#    ./berry -s -g -m lib/libesp32/berry_animation -e "import tasmota" lib/libesp32/berry_animation/tests/pulse_position_animation_test.be

import animation

# Test counter
var test_count = 0
var passed_count = 0

def test_assert(condition, message)
  test_count += 1
  if condition
    passed_count += 1
    print(f"✓ Test {test_count}: {message}")
  else
    print(f"✗ Test {test_count}: {message}")
  end
end

def run_tests()
  print("Running Pulse Position Animation Tests...")
  print("==================================================")
  
  # Test 1: Basic construction
  var pulse = animation.pulse_position_animation(0xFFFF0000, 3, 2, 1, 0, true, "test_pulse")
  test_assert(pulse != nil, "Pulse position animation creation")
  test_assert(pulse.color == 0xFFFF0000, "Initial color setting")
  test_assert(pulse.pulse_size == 2, "Initial pulse size setting")
  test_assert(pulse.slew_size == 1, "Initial slew size setting")
  test_assert(pulse.pos == 3, "Initial position setting")
  
  # Test 2: Parameter validation and updates
  test_assert(pulse.set_color(0xFF00FF00) == pulse, "Color setter returns self")
  test_assert(pulse.color == 0xFF00FF00, "Color update")
  
  test_assert(pulse.set_pos(5) == pulse, "Position setter returns self")
  test_assert(pulse.pos == 5, "Position update")
  
  test_assert(pulse.set_pulse_size(4) == pulse, "Pulse size setter returns self")
  test_assert(pulse.pulse_size == 4, "Pulse size update")
  
  test_assert(pulse.set_slew_size(3) == pulse, "Slew size setter returns self")
  test_assert(pulse.slew_size == 3, "Slew size update")
  
  test_assert(pulse.set_back_color(0xFF000080) == pulse, "Background color setter returns self")
  test_assert(pulse.back_color == 0xFF000080, "Background color update")
  
  # Test 3: Negative value handling
  pulse.set_pulse_size(-1)
  test_assert(pulse.pulse_size == 0, "Negative pulse size clamped to 0")
  
  pulse.set_slew_size(-2)
  test_assert(pulse.slew_size == 0, "Negative slew size clamped to 0")
  
  # Test 4: Animation lifecycle
  test_assert(!pulse.is_running, "Animation starts stopped")
  pulse.start()
  test_assert(pulse.is_running, "Animation starts correctly")
  pulse.stop()
  test_assert(!pulse.is_running, "Animation stops correctly")
  
  # Test 5: Frame rendering - basic pulse
  var frame = animation.frame_buffer(10)
  pulse.set_color(0xFFFF0000)  # Red
  pulse.set_pos(3)
  pulse.set_pulse_size(2)
  pulse.set_slew_size(0)
  pulse.set_back_color(0xFF000000)  # Transparent
  pulse.start()
  
  var rendered = pulse.render(frame, tasmota.millis())
  test_assert(rendered, "Render returns true when running")
  
  # Check that pixels 3 and 4 are red, others are transparent
  test_assert(frame.get_pixel_color(2) == 0x00000000, "Pixel before pulse is transparent")
  test_assert(frame.get_pixel_color(3) == 0xFFFF0000, "First pulse pixel is red")
  test_assert(frame.get_pixel_color(4) == 0xFFFF0000, "Second pulse pixel is red")
  test_assert(frame.get_pixel_color(5) == 0x00000000, "Pixel after pulse is transparent")
  
  # Test 6: Frame rendering with background
  frame.clear()
  pulse.set_back_color(0xFF000080)  # Dark blue background
  pulse.render(frame, tasmota.millis())
  
  test_assert(frame.get_pixel_color(0) == 0xFF000080, "Background pixel is dark blue")
  test_assert(frame.get_pixel_color(3) == 0xFFFF0000, "Pulse pixel overrides background")
  test_assert(frame.get_pixel_color(9) == 0xFF000080, "Last background pixel is dark blue")
  
  # Test 7: Frame rendering with slew
  frame.clear()
  pulse.set_back_color(0xFF000000)  # Transparent background
  pulse.set_pos(4)
  pulse.set_pulse_size(2)
  pulse.set_slew_size(1)
  pulse.render(frame, tasmota.millis())
  
  # Check main pulse
  test_assert(frame.get_pixel_color(4) == 0xFFFF0000, "Main pulse pixel 1 is red")
  test_assert(frame.get_pixel_color(5) == 0xFFFF0000, "Main pulse pixel 2 is red")
  
  # Check slew regions have some color (not fully transparent, not fully red)
  var left_slew = frame.get_pixel_color(3)
  var right_slew = frame.get_pixel_color(6)
  test_assert(left_slew != 0x00000000 && left_slew != 0xFFFF0000, "Left slew has blended color")
  # Debug the right slew
  # print(f"DEBUG: right_slew = 0x{right_slew:08X}, expected != 0x00000000 && != 0xFFFF0000")
  test_assert(right_slew != 0x00000000 && right_slew != 0xFFFF0000, "Right slew has blended color")
  
  # Test 8: Edge cases - pulse at boundaries
  frame.clear()
  pulse.set_pos(0)
  pulse.set_pulse_size(2)
  pulse.set_slew_size(1)
  pulse.render(frame, tasmota.millis())
  
  test_assert(frame.get_pixel_color(0) == 0xFFFF0000, "Pulse at start boundary works")
  test_assert(frame.get_pixel_color(1) == 0xFFFF0000, "Pulse at start boundary works")
  
  frame.clear()
  pulse.set_pos(8)
  pulse.set_pulse_size(2)
  pulse.set_slew_size(1)
  pulse.render(frame, tasmota.millis())
  
  test_assert(frame.get_pixel_color(8) == 0xFFFF0000, "Pulse at end boundary works")
  test_assert(frame.get_pixel_color(9) == 0xFFFF0000, "Pulse at end boundary works")
  
  # Test 9: Zero-width pulse (only slew)
  frame.clear()
  pulse.set_pos(5)
  pulse.set_pulse_size(0)
  pulse.set_slew_size(2)
  pulse.render(frame, tasmota.millis())
  
  # Should have slew on both sides but no main pulse
  var left_slew1 = frame.get_pixel_color(3)
  var left_slew2 = frame.get_pixel_color(4)
  var right_slew1 = frame.get_pixel_color(5)
  var right_slew2 = frame.get_pixel_color(6)
  
  test_assert(left_slew1 != 0x00000000, "Zero-width pulse has left slew")
  test_assert(left_slew2 != 0x00000000, "Zero-width pulse has left slew")
  test_assert(right_slew1 != 0x00000000, "Zero-width pulse has right slew")
  test_assert(right_slew2 != 0x00000000, "Zero-width pulse has right slew")
  
  # Test 10: Stopped animation doesn't render
  pulse.stop()
  frame.clear()
  var rendered_stopped = pulse.render(frame, tasmota.millis())
  test_assert(!rendered_stopped, "Stopped animation doesn't render")
  test_assert(frame.get_pixel_color(5) == 0x00000000, "Frame remains clear when animation stopped")
  
  # Test 11: Parameter constraints
  test_assert(pulse.set_param("pos", 15), "Valid position parameter accepted")
  test_assert(pulse.pos == 15, "Position parameter updated")
  
  test_assert(pulse.set_param("pulse_size", 5), "Valid pulse size parameter accepted")
  test_assert(pulse.pulse_size == 5, "Pulse size parameter updated")
  
  test_assert(pulse.set_param("slew_size", 3), "Valid slew size parameter accepted")
  test_assert(pulse.slew_size == 3, "Slew size parameter updated")
  
  # Test 12: String representation
  var str_repr = pulse.tostring()
  test_assert(type(str_repr) == "string", "String representation returns string")
  import string
  test_assert(string.find(str_repr, "PulsePositionAnimation") >= 0, "String representation contains class name")
  
  print("==================================================")
  print(f"Tests completed: {passed_count}/{test_count} passed")
  
  if passed_count == test_count
    print("🎉 All tests passed!")
    return true
  else
    print(f"❌ {test_count - passed_count} tests failed")
    raise "test_failed"
  end
end

# Run the tests
var success = run_tests()

return success