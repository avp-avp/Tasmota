# DSL Core Processing Methods Test Suite
# Tests for the simplified DSL transpiler's core processing capabilities
#
# Command to run test is:
#    ./berry -s -g -m lib/libesp32/berry_animation -e "import tasmota" lib/libesp32/berry_animation/tests/dsl_core_processing_test.be

import tasmota
import animation
import string

# Test basic color processing
def test_color_processing()
  print("Testing color processing...")
  
  # Test hex colors
  var hex_tests = [
    ["color custom_red = 0xFF0000", "var custom_red_ = 0xFFFF0000"],
    ["color custom_blue = 0x0000FF", "var custom_blue_ = 0xFF0000FF"],
    ["color custom_green = 0x00FF00", "var custom_green_ = 0xFF00FF00"]
  ]
  
  for test : hex_tests
    var dsl_input = test[0]
    var expected_output = test[1]
    
    var berry_code = animation.compile_dsl(dsl_input)
    assert(berry_code != nil, "Should compile color: " + dsl_input)
    assert(string.find(berry_code, expected_output) >= 0, "Should contain: " + expected_output)
  end
  
  # Test named colors
  var named_color_tests = [
    ["color my_red = red", "var my_red_ = 0xFFFF0000"],
    ["color my_blue = blue", "var my_blue_ = 0xFF0000FF"],
    ["color my_white = white", "var my_white_ = 0xFFFFFFFF"]
  ]
  
  for test : named_color_tests
    var dsl_input = test[0]
    var expected_output = test[1]
    
    var berry_code = animation.compile_dsl(dsl_input)
    assert(berry_code != nil, "Should compile named color: " + dsl_input)
    assert(string.find(berry_code, expected_output) >= 0, "Should contain: " + expected_output)
  end
  
  print("✓ Color processing test passed")
  return true
end

# Test basic pattern processing
def test_pattern_processing()
  print("Testing pattern processing...")
  
  # Test solid patterns
  var pattern_tests = [
    ["color red_alt = 0xFF0100\n"
     "pattern solid_red = solid(red_alt)",
     "var solid_red_ = animation.solid(animation.global('red_alt_', 'red_alt'))"],
    ["pattern solid_blue = solid(blue)",
     "var solid_blue_ = animation.solid(0xFF0000FF)"]
  ]
  
  for test : pattern_tests
    var dsl_input = test[0]
    var expected_output = test[1]
    
    var berry_code = animation.compile_dsl(dsl_input)
    assert(berry_code != nil, "Should compile pattern: " + dsl_input)
    assert(string.find(berry_code, expected_output) >= 0, "Should contain: " + expected_output)
  end
  
  print("✓ Pattern processing test passed")
  return true
end

# Test basic animation processing
def test_animation_processing()
  print("Testing animation processing...")
  
  # Test direct color to animation
  var color_anim_tests = [
    ["color red_alt = 0xFF0100\n"
     "animation red_anim = red_alt",
     "var red_anim_ = animation.global('red_alt_', 'red_alt')"],
    ["animation blue_anim = blue",
     "var blue_anim_ = 0xFF0000FF"]
  ]
  
  for test : color_anim_tests
    var dsl_input = test[0]
    var expected_output = test[1]
    
    var berry_code = animation.compile_dsl(dsl_input)
    assert(berry_code != nil, "Should compile color animation: " + dsl_input)
    assert(string.find(berry_code, expected_output) >= 0, "Should contain: " + expected_output)
  end
  
  # Test pattern to animation
  var pattern_anim_tests = [
    ["pattern solid_red = solid(red)\n"
     "animation red_anim = solid_red", 
     "var red_anim_ = animation.global('solid_red_', 'solid_red')"]
  ]
  
  for test : pattern_anim_tests
    var dsl_input = test[0]
    var expected_output = test[1]
    
    var berry_code = animation.compile_dsl(dsl_input)
    assert(berry_code != nil, "Should compile pattern animation: " + dsl_input)
    assert(string.find(berry_code, expected_output) >= 0, "Should contain: " + expected_output)
  end
  
  # Test solid() as animation
  var solid_anim_tests = [
    ["pattern solid_red = solid(red)\n"
     "animation red_anim = solid_red",
     "var red_anim_ = animation.global('solid_red_', 'solid_red')"]
  ]
  
  for test : solid_anim_tests
    var dsl_input = test[0]
    var expected_output = test[1]
    
    var berry_code = animation.compile_dsl(dsl_input)
    assert(berry_code != nil, "Should compile solid animation: " + dsl_input)
    assert(string.find(berry_code, expected_output) >= 0, "Should contain: " + expected_output)
  end
  
  # Test pulse animations
  var pulse_tests = [
    ["pattern solid_red = solid(red)\n"
     "animation pulse_red = pulse_animation(solid_red, 2s)",
     "var pulse_red_ = animation.pulse_animation(animation.global('solid_red_', 'solid_red'), 2000)"]
  ]
  
  for test : pulse_tests
    var dsl_input = test[0]
    var expected_output = test[1]
    
    var berry_code = animation.compile_dsl(dsl_input)
    # print("Generated Berry code:")
    # print("==================================================")
    # print(berry_code)
    # print("==================================================")
    assert(berry_code != nil, "Should compile pulse animation: " + dsl_input)
    assert(string.find(berry_code, expected_output) >= 0, "Should contain: " + expected_output)
  end
  
  print("✓ Animation processing test passed")
  return true
end

# Test strip configuration
def test_strip_configuration()
  print("Testing strip configuration...")
  
  var strip_tests = [
    ["strip length 30", "var engine = animation.init_strip(30)"],
    ["strip length 60", "var engine = animation.init_strip(60)"],
    ["strip length 120", "var engine = animation.init_strip(120)"]
  ]
  
  for test : strip_tests
    var dsl_input = test[0]
    var expected_output = test[1]
    
    var berry_code = animation.compile_dsl(dsl_input)
    assert(berry_code != nil, "Should compile strip config: " + dsl_input)
    assert(string.find(berry_code, expected_output) >= 0, "Should contain: " + expected_output)
  end
  
  print("✓ Strip configuration test passed")
  return true
end

# Test variable assignments
def test_variable_assignments()
  print("Testing variable assignments...")
  
  var var_tests = [
    ["set brightness = 75%", "var brightness_ = 191"],  # 75% of 255 = 191.25 -> 191
    ["set duration = 3s", "var duration_ = 3000"],     # 3 seconds in ms
    ["set count = 5", "var count_ = 5"],               # Plain number
    ["set opacity = 50%", "var opacity_ = 127"]        # 50% of 255 = 127.5 -> 127
  ]
  
  for test : var_tests
    var dsl_input = test[0]
    var expected_output = test[1]
    
    var berry_code = animation.compile_dsl(dsl_input)
    assert(berry_code != nil, "Should compile variable: " + dsl_input)
    assert(string.find(berry_code, expected_output) >= 0, "Should contain: " + expected_output)
  end
  
  print("✓ Variable assignments test passed")
  return true
end

# Test sequence processing
def test_sequence_processing()
  print("Testing sequence processing...")
  
  # Test basic sequence
  var basic_seq_dsl = "color custom_red = 0xFF0000\n" +
    "animation red_anim = custom_red\n" +
    "sequence demo {\n" +
    "  play red_anim for 2s\n" +
    "}\n" +
    "run demo"
  
  var berry_code = animation.compile_dsl(basic_seq_dsl)

  assert(berry_code != nil, "Should compile basic sequence")
  assert(string.find(berry_code, "def sequence_demo()") >= 0, "Should define sequence function")
  assert(string.find(berry_code, "red_anim") >= 0, "Should reference animation")
  assert(string.find(berry_code, "animation.create_play_step(animation.global('red_anim_'), 2000)") >= 0, "Should create play step")
  assert(string.find(berry_code, "var seq_manager = global.sequence_demo()") >= 0, "Should call sequence")
  assert(string.find(berry_code, "engine.add_sequence_manager(seq_manager)") >= 0, "Should add sequence manager")
  assert(string.find(berry_code, "engine.start()") >= 0, "Should start engine")
  
  # Test repeat in sequence
  var repeat_seq_dsl = "color custom_blue = 0x0000FF\n" +
    "animation blue_anim = custom_blue\n" +
    "sequence test {\n" +
    "  repeat 3 times:\n" +
    "    play blue_anim for 1s\n" +
    "    wait 500ms\n" +
    "}\n" +
    "run test"
  
  berry_code = animation.compile_dsl(repeat_seq_dsl)
  
  # print("Generated Berry code:")
  # print("==================================================")
  # print(berry_code)
  # print("==================================================")
  assert(berry_code != nil, "Should compile repeat sequence")
  assert(string.find(berry_code, "for repeat_i : 0..3-1") >= 0, "Should generate repeat loop")
  assert(string.find(berry_code, "animation.create_wait_step(500)") >= 0, "Should generate wait step")
  
  print("✓ Sequence processing test passed")
  return true
end

# Test time and percentage conversions
def test_value_conversions()
  print("Testing value conversions...")
  
  # Test time conversions
  var time_tests = [
    ["set delay = 1s", "var delay_ = 1000"],
    ["set delay = 500ms", "var delay_ = 500"],
    ["set delay = 2m", "var delay_ = 120000"],
    ["set delay = 1h", "var delay_ = 3600000"]
  ]
  
  for test : time_tests
    var dsl_input = test[0]
    var expected_output = test[1]
    
    var berry_code = animation.compile_dsl(dsl_input)
    assert(berry_code != nil, "Should compile time value: " + dsl_input)
    assert(string.find(berry_code, expected_output) >= 0, "Should contain: " + expected_output)
  end
  
  # Test percentage conversions
  var percent_tests = [
    ["set opacity = 0%", "var opacity_ = 0"],
    ["set opacity = 25%", "var opacity_ = 63"],    # 25% of 255 = 63.75 -> 63
    ["set opacity = 50%", "var opacity_ = 127"],   # 50% of 255 = 127.5 -> 127
    ["set opacity = 100%", "var opacity_ = 255"]   # 100% of 255
  ]
  
  for test : percent_tests
    var dsl_input = test[0]
    var expected_output = test[1]
    
    var berry_code = animation.compile_dsl(dsl_input)
    assert(berry_code != nil, "Should compile percentage: " + dsl_input)
    assert(string.find(berry_code, expected_output) >= 0, "Should contain: " + expected_output)
  end
  
  print("✓ Value conversions test passed")
  return true
end

# Test property assignments
def test_property_assignments()
  print("Testing property assignments...")
  
  var property_tests = [
    ["color custom_red = 0xFF0000\nanimation red_anim = solid(custom_red)\nred_anim.pos = 15", 
     "animation.global('red_anim_').pos = 15"],
    ["animation test_anim = solid(blue)\ntest_anim.opacity = 128", 
     "animation.global('test_anim_').opacity = 128"],
    ["animation pulse_anim = pulse(solid(red), 2s)\npulse_anim.priority = 5", 
     "animation.global('pulse_anim_').priority = 5"]
  ]
  
  for test : property_tests
    var dsl_input = test[0]
    var expected_output = test[1]
    
    var berry_code = animation.compile_dsl(dsl_input)
    assert(berry_code != nil, "Should compile property assignment: " + dsl_input)
    assert(string.find(berry_code, expected_output) >= 0, "Should contain: " + expected_output)
  end
  
  print("✓ Property assignments test passed")
  return true
end

# Test reserved name validation
def test_reserved_name_validation()
  print("Testing reserved name validation...")
  print("  (Expected: Exceptions for reserved names)")
  
  # Test predefined color rejection
  var predefined_color_tests = [
    "color red = 0x800000",      # Predefined color
    "color blue = 0x000080",     # Predefined color
    "color green = 0x008000",    # Predefined color
    "color white = 0xFFFFFF",    # Predefined color
    "color black = 0x000000",    # Predefined color
    "color yellow = 0xFFFF00",   # Predefined color
    "color orange = 0xFFA500",   # Predefined color
    "color purple = 0x800080"    # Predefined color
  ]
  
  for dsl_input : predefined_color_tests
    var exception_caught = false
    var error_message = ""
    
    try
      var berry_code = animation.compile_dsl(dsl_input)
      assert(false, "Should have raised exception for predefined color: " + dsl_input)
    except "dsl_compilation_error" as e, msg
      exception_caught = true
      error_message = msg
    end
    
    assert(exception_caught, "Should raise dsl_compilation_error for: " + dsl_input)
    
    # Check that error message mentions the predefined color
    var color_name = string.split(dsl_input, " ")[1]  # Extract color name
    assert(string.find(error_message, "Cannot redefine predefined color '" + color_name + "'") >= 0, 
           "Should have specific error for predefined color: " + color_name)
  end
  
  # Test DSL keyword rejection (these should be handled by existing system)
  var dsl_keyword_tests = [
    "color color = 0xFF0000",    # DSL keyword
    "animation strip = solid(red)"  # DSL keyword
    # Note: easing functions (smooth, linear, etc.) are no longer keywords
  ]
  
  for dsl_input : dsl_keyword_tests
    var exception_caught = false
    
    try
      var berry_code = animation.compile_dsl(dsl_input)
      assert(false, "Should have raised exception for DSL keyword: " + dsl_input)
    except "dsl_compilation_error" as e, msg
      exception_caught = true
      # DSL keywords should fail at the parser level with different error messages
    end
    
    assert(exception_caught, "Should raise dsl_compilation_error for DSL keyword: " + dsl_input)
  end
  
  # Test valid custom names (should succeed)
  var valid_name_tests = [
    "color my_red = 0xFF0000",
    "color custom_blue = 0x0000FF", 
    "color fire_color = 0xFF4500",
    "color ocean_blue = 0x006994",
    "color red_custom = 0x800000",
    "color smooth_custom = 0x808080",
    # Easing function names are now valid as user-defined names
    "pattern smooth = solid(blue)",
    "animation linear = solid(green)"
  ]
  
  for dsl_input : valid_name_tests
    try
      var berry_code = animation.compile_dsl(dsl_input)
      assert(berry_code != nil, "Should accept valid custom name: " + dsl_input)
    except "dsl_compilation_error" as e, msg
      assert(false, "Should not raise exception for valid name: " + dsl_input + " - Error: " + msg)
    end
  end
  
  print("✓ Reserved name validation test passed")
  return true
end

# Run all tests
def run_core_processing_tests()
  print("=== DSL Core Processing Methods Test Suite ===")
  print("Testing simplified transpiler's core processing capabilities...")
  print("")
  
  var tests = [
    test_color_processing,
    test_pattern_processing,
    test_animation_processing,
    test_strip_configuration,
    test_variable_assignments,
    test_sequence_processing,
    test_value_conversions,
    test_property_assignments,
    test_reserved_name_validation
  ]
  
  var passed = 0
  var total = size(tests)
  
  for test_func : tests
    try
      if test_func()
        passed += 1
      else
        print("✗ Test failed")
      end
    except .. as error_type, error_message
      print("✗ Test crashed: " + str(error_type) + " - " + str(error_message))
    end
    print("")  # Add spacing between tests
  end
  
  print("=== Core Processing Results: " + str(passed) + "/" + str(total) + " tests passed ===")
  
  if passed == total
    print("🎉 All DSL core processing tests passed!")
    return true
  else
    print("❌ Some DSL core processing tests failed")
    raise "test_failed"
  end
end

# Auto-run tests when file is executed
run_core_processing_tests()