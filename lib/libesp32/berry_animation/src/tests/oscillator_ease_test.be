# Test suite for OscillatorValueProvider ease_in and ease_out functionality
#
# This test verifies that the new EASE_IN and EASE_OUT waveforms work correctly
# and produce the expected easing curves.

import animation
import string

# Test the EASE_IN waveform
def test_ease_in_waveform()
  print("Testing EASE_IN waveform...")
  
  var provider = animation.oscillator_value_provider(0, 100, 1000, animation.EASE_IN)
  
  # Test at key points in the cycle
  # At t=0, should be at starting value (0)
  var value_0 = provider.get_value(provider.origin)
  assert(value_0 == 0, f"EASE_IN at t=0 should be 0, got {value_0}")
  
  # At t=250ms (25% through), should be around 6.25 (25%^2 of 100)
  var value_25 = provider.get_value(provider.origin + 250)
  assert(value_25 >= 5 && value_25 <= 8, f"EASE_IN at 25% should be ~6, got {value_25}")
  
  # At t=500ms (50% through), should be around 25 (50%^2 of 100)
  var value_50 = provider.get_value(provider.origin + 500)
  assert(value_50 >= 23 && value_50 <= 27, f"EASE_IN at 50% should be ~25, got {value_50}")
  
  # At t=750ms (75% through), should be around 56.25 (75%^2 of 100)
  var value_75 = provider.get_value(provider.origin + 750)
  assert(value_75 >= 54 && value_75 <= 58, f"EASE_IN at 75% should be ~56, got {value_75}")
  
  # At t=1000ms (100% through), should be at end value (100)
  var value_100 = provider.get_value(provider.origin + 999)  # Just before wrap
  assert(value_100 >= 98 && value_100 <= 100, f"EASE_IN at 100% should be ~100, got {value_100}")
  
  # Verify the curve is accelerating (derivative increasing)
  assert(value_25 - value_0 < value_50 - value_25, "EASE_IN should accelerate")
  assert(value_50 - value_25 < value_75 - value_50, "EASE_IN should continue accelerating")
  
  print("✓ EASE_IN waveform test passed")
end

# Test the EASE_OUT waveform
def test_ease_out_waveform()
  print("Testing EASE_OUT waveform...")
  
  var provider = animation.oscillator_value_provider(0, 100, 1000, animation.EASE_OUT)
  
  # Test at key points in the cycle
  # At t=0, should be at starting value (0)
  var value_0 = provider.get_value(provider.origin)
  assert(value_0 == 0, f"EASE_OUT at t=0 should be 0, got {value_0}")
  
  # At t=250ms (25% through), should be around 43.75 (1-(1-25%)^2 of 100)
  var value_25 = provider.get_value(provider.origin + 250)
  assert(value_25 >= 42 && value_25 <= 46, f"EASE_OUT at 25% should be ~44, got {value_25}")
  
  # At t=500ms (50% through), should be around 75 (1-(1-50%)^2 of 100)
  var value_50 = provider.get_value(provider.origin + 500)
  assert(value_50 >= 73 && value_50 <= 77, f"EASE_OUT at 50% should be ~75, got {value_50}")
  
  # At t=750ms (75% through), should be around 93.75 (1-(1-75%)^2 of 100)
  var value_75 = provider.get_value(provider.origin + 750)
  assert(value_75 >= 92 && value_75 <= 96, f"EASE_OUT at 75% should be ~94, got {value_75}")
  
  # At t=1000ms (100% through), should be at end value (100)
  var value_100 = provider.get_value(provider.origin + 999)  # Just before wrap
  assert(value_100 >= 98 && value_100 <= 100, f"EASE_OUT at 100% should be ~100, got {value_100}")
  
  # Verify the curve is decelerating (derivative decreasing)
  assert(value_25 - value_0 > value_50 - value_25, "EASE_OUT should decelerate")
  assert(value_50 - value_25 > value_75 - value_50, "EASE_OUT should continue decelerating")
  
  print("✓ EASE_OUT waveform test passed")
end

# Test the convenience constructor functions
def test_ease_constructors()
  print("Testing ease constructor functions...")
  
  # Test ease_in constructor
  var ease_in_provider = animation.ease_in(10, 90, 2000)
  assert(ease_in_provider.a == 10, "ease_in should set correct start value")
  assert(ease_in_provider.b == 90, "ease_in should set correct end value")
  assert(ease_in_provider.duration_ms == 2000, "ease_in should set correct duration")
  assert(ease_in_provider.form == animation.EASE_IN, "ease_in should set EASE_IN form")
  
  # Test ease_out constructor
  var ease_out_provider = animation.ease_out(20, 80, 1500)
  assert(ease_out_provider.a == 20, "ease_out should set correct start value")
  assert(ease_out_provider.b == 80, "ease_out should set correct end value")
  assert(ease_out_provider.duration_ms == 1500, "ease_out should set correct duration")
  assert(ease_out_provider.form == animation.EASE_OUT, "ease_out should set EASE_OUT form")
  
  print("✓ Ease constructor functions test passed")
end

# Test that easing works with different value ranges
def test_ease_value_ranges()
  print("Testing ease with different value ranges...")
  
  # Test with negative values
  var neg_ease_in = animation.ease_in(-50, 50, 1000)
  var neg_value_0 = neg_ease_in.get_value(neg_ease_in.origin)
  var neg_value_50 = neg_ease_in.get_value(neg_ease_in.origin + 500)
  var neg_value_100 = neg_ease_in.get_value(neg_ease_in.origin + 999)
  
  assert(neg_value_0 == -50, "Negative range should start at -50")
  assert(neg_value_50 >= -28 && neg_value_50 <= -22, "Negative range mid-point should be ~-25")
  assert(neg_value_100 >= 48 && neg_value_100 <= 50, "Negative range should end at ~50")
  
  # Test with small ranges
  var small_ease_out = animation.ease_out(100, 110, 1000)
  var small_value_0 = small_ease_out.get_value(small_ease_out.origin)
  var small_value_50 = small_ease_out.get_value(small_ease_out.origin + 500)
  var small_value_100 = small_ease_out.get_value(small_ease_out.origin + 999)
  
  assert(small_value_0 == 100, "Small range should start at 100")
  assert(small_value_50 >= 107 && small_value_50 <= 108, "Small range mid-point should be ~107.5")
  assert(small_value_100 >= 109 && small_value_100 <= 110, "Small range should end at ~110")
  
  print("✓ Ease value ranges test passed")
end

# Test that easing works with phase shifts
def test_ease_with_phase()
  print("Testing ease with phase shifts...")
  
  var provider = animation.ease_in(0, 100, 1000, animation.EASE_IN)
  provider.set_phase(25)  # 25% phase shift
  
  # With 25% phase shift, the curve should be shifted forward
  var value_0 = provider.get_value(provider.origin)
  var value_25 = provider.get_value(provider.origin + 250)
  
  # At t=0 with 25% phase, we should see the value that would normally be at t=250ms
  assert(value_0 >= 5 && value_0 <= 8, f"Phase-shifted EASE_IN at t=0 should be ~6, got {value_0}")
  
  print("✓ Ease with phase test passed")
end

# Test string representation includes new waveforms
def test_ease_tostring()
  print("Testing ease tostring representation...")
  
  var ease_in_provider = animation.ease_in(0, 255, 3000)
  var ease_out_provider = animation.ease_out(10, 200, 2500)
  
  var ease_in_str = ease_in_provider.tostring()
  var ease_out_str = ease_out_provider.tostring()
  
  assert(string.find(ease_in_str, "EASE_IN") >= 0, "EASE_IN tostring should contain 'EASE_IN'")
  assert(string.find(ease_out_str, "EASE_OUT") >= 0, "EASE_OUT tostring should contain 'EASE_OUT'")
  
  print("✓ Ease tostring test passed")
end

# Test that constants are properly exported
def test_ease_constants()
  print("Testing ease constants export...")
  
  assert(animation.EASE_IN == 5, "EASE_IN constant should be 5")
  assert(animation.EASE_OUT == 6, "EASE_OUT constant should be 6")
  
  # Test that constants work with direct constructor
  var direct_ease_in = animation.oscillator_value_provider(0, 100, 1000, animation.EASE_IN)
  var direct_ease_out = animation.oscillator_value_provider(0, 100, 1000, animation.EASE_OUT)
  
  assert(direct_ease_in.form == animation.EASE_IN, "Direct EASE_IN should work")
  assert(direct_ease_out.form == animation.EASE_OUT, "Direct EASE_OUT should work")
  
  print("✓ Ease constants test passed")
end

# Run all tests
def run_oscillator_ease_tests()
  print("=== OscillatorValueProvider Ease Tests ===")
  
  try
    test_ease_in_waveform()
    test_ease_out_waveform()
    test_ease_constructors()
    test_ease_value_ranges()
    test_ease_with_phase()
    test_ease_tostring()
    test_ease_constants()
    
    print("=== All OscillatorValueProvider ease tests passed! ===")
    return true
  except .. as e, msg
    print(f"Test failed: {e} - {msg}")
    raise "test_failed"
  end
end

# Export the test function
animation.run_oscillator_ease_tests = run_oscillator_ease_tests

run_oscillator_ease_tests()

return run_oscillator_ease_tests