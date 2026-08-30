from mimic import MimicBridge
import time

bridge = MimicBridge()
if bridge.connect():
    print("Hello this is Blink LED")
    
    # Initialize pins
    bridge.execute("PIN_SET_OUT C13")
    bridge.execute("PIN_SET_IN B0 UP") # Assuming pull-up for the button
    
    delay = 1.0
    last_toggle_time = time.time()
    last_button_state = True # True = HIGH (not pressed)
    
    print(f"Starting to blink C13 with a delay of {delay:.3f} seconds. Press B0 to speed up!")
    
    # Polling loop to handle blinking and button presses
    while True:
        # Check button state
        response = bridge.execute("PIN_READ B0")
        current_button_state = not any("LOW" in r for r in response)
        
        # Edge detection: button just pressed (went from HIGH to LOW)
        if last_button_state and not current_button_state:
            delay *= 0.95 # Increase speed by 5% (decrease delay by 5%)
            print(f"Button Pressed! Speed increased. New delay: {delay:.3f} seconds")
            
        last_button_state = current_button_state
        
        # Non-blocking blink
        if time.time() - last_toggle_time >= delay:
            bridge.execute("PIN_TOGGLE C13")
            last_toggle_time = time.time()
            
        time.sleep(0.01) # Small delay to avoid flooding the CPU
