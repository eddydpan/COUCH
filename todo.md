# To-Do List
As of 11/14 we have open-loop control

## Development Roadmap

### Phase 1: Bench Testing & PID Tuning (Current Phase)
**Status:** Motor controller code written, not yet integrated
- [x] Implement PID controller with hall sensor feedback
- [x] Create bench test mode functions (Triangle/Circle buttons)
- [ ] Measure actual pole pairs from motors
- [ ] Wire up hall sensors to GPIO pins (32-35, 14, 27)
- [ ] Run bench test to verify both motors track same speed
- [ ] Tune PID gains (kp, ki, kd) using test mode
- [ ] Verify < 5% speed difference between motors

### Phase 2: Integration & Tank Drive Testing
**After PID tuning on bench**
- [ ] Integrate motor_controller into my_platform.c
  - Call motor_controller_init() in my_platform_init()
  - Replace direct DAC writes with motor_controller_update_input()
  - Wire X button to motor_controller_emergency_stop()
- [ ] Test tank drive (differential drive)
  - Left joystick Y = forward/backward speed
  - Right joystick X = left/right turning
- [ ] Verify straight-line driving without drift
- [ ] Test turning radius and responsiveness

### Phase 3: Car-Style Steering Conversion
**Convert from tank drive to car steering**
- [ ] Design new control scheme:
  - Left joystick Y = throttle (forward/backward)
  - Right joystick X = steering angle
  - Algorithm: Convert steering angle to differential wheel speeds
- [ ] Implement steering geometry:
  - Inner wheel slower than outer wheel on turns
  - Calculate turn radius based on wheelbase
  - Smooth steering response curve
- [ ] Test and tune steering feel
- [ ] Add steering limits to prevent tip-over

### Phase 4: Safety & Advanced Features
- [ ] Implement flyback/regenerative braking circuit
  - "Crowbar" circuit for energy dissipation
  - Protect ESCs from back-EMF during braking
- [ ] Add "coasting mode" for downhill
  - Detect downhill via hall sensors (increasing speed with no throttle)
  - Limit speed to safe threshold
  - Auto-engage braking if exceeding limit
- [ ] Optocoupler isolation for 48V system (PC817/TLP250)
- [ ] Battery voltage monitoring
- [ ] Low-battery warning/cutoff

### Phase 5: Polish & Optimization
- [ ] Rename my_platform.c to controller_interface.c
- [ ] Delete unused main.c file
- [ ] Add telemetry/logging over WiFi or UART
- [ ] Save PID tuning to NVS (non-volatile storage)
- [ ] Implement different drive modes (sport/eco/safety)

---

## Bench Testing Setup (Roadkill Configuration)

### Hardware Configuration
**GPIO Pin Assignments (ESP32-WROOM):**
- **Left Motor Hall Sensors:** GPIO32 (A), GPIO33 (B), GPIO34 (C)
- **Right Motor Hall Sensors:** GPIO35 (A), GPIO14 (B), GPIO27 (C)
- **DAC Throttle Outputs:** GPIO25 (Left), GPIO26 (Right)

**Motor Specs:**
- Max RPM: 4500
- Pole pairs: Unknown (needs measurement - see below)

### Test Controls (PS4 Controller)
- **Triangle (△)**: Run 10-second speed matching test at 1000 RPM
  - Logs speed comparison every 500ms
  - Shows RPM difference between motors
  - Verifies both motors track at same speed
  
- **Circle (○)**: Print full motor diagnostics
  - Current RPM, target RPM, error percentage
  - Throttle voltage
  - Hall sensor counts and state
  - PID gains and integral term
  
- **X (Square)**: Emergency stop (when fully integrated)

### Testing Procedure
1. **Initial Setup:**
   - Place both motors in "roadkill" bench configuration
   - Ensure hall sensors are connected to correct GPIO pins
   - Power up ESP32 and motors separately (48V isolation!)

2. **Measure Pole Pairs (IMPORTANT):**
   ```
   a. Press Circle (○) to see current hall count
   b. Manually spin one wheel EXACTLY 1 full rotation
   c. Press Circle (○) again to see new count
   d. Calculate: pole_pairs = (new_count - old_count) / 6
   e. Update DEFAULT_POLE_PAIRS in motor_controller.c
   ```
   **Why it matters:** Wrong pole pairs = wrong RPM reading = bad PID control

3. **Speed Matching Test:**
   - Press Triangle (△) to start test
   - Motors will run at 1000 RPM for 10 seconds
   - Watch log output for speed differences
   
4. **Expected Results:**
   - **Good:** Speed difference < 5% → Motors are well matched
   - **Acceptable:** 5-10% difference → May need per-motor PID tuning
   - **Bad:** > 10% difference → Check wiring, mechanical issues, or different motor characteristics

5. **Example Log Output:**
   ```
   [0.5s] L: 985 RPM, R: 1010 RPM | Diff: 25 RPM (2.5%) | L_V: 0.73V, R_V: 0.75V
   ✓ Motors tracking well (< 5% error)
   ```

### PID Tuning Process (After Successful Bench Test)
**Phase 1: Find Kp (Proportional)**
- Set `ki = 0, kd = 0, kp = 0.1`
- Push joystick forward slowly
- If sluggish → increase kp by 0.1
- If oscillates → decrease kp by 0.05
- Goal: Quick response without oscillation

**Phase 2: Add Ki (Integral)**
- Keep tuned kp, set `ki = 0.01`
- Test straight-line driving
- If drifts over time → increase ki
- If hunts/oscillates → decrease ki
- Goal: Zero steady-state error

**Phase 3: Add Kd (Derivative)** (optional)
- Set `kd = 0.01`
- Test sudden joystick changes
- Usually not needed for motor control

**Integration test:** Push both joysticks forward equally. If couch drives straight, PID is working correctly.

### Common Issues
- **Large speed difference:** One motor has more resistance, needs individual PID tuning
- **Hall count not changing:** Check wiring, ensure 3.3V logic level
- **Motors don't respond:** Verify DAC output with multimeter (0-3.3V range)
- **Erratic RPM readings:** Wrong pole pairs, or hall sensor noise

---
