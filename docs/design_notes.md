> [!NOTE]
> All information in this document are prototype based ideas and must be taken with caution...

<details>
<summary>:frog: Algae Handler Function</summary>
  
  \
  Vertical Position -> in robot perimeter
  \
  Horizontal Position -> out of robot perimeter

### Starting Position
  - No Motor Movement: No Spin
  - Solenoid: Vertical Position

### Intake Position
  - Motor Movement: Spin
  - Solenoid: Horizontal Position

### Has Algae
  - Hit Limit Switch
    - Stop Motor
    - No Motor Movement: No Spin
    - Solenoid: Vertical Position
  - Hall Effect Sensor: When magnet gets detected

### Holding Position
  - No Motor Movement: No Spin
  - Solenoid: Vertical Position
  - Hold
    - Limit and Hall Effect both switched: Stop Running
    - Hall Effect w/o Limit: Run Motor until Limit pressed

### Outtake Position
  - Motor Movement: Neg. Spin -> Timed?
  - Solenoid: Vertical Position
  - End Hold
    
</details>
<details>
<summary> :shell: Coral Handler Function </summary>

  >👓 Use encoders for accurate LR and Vertical Positioning?
  >\
  >🛑 Possible Limit Switches for protection, not guaranteed
  
  Intake Position -> Down Position
  \
  Outtake Lvl 1 Position -> 45 Angle Outtake (+ error)

### Starting Position
  - Outtake Motor: No Spin
    - Coral Limit Switch: not hit
  - LR Motor: Right
  - Vertical Motor: Intake Position
  - Elevator Position: Start


### Intake Position
  - Outtake Motor: Neg. Spin until Coral Limit Switch hit
    - Coral Limit Switch: hit
  - LR Motor: Right
  - Vertical Motor: Intake Position
  - Elevator Position: Start


## :fishing_pole_and_fish: Reef Positioning States

### Left Trough Position
  - Outtake Motor: Spin
    - Coral Limit Switch: not hit
  - LR Motor: Base off of Odom
  - Vertical Motor: Outtake Level 1 Position
  - Elevator Position: Start

### Right Trough Position
  - Outtake Motor: Spin
    - Coral Limit Switch: not hit
  - LR Motor: Base off of Odom
  - Vertical Motor: Outtake Level 1 Position
  - Elevator Position: Start

### Reef Level 2 Position
  - Outtake Motor: Spin
    - Coral Limit Switch: not hit
  - LR Motor: Base off of Odom
  - Vertical Motor: Outtake Level 2 Position
  - Elevator Position: Level 2
    
### Reef Level 3 Position
  - Outtake Motor: Spin
    - Coral Limit Switch: not hit
  - LR Motor: Base off of Odom
  - Vertical Motor: Outtake Level 3 Position
  - Elevator Position: Level 3

### Reef Level 4 Position
  - Outtake Motor: Spin
    - Coral Limit Switch: not hit
  - LR Motor: Base off of Odom
  - Vertical Motor: Outtake Level 4 Position
  - Elevator Position: Level 4

  > ⚠️
  > Do not know about a possible Ending Position.

- Possible Ending Position?
</details>
<details>
<summary> :diving_mask: Elevator Function </summary>

### Start (& Intake + Trough) Position
  - Ele. Motor: Rev. Spin until bottom limit swtich hit
  - Bottom Limit Switch: hit
  - Top Limit Switch: not hit
  
### Level 2 Position
  - Ele. Motor: Spin until rotation/height reached (**calculate**)
    - use encoder for rotation measurment 
  - Bottom Limit Switch: not hit
  - Top Limit Switch: not hit
  
### Level 3 Position
  - Ele. Motor: Spin until rotation/height reached (**calculate**)
    - use encoder for rotation measurment 
  - Bottom Limit Switch: not hit
  - Top Limit Switch: not hit
  
### Level 4 Position  
  - Ele. Motor: Spin until rotation/height reached (**calculate**)
      - use encoder for rotation measurment
  - Bottom Limit Switch: not hit
  - Top Limit Switch: hit
</details>

<details>
<summary> :crab: Climber Function </summary>

  ##### Posibility of using M Solenoid for sensor?

### Start Position
  - Up/Down Motor: Up Position, No Spin
  - Clamp Solenoid: Open Position
    
### Ready to Climb Position:
  -  Up/Down Motor: Down Position, Spin
  -  Clamp Solenoid: Open Position

### Clamp Position:
  - Up/Down Motor: Down Position, No Spin
  - Clamp Solenoid: Closed Position
    
### Climbing Position:
  - Up/Down Motor: Up Position, Spin
  - Clamp Solenoid: Closed Position
    
</details>

<details>
<summary> :droplet: LEDs Function </summary>
  
    Turned On & Connected: Rainbow
    Coral Intaked: Green
    Algae Intaked: Orange
    Right Side Intake Running: Red
    Left Side Intake Running: Purple
    Robot Balanced?: Blue
</details>
