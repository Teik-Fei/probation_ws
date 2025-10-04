# Gate Detection and Navigation

## 🎥 Demo
▶️ Demo Video: [here](https://drive.google.com/file/d/1-V-Eluy8XOdH0yUg0chQJ3mrNMu2OaWk/view?usp=sharing)

---

## 📌 Project Logic Flow
- ✅ Switches vehicle mode to `GUIDED` on startup  
- ⬇️ Descends to target depth for better gate visibility  
- 🔄 Performs a search pattern to locate the gate  
- 🎯 Aligns with the gate using bounding box detection  
- 🚀 Moves forward to pass through the gate  
- 🔁 Repeats cycle for multiple gates  

## 💡 Thought Process
- Vehicle need to change from `ALT_HOLD` to `GUIDED` which mean the node have to send a request to `mavros/set_mode`. (part 1)
- Vehicle need to descend to a certain depth to scan for gate, after playing around with the simulator I found out -1.4m to -1.6m is a suitable depth for the vehicle to pass through the gate. (part 2)
- To search for the gate, the vehicle have to rotate for it detect the gate. Upon detecting the gate, vehicle have to align to the center of the gate. I didn't use the raw coordinates from the bounding box as I realised the vehicle will always recenter and hit the side of the gate as the vehicle move closer to the gate due to new coordinates received from detection. So I make use of the compass heading to make sure the vehicle is facing the gate while it is centered and give the command for the vehicle to move forward ignoring the new coordinates of the gate. After playing with the simulator I found out the compass heading between `175° to 180°` and `355° to 358°` is the suitable angle as the vehicle is facing directly at the gate. (part 3)
- The condition for the vehicle to move forward and pass through the gate is when the compass heading is within range and the vehicle is in the center of the gate. When this condition is met, the timer will start counting till 30 seconds while publishing data to `/mavros/setpoint_velocity/cmd_vel_unstamped/x`. (part 4)
- After passing through the gate the process need to repeat itself so I have to create 3 stages for the vehicle to search for the gate again after passing through it. The 3 different stages I need to have are `SEARCH `, `ALIGN` and  `PASS_THROUGH `. (part 5)

---

## 🛠️ Topics

### 📥 Subscribed
- `/main_camera/detection/bounding_boxes` → Gate and red flare detection (vision_msgs/BoundingBoxArray).
- `/mavros/global_position/rel_alt` → Relative altitude (std_msgs/Float64).
- `/mavros/global_position/compass_hdg` → Compass heading (std_msgs/Float64).

### 📤 Published
- `/mavros/setpoint_velocity/cmd_vel_unstamped/x` → Forward velocity (std_msgs/Float32).
- `/mavros/setpoint_velocity/cmd_vel_unstamped/y` → Lateral velocity (std_msgs/Float32).
- `/mavros/setpoint_velocity/cmd_vel_unstamped/z` → Vertical velocity (std_msgs/Float32).
- `/mavros/setpoint_velocity/cmd_vel_unstamped/r` → Rotational velocity (std_msgs/Float32).

---

## 🚦 State Machine

1. 🔍 **SEARCH**  
   - Descend until target depth. (around -1.4m to -1.5m)  
   - Rotate until gate is detected.  

    <details> <summary>SEARCH Code (click to expand)</summary>

    ```cpp
    case UAVState::SEARCH:
    {
        //make sure the vehicle descend to below -1.4m
        if(rel_alt.data > -1.4)
        {
            z_vel.data = -0.5; //-ve to descend
        }
        else
        {
            if (gate_detected.data)
            {
                state_ = UAVState::ALIGN;
                RCLCPP_INFO(this->get_logger(), "Gate detected → ALIGN");
            }
            else
            {
                r_vel.data = 0.5;  // rotate until gate appears
            }
        }
        break;
    }
    ```
    </details>
  
2. 🎯 **ALIGN**  
   - Adjust UAV position laterally and vertically based on gate bounding box.  
   - Correct heading to face the gate. (around 175° to 180° and 355° to 358°)  
   - Transition to PASS_THROUGH when aligned. 

    <details> <summary>ALIGN Code (click to expand)</summary>

    ```cpp
    case UAVState::ALIGN:
    {
        error_x.data = gate_x.data - 0.50;
        error_y.data = gate_y.data - 0.50;
                
        float heading = com_hdg.data;  

        bool in_range = (heading >= 175.0 && heading <= 180.0) || (heading >= 355.0 && heading <= 358.0);

        if (in_range)
        {
            r_vel.data = 0.0;  // stop rotating
            y_vel.data = -error_x.data * 1.0;  // lateral correction
            z_vel.data = -error_y.data * 1.0;  // vertical correction
        }
        else if (heading < 175.0) 
        {
            if(heading > 0.0 && heading <= 90.0)
            {
                r_vel.data = 0.2; //rotate left
            }
            else
            {
                r_vel.data = -0.2; // rotate right
            }
        }
        else if (heading > 180.0 && heading < 355.0) 
        {
            if(heading > 270 && heading <= 355.0)
            {
                r_vel.data = -0.2; //rotate right
            }
            else
            {
                r_vel.data = 0.2;  // rotate left
            }
        }

        RCLCPP_INFO(this->get_logger(), "Error X: %.2f | Error Y: %.2f", error_x.data, error_y.data);

        if(((fabs(error_x.data) > center_threshold && fabs  (error_x.data) <= 0.2) 
        || (fabs(error_y.data) > center_threshold && fabs(error_y.data) <= 0.2)) && in_range == true)
        {
            x_vel.data = 0.1; //slow movement towards the gate to help with alignment
        }

        if(red_flare_detected.data == true)
        {
            if(red_flare_h.data > 0.5)
            {
                y_vel.data = -0.3; //move right if red flare is detected
            }
        }

        if (fabs(error_x.data) < center_threshold && fabs(error_y.data) < center_threshold 
        && ((com_hdg.data >= 175.0 && com_hdg.data <= 180.0) || (com_hdg.data >= 355.0 && com_hdg.data <= 358.0)))
        {
            state_ = UAVState::PASS_THROUGH; //vehicle will only pass through when it is centered and the heading is correct
            start_time_ = this->now(); //start of timer
            RCLCPP_INFO(this->get_logger(), "Aligned → PASS_THROUGH");
        }
        break;
    }
    ```
    </details>

3. 🚀 **PASS_THROUGH**  
   - Move forward through the gate for a fixed duration. (0.5m/s for 30 seconds)  
   - Return to SEARCH state after passing through.  

    <details> <summary>PASS_THROUGH Code (click to expand)</summary>

    ```cpp
    case UAVState::PASS_THROUGH:
    {
        //the 'start_time_' is the timer from ALIGN state
        if ((this->now() - start_time_).seconds() < 30.0)
        {
            x_vel.data = 0.5;  // move forward
            z_vel.data = 0.0;
            if(red_flare_detected.data == true)
            {
                if(red_flare_h.data > 0.60)
                {
                    y_vel.data = -0.5; //vehicle will shift right if red flare is detected
                }
            }
        }
        else
        {
            state_ = UAVState::SEARCH;  // go back to searching for next gate
            RCLCPP_INFO(this->get_logger(), "Finished pass → SEARCH");
        }
        break;
    }
    ```
    </details>

---

## ▶️ Usage

**Build and Launch**
```bash
cd ~/probation_ws
colcon build --symlink-install
source install/setup.bash
ros2 run probation_task Auto_UAV
```

## 👤 Author

Created by: TEIK FEI

