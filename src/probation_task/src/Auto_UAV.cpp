#include <memory>
#include <functional>
#include <string>
#include <chrono>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/bounding_box_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <mavros_msgs/srv/set_mode.hpp>

enum class UAVState { SEARCH, ALIGN, PASS_THROUGH };

class Auto_UAV : public rclcpp::Node
{
public:
    Auto_UAV() : Node("Auto_UAV"), state_(UAVState::SEARCH)
    {
        // Subscriptions
        Cam_Sub = this->create_subscription<vision_msgs::msg::BoundingBoxArray>(
            "/main_camera/detection/bounding_boxes", 10,
            std::bind(&Auto_UAV::CamCallback, this, std::placeholders::_1));

        Rel_Alt_Sub = this->create_subscription<std_msgs::msg::Float64>(
            "/mavros/global_position/rel_alt", 10,
            std::bind(&Auto_UAV::RelAltCallback, this, std::placeholders::_1));

        Com_Hdg_Sub = this->create_subscription<std_msgs::msg::Float64>(
            "/mavros/global_position/compass_hdg", 10,
            std::bind(&Auto_UAV::ComHdgCallback, this, std::placeholders::_1));

        // Publishers
        coord_x_Pub = this->create_publisher<std_msgs::msg::Float32>("/mavros/setpoint_velocity/cmd_vel_unstamped/x", 10);
        coord_y_Pub = this->create_publisher<std_msgs::msg::Float32>("/mavros/setpoint_velocity/cmd_vel_unstamped/y", 10);
        coord_z_Pub = this->create_publisher<std_msgs::msg::Float32>("/mavros/setpoint_velocity/cmd_vel_unstamped/z", 10);
        coord_r_Pub = this->create_publisher<std_msgs::msg::Float32>("/mavros/setpoint_velocity/cmd_vel_unstamped/r", 10);

        // Timers
        control_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&Auto_UAV::control_loop, this));

        Set_Mode("GUIDED");
    }

private:
    UAVState state_;
    rclcpp::Time start_time_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    std_msgs::msg::Float32 gate_x, gate_y, error_x, error_y, red_flare_h;
    std_msgs::msg::Bool gate_detected, red_flare_detected;
    std_msgs::msg::Float64 rel_alt, com_hdg;

    rclcpp::Subscription<vision_msgs::msg::BoundingBoxArray>::SharedPtr Cam_Sub;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr Rel_Alt_Sub, Com_Hdg_Sub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr coord_x_Pub, coord_y_Pub, coord_z_Pub, coord_r_Pub;

    const float center_threshold = 0.05;

    //main loop
    void control_loop()
    {
        std_msgs::msg::Float32 x_vel, y_vel, z_vel, r_vel;
        x_vel.data = y_vel.data = z_vel.data = r_vel.data = 0.0;

        //3 diff stages - search,align,pass through
        switch (state_)
        {
            case UAVState::SEARCH:
            {
                //make sure the vehicle descend to below -1.3m
                if(rel_alt.data > -1.4)
                {
                    z_vel.data = -0.5;
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
            
            case UAVState::ALIGN:
            {
                error_x.data = gate_x.data - 0.50;
                error_y.data = gate_y.data - 0.50;
                
                float heading = com_hdg.data;  

                bool in_range = (heading >= 175.0 && heading <= 180.0) || (heading >= 355.0 && heading <= 358.0);

                if (in_range)
                {
                    r_vel.data = 0.0;  // stop rotating
                    y_vel.data = -error_x.data * 1.0;  // lateral
                    z_vel.data = -error_y.data * 1.0;  // vertical
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

                if(((fabs(error_x.data) > center_threshold && fabs(error_x.data) <= 0.2) 
                    || (fabs(error_y.data) > center_threshold && fabs(error_y.data) <= 0.2)) && in_range == true)
                {
                    x_vel.data = 0.1;
                }

                if(red_flare_detected.data == true)
                {
                    if(red_flare_h.data > 0.5)
                    {
                        y_vel.data = -0.3;
                    }
                }

                if (fabs(error_x.data) < center_threshold && fabs(error_y.data) < center_threshold 
                    && ((com_hdg.data >= 175.0 && com_hdg.data <= 180.0) || (com_hdg.data >= 355.0 && com_hdg.data <= 358.0)))
                {
                    state_ = UAVState::PASS_THROUGH;
                    start_time_ = this->now();
                    RCLCPP_INFO(this->get_logger(), "Aligned → PASS_THROUGH");
                }
                break;
            }

            case UAVState::PASS_THROUGH:
            {
                if ((this->now() - start_time_).seconds() < 30.0)
                {
                    x_vel.data = 0.5;  // move forward
                    z_vel.data = 0.0;
                    if(red_flare_detected.data == true)
                    {
                        if(red_flare_h.data > 0.60)
                        {
                            y_vel.data = -0.5;
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
        }
            // Publish commands
            coord_x_Pub->publish(x_vel);
            coord_y_Pub->publish(y_vel);
            coord_z_Pub->publish(z_vel);
            coord_r_Pub->publish(r_vel);
        
    }

        // ===== Callbacks =====
        void CamCallback(const vision_msgs::msg::BoundingBoxArray::SharedPtr msg)
        {
            gate_detected.data = false;
            red_flare_detected.data = false;
            for (const auto &box : msg->bounding_boxes)
            {
                if (box.label_name == "gate")
                {
                    gate_detected.data = true;
                    gate_x.data = box.x;
                    gate_y.data = box.y;
                    break;
                }
                if(box.label_name == "red_flare")
                {
                    red_flare_detected.data = true;
                    red_flare_h.data = box.h;
                    break;
                }
            }
        }

        void RelAltCallback(const std_msgs::msg::Float64::SharedPtr msg)
        {
            rel_alt.data = msg->data;
        }

        void ComHdgCallback(const std_msgs::msg::Float64::SharedPtr msg)
        {
            com_hdg.data = msg->data;
        }

        // ===== Set Mode =====
        void Set_Mode(const std::string &mode)
        {
            auto client = this->create_client<mavros_msgs::srv::SetMode>("mavros/set_mode");
            auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
            request->custom_mode = mode;

            while (!client->wait_for_service(std::chrono::seconds(1)))
            {
                if (!rclcpp::ok()) return;
                RCLCPP_INFO(this->get_logger(), "Waiting for set_mode service...");
            }

            auto result = client->async_send_request(request);
            if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
            {
                RCLCPP_INFO(this->get_logger(), "Mode changed to %s", mode.c_str());
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Failed to call set_mode");
            }
        }
    };

int main(int argc, char * argv[])
{
    
    rclcpp::init(argc, argv);
    RCLCPP_INFO(rclcpp::get_logger("Auto_UAV"), "Starting Auto_UAV node");
    rclcpp::spin(std::make_shared<Auto_UAV>());
    rclcpp::shutdown();
    return 0;
}
