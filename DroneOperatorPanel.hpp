/*
 * DroneOperatorPanel.hpp
 *
 * Header file declaring the DroneOperatorPanel class, a custom
 * RViz panel plugin for ROS. This class provides an operator
 * interface with UI elements such as a button, text box, and camera feed toggle button.
 * 
 * The plugin can be loaded in RViz to allow users to input
 * mission details, control drone operations, and toggle the camera feed.
 *
 * The file uses the Q_OBJECT macro for Qt's meta-object
 * system and contains slots and signals for Qt UI interactions.
 */

#ifndef DRONE_OPERATOR_PANEL_HPP
#define DRONE_OPERATOR_PANEL_HPP

#include "DroneOperatorPanel.hpp"
#include <rviz/panel.h>
#include <ros/ros.h>
#include <QPushButton>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QLabel>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <pluginlib/class_list_macros.h>
#include <QPixmap>
#include <QImage>

namespace drone_operator_ui
{
    class DroneOperatorPanel : public rviz::Panel
    {
        Q_OBJECT

    public:
        // Constructor
        DroneOperatorPanel(QWidget* parent = nullptr);
        
        // Destructor
        virtual ~DroneOperatorPanel() = default;

        // Override save and load methods to preserve settings
        virtual void save(rviz::Config config) const override;
        virtual void load(const rviz::Config& config) override;

    private Q_SLOTS:
        // Slot function called when the Start Mission button is clicked
        void onButtonClicked();
        
        // Slot function for receiving and processing image messages
        void onImageReceived(const sensor_msgs::ImageConstPtr& msg);

        // Slot function to toggle the camera feed
        void onToggleCamera();

    private:
        QLineEdit* missionID_input_;     // Mission ID input field
        QPushButton* start_button_;      // Start Mission button
        QPushButton* toggle_camera_button_;  // Toggle Camera button
        QLabel* camera_display_;         // Widget to display camera feed

        ros::NodeHandle nh_;             // ROS node handle
        image_transport::ImageTransport it_;
        image_transport::Subscriber image_sub_; // Image subscriber

        QString image_topic_;            // Current camera topic name
        bool camera_on_;                 // Camera feed status (on/off)

        ros::ServiceClient camera_toggle_client_;  // **Declare the service client**
    };

} // namespace drone_operator_ui

#endif // DRONE_OPERATOR_PANEL_HPP
