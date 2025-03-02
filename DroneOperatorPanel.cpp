/*
 * DroneOperatorPanel.cpp
 *
 * Defines a custom RViz panel plugin for controlling drone missions.
 */

#include "DroneOperatorPanel.hpp"
#include <rviz/panel.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>  // Include service message type
#include <QPushButton>
#include <QLineEdit>
#include <QVBoxLayout>
#include <QLabel>
#include <QImage>
#include <QPixmap>
#include <pluginlib/class_list_macros.h>

namespace drone_operator_ui
{
    DroneOperatorPanel::DroneOperatorPanel(QWidget* parent)
        : rviz::Panel(parent), nh_(), it_(nh_), camera_on_(false)
    {
        // UI components
        QVBoxLayout* layout = new QVBoxLayout;

        // Initialize UI elements
        missionID_input_ = new QLineEdit;
        toggle_camera_button_ = new QPushButton("Enable Camera");
        start_button_ = new QPushButton("Start Mission");
        camera_display_ = new QLabel("Camera feed will appear here.");
        camera_display_->setAlignment(Qt::AlignCenter);
        camera_display_->setFixedSize(640, 480); // Set size for static image

        // Add components to layout
        layout->addWidget(new QLabel("Mission ID:"));
        layout->addWidget(missionID_input_);
        layout->addWidget(start_button_);
        layout->addWidget(toggle_camera_button_);
        layout->addWidget(camera_display_);

        // Connect signals to slots
        connect(start_button_, SIGNAL(clicked()), this, SLOT(onButtonClicked()));
        connect(toggle_camera_button_, SIGNAL(clicked()), this, SLOT(onToggleCamera()));

        // Initialize ROS service client
        camera_toggle_client_ = nh_.serviceClient<std_srvs::SetBool>("/toggle_camera");

        // Set layout
        setLayout(layout);
    }

    void DroneOperatorPanel::onButtonClicked()
    {
        if (!missionID_input_)
        {
            ROS_ERROR("Mission ID input box is not initialized!");
            return;
        }

        ROS_INFO("Mission started: %s", missionID_input_->text().toStdString().c_str());
    }

    void DroneOperatorPanel::onToggleCamera()
    {
        std_srvs::SetBool srv;
        srv.request.data = !camera_on_;  // Toggle boolean state

        if (camera_toggle_client_.call(srv))
        {
            camera_on_ = srv.response.success; // Update based on response
            if (camera_on_)
            {
                camera_display_->setText("Static image displayed.");
                toggle_camera_button_->setText("Disable Camera");

                // Load and display static image
                QImage image("/home/bhatthit/sae_vtol/src/drone_operator_ui/src/minerva.jpg");
                camera_display_->setPixmap(QPixmap::fromImage(image));
            }
            else
            {
                camera_display_->setText("Camera feed disabled.");
                toggle_camera_button_->setText("Enable Camera");
            }
        }
        else
        {
            ROS_ERROR("Failed to call service /toggle_camera");
        }
    }

    void DroneOperatorPanel::save(rviz::Config config) const
    {
        rviz::Panel::save(config);
    }

    void DroneOperatorPanel::load(const rviz::Config& config)
    {
        rviz::Panel::load(config);
    }
}  // namespace drone_operator_ui

// Register the plugin
PLUGINLIB_EXPORT_CLASS(drone_operator_ui::DroneOperatorPanel, rviz::Panel)
