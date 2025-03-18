/********************************************************************************
 * Copyright (C) 2017-2020 German Aerospace Center (DLR).
 * Eclipse ADORe, Automated Driving Open Research https://eclipse.org/adore
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0.
 *
 * SPDX-License-Identifier: EPL-2.0
 *
 * Contributors:
 *    Giovanni Lucente
 ********************************************************************************/

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "adore_dynamics_conversions.hpp"
#include "adore_map/map.hpp"
#include "adore_map/map_loader.hpp"
#include "adore_map/route.hpp"
#include "adore_map/traffic_light.hpp"
#include "adore_map_conversions.hpp"
#include "adore_math/angles.h"
#include "adore_math/distance.h"
#include "adore_math/polygon.h"
#include "adore_ros2_msgs/msg/map.hpp"
#include "adore_ros2_msgs/msg/route.hpp"
#include "adore_ros2_msgs/msg/traffic_participant_set.hpp"
#include "adore_ros2_msgs/msg/visualizable_object.hpp"

#include "bob_perception_msgs/msg/tracked_oriented_box_v2x_array.hpp"
#include "planning/multi_agent_PID.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

namespace adore
{

class DecisionMakerInfrastructure : public rclcpp::Node
{
private:

  /******************************* PUBLISHERS RELATED MEMBERS ************************************************************/
  rclcpp::TimerBase::SharedPtr                                              main_timer;
  rclcpp::Publisher<adore_ros2_msgs::msg::TrafficParticipantSet>::SharedPtr publisher_planned_traffic;
  rclcpp::Publisher<adore_ros2_msgs::msg::Map>::SharedPtr                   publisher_local_map;
  rclcpp::Publisher<adore_ros2_msgs::msg::VisualizableObject>::SharedPtr    publisher_infrastructure_position;

  /******************************* SUBSCRIBERS RELATED MEMBERS ************************************************************/
  using StateSubscriber = rclcpp::Subscription<adore_ros2_msgs::msg::TrafficParticipant>::SharedPtr;
  std::unordered_map<std::string, StateSubscriber> traffic_participant_subscribers;

  /******************************* OTHER MEMBERS *************************************************************************/
  std::optional<adore::map::Map>         road_map = std::nullopt;
  adore::dynamics::TrafficParticipantSet latest_traffic_participant_set;

public:

  bool                                  debug_mode_active = true;
  double                                dt                = 0.1;
  double                                local_map_size    = 50;
  adore::math::Pose2d                   infrastructure_pose;
  adore::dynamics::VehicleCommandLimits command_limits = { 0.7, -2.0, 2.0 };
  std::map<std::string, double>         multi_agent_PID_settings;
  adore::planner::MultiAgentPID         multi_agent_PID_planner;

  std::string map_file_location;

  void run();
  void update_state();
  void create_subscribers();
  void create_publishers();
  void load_parameters();
  void print_init_info();
  void print_debug_info();
  void compute_routes_for_traffic_participant_set( adore::dynamics::TrafficParticipantSet& traffic_participant_set,
                                                   const adore::map::Map&                  road_map );
  void all_vehicles_follow_routes();
  void update_dynamic_subscriptions();

  /******************************* PUBLISHER RELATED FUNCTIONS ************************************************************/
  void publish_local_map();
  void publish_infrastructure_position();

  /******************************* SUBSCRIBER RELATED FUNCTIONS************************************************************/

  void traffic_participant_callback( const adore_ros2_msgs::msg::TrafficParticipant& msg );

  DecisionMakerInfrastructure();
};

} // namespace adore
