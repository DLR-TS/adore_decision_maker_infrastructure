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

#include "decision_maker_infrastructure.hpp"

#include <adore_dynamics_conversions.hpp>
#include <adore_math/point.h>

namespace adore
{

DecisionMakerInfrastructure::DecisionMakerInfrastructure(const rclcpp::NodeOptions & options) : Node( "decision_maker_infrastructure" , options)
{
  load_parameters();
  // Load map
  road_map = map::MapLoader::load_from_file( map_file_location );
  create_subscribers();
  create_publishers();
  print_init_info();
}

void
DecisionMakerInfrastructure::run()
{
  auto start_time = std::chrono::high_resolution_clock::now(); // Start timer

  if( road_map.has_value() )
  {
    compute_routes_for_traffic_participant_set( latest_traffic_participant_set, road_map.value() );
  }
  all_vehicles_follow_routes();
  auto end_time = std::chrono::high_resolution_clock::now(); // End timer

  // Compute elapsed time in milliseconds
  double elapsed_time_ms = std::chrono::duration<double, std::milli>( end_time - start_time ).count();

  // Log the elapsed time
  RCLCPP_INFO( this->get_logger(), "Planning took %.3f ms", elapsed_time_ms );

  if( debug_mode_active )
    print_debug_info();
  publish_local_map();
  publish_infrastructure_position();
}

void
DecisionMakerInfrastructure::all_vehicles_follow_routes()
{
  // auto mrm_participant_set = latest_traffic_participant_set;
  multi_agent_PID_planner.plan_trajectories( latest_traffic_participant_set );
  // multi_agent_PID_planner_MRM.plan_trajectories( mrm_participant_set );
  // TODO add the MRM trajectories to the traffic participants
  publisher_planned_traffic->publish( dynamics::conversions::to_ros_msg( latest_traffic_participant_set ) );
}

void
DecisionMakerInfrastructure::create_subscribers()
{
  subscriber_traffic_participant_set = create_subscription<adore_ros2_msgs::msg::TrafficParticipantSet>(
    "traffic_participants", 1, std::bind( &DecisionMakerInfrastructure::traffic_participants_callback, this, std::placeholders::_1 ) );
  main_timer = create_wall_timer( 100ms, std::bind( &DecisionMakerInfrastructure::run, this ) );
}

void
DecisionMakerInfrastructure::create_publishers()
{
  publisher_planned_traffic = create_publisher<adore_ros2_msgs::msg::TrafficParticipantSet>( "traffic_participants_with_trajectories", 1 );
  publisher_local_map       = create_publisher<adore_ros2_msgs::msg::Map>( "local_map", 1 );
  publisher_infrastructure_position = create_publisher<adore_ros2_msgs::msg::VisualizableObject>( "infrastructure_position", 1 );
}

void
DecisionMakerInfrastructure::load_parameters()
{
  declare_parameter( "debug_mode_active", true );
  get_parameter( "debug_mode_active", debug_mode_active );

  declare_parameter( "dt", 0.1 );
  get_parameter( "dt", dt );

  declare_parameter( "max_acceleration", 2.0 );
  declare_parameter( "min_acceleration", -2.0 );
  declare_parameter( "max_steering", 0.7 );
  get_parameter( "max_acceleration", command_limits.max_acceleration );
  get_parameter( "min_acceleration", command_limits.min_acceleration );
  get_parameter( "max_steering", command_limits.max_steering_angle );

  declare_parameter( "map file", "" );
  get_parameter( "map file", map_file_location );

  declare_parameter( "infrastructure_position_x", 0.0 );
  declare_parameter( "infrastructure_position_y", 0.0 );
  declare_parameter( "infrastructure_yaw", 0.0 );
  get_parameter( "infrastructure_position_x", infrastructure_pose.x );
  get_parameter( "infrastructure_position_y", infrastructure_pose.y );
  get_parameter( "infrastructure_yaw", infrastructure_pose.yaw );

  // Multi Agent PID related parameters
  std::vector<std::string> keys;
  std::vector<double>      values;
  declare_parameter( "multi_agent_PID_settings_keys", keys );
  declare_parameter( "multi_agent_PID_settings_values", values );
  get_parameter( "multi_agent_PID_settings_keys", keys );
  get_parameter( "multi_agent_PID_settings_values", values );

  if( keys.size() != values.size() )
  {
    RCLCPP_ERROR( this->get_logger(), "multi agent PID settings keys and values size mismatch!" );
    return;
  }
  for( size_t i = 0; i < keys.size(); ++i )
  {
    multi_agent_PID_settings.insert( { keys[i], values[i] } );
    std::cerr << "keys: " << keys[i] << ": " << values[i] << std::endl;
  }


  std::vector<double> validity_area_points; // request assistance polygon
  declare_parameter( "validity_polygon", std::vector<double>{} );
  get_parameter( "validity_polygon", validity_area_points );

  // Convert the parameter into a Polygon2d
  if( validity_area_points.size() >= 6 ) // minimum 3 x, 3 y
  {
    math::Polygon2d validity_area;
    validity_area.points.reserve( validity_area_points.size() / 2 );

    for( size_t i = 0; i < validity_area_points.size(); i += 2 )
    {
      double x = validity_area_points[i];
      double y = validity_area_points[i + 1];
      validity_area.points.push_back( { x, y } );
    }
    latest_traffic_participant_set.validity_area = validity_area;
  }

  multi_agent_PID_planner.set_parameters( multi_agent_PID_settings );
  // multi_agent_PID_planner_MRM           = multi_agent_PID_planner;
  // mutli_agent_PID_planner_MRM.max_speed = 0.0;
}

void
DecisionMakerInfrastructure::print_init_info()
{
  std::cout << "DecisionMakerInfrastructure node initialized.\n";
  std::cout << "Debug mode: " << ( debug_mode_active ? "Active" : "Inactive" ) << std::endl;
}

void
DecisionMakerInfrastructure::print_debug_info()
{
  double current_time_seconds = now().seconds();
  std::cerr << "------- Decision Maker Infrastructure Debug Information -------" << std::endl;
  std::cerr << "Current Time: " << current_time_seconds << " seconds" << std::endl;

  if( road_map.has_value() )
    std::cerr << "Local map data available.\n";
  else
    std::cerr << "No local map data.\n";

  std::cerr << "traffic participants " << latest_traffic_participant_set.participants.size() << std::endl;

  std::cerr << "------- ============================== -------" << std::endl;
}

void
DecisionMakerInfrastructure::compute_routes_for_traffic_participant_set( dynamics::TrafficParticipantSet& traffic_participant_set,
                                                                         const map::Map&                  road_map )
{
  for( auto& [id, participant] : traffic_participant_set.participants )
  {
    bool no_goal  = !participant.goal_point.has_value();
    bool no_route = !participant.route.has_value();

    if( !no_route )
      participant.route->map = std::make_shared<map::Map>( road_map );

    if( !no_goal && no_route )
    {
      participant.route = map::Route( participant.state, participant.goal_point.value(), road_map );
      if( participant.route->center_lane.empty() )
      {
        participant.route = std::nullopt;
        std::cerr << "No route found for traffic participant" << std::endl;
      }
    }
  }
}

void
DecisionMakerInfrastructure::publish_local_map()
{
  if( !road_map.has_value() )
    return;
  auto local_map = road_map->get_submap( infrastructure_pose, local_map_size, local_map_size );
  publisher_local_map->publish( map::conversions::to_ros_msg( local_map ) );
}

void
DecisionMakerInfrastructure::publish_infrastructure_position()
{
  adore_ros2_msgs::msg::VisualizableObject obj;
  obj.x               = infrastructure_pose.x;
  obj.y               = infrastructure_pose.y;
  obj.yaw             = infrastructure_pose.yaw;
  obj.z               = 0.0;
  obj.model           = "low_poly_trailer_model.dae";
  obj.header.frame_id = "world";

  publisher_infrastructure_position->publish( obj );
}

void
DecisionMakerInfrastructure::traffic_participants_callback( const adore_ros2_msgs::msg::TrafficParticipantSet& msg )
{
  latest_traffic_participant_set = dynamics::conversions::to_cpp_type( msg );
}

}; // namespace adore


#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(adore::DecisionMakerInfrastructure)
