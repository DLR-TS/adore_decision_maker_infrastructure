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

namespace adore
{

DecisionMakerInfrastructure::DecisionMakerInfrastructure() :
  Node( "decision_maker_infrastructure" )
{
  load_parameters();
  create_subscribers();
  create_publishers();
  print_init_info();
}

void
DecisionMakerInfrastructure::run()
{
  update_dynamic_subscriptions();
  dynamics::remove_old_participants( latest_traffic_participant_set, 0.5, this->now().seconds() );
  if (road_map.has_value())
  {
    compute_routes_for_traffic_participant_set(latest_traffic_participant_set, road_map.value());
  }
  all_vehicles_follow_routes();
  if( debug_mode_active )
    print_debug_info();
  publish_local_map();
  publish_infrastructure_position();
}

void
DecisionMakerInfrastructure::all_vehicles_follow_routes()
{
  multi_agent_PID_planner.plan_trajectories( latest_traffic_participant_set, command_limits );
  publisher_planned_traffic->publish( dynamics::conversions::to_ros_msg( latest_traffic_participant_set ) );
  
}

void
DecisionMakerInfrastructure::create_subscribers()
{

  main_timer = this->create_wall_timer( 100ms, std::bind( &DecisionMakerInfrastructure::run, this ) );
}

void
DecisionMakerInfrastructure::create_publishers()
{
  publisher_planned_traffic = create_publisher<adore_ros2_msgs::msg::TrafficParticipantSet>( "traffic_participants", 10 );
  publisher_local_map = create_publisher<adore_ros2_msgs::msg::Map>( "local_map", 10 );
  publisher_infrastructure_position = create_publisher<adore_ros2_msgs::msg::VehicleStateDynamic>( "vehicle_state/dynamic", 10 );
}

void
DecisionMakerInfrastructure::load_parameters()
{
  declare_parameter( "debug_mode_active", true );
  get_parameter( "debug_mode_active", debug_mode_active );

  declare_parameter( "dt", 0.05 );
  get_parameter( "dt", dt );

  declare_parameter( "max_acceleration", 2.0 );
  declare_parameter( "min_acceleration", -2.0 );
  declare_parameter( "max_steering", 0.7 );
  get_parameter( "max_acceleration", command_limits.max_acceleration );
  get_parameter( "min_acceleration", command_limits.min_acceleration );
  get_parameter( "max_steering", command_limits.max_steering_angle );

  declare_parameter( "R2S map file", "" );
  get_parameter( "R2S map file", map_file_location );

  declare_parameter( "infrastructure_position_x", 0.0 );
  declare_parameter( "infrastructure_position_y", 0.0 );
  get_parameter( "infrastructure_position_x", infrastructure_state.x );
  get_parameter( "infrastructure_position_y", infrastructure_state.y );

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

  multi_agent_PID_planner.set_parameters( multi_agent_PID_settings );

  // Load map
  road_map = map::MapLoader::load_from_r2s_file( map_file_location );
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

  std::cerr << "traffic participants " << latest_traffic_participant_set.size() << std::endl;

  std::cerr << "traffic participant subscriptions: " << traffic_participant_subscribers.size() << std::endl;


  std::cerr << "------- ============================== -------" << std::endl;
}

void
DecisionMakerInfrastructure::compute_routes_for_traffic_participant_set( dynamics::TrafficParticipantSet& traffic_participant_set,
                                                                         map::Map&                        road_map )
{
  for( auto& pair : traffic_participant_set )
  {
    if( pair.second.goal_point.has_value() )
    {
      pair.second.route = road_map.get_route( pair.second.state, pair.second.goal_point.value() );
    }
    else
    {
      std::cerr << "ERROR in decision maker infrastructure, one traffic participant has no goal point, route can not be computed"
                << std::endl;
      return;
    }
  }
}

void
DecisionMakerInfrastructure::traffic_participant_callback( const adore_ros2_msgs::msg::TrafficParticipant& msg )
{
  auto new_participant = dynamics::conversions::to_cpp_type( msg );
  dynamics::update_traffic_participants( latest_traffic_participant_set, new_participant );
}

void
DecisionMakerInfrastructure::update_dynamic_subscriptions()
{
  auto       topic_names_and_types = get_topic_names_and_types();
  std::regex valid_topic_regex( R"(^/([^/]+)/traffic_participant$)" );
  std::regex valid_type_regex( R"(^adore_ros2_msgs/msg/TrafficParticipant$)" );

  for( const auto& topic : topic_names_and_types )
  {
    const std::string&              topic_name = topic.first;
    const std::vector<std::string>& types      = topic.second;

    std::smatch match;
    if( std::regex_match( topic_name, match, valid_topic_regex )
        && std::any_of( types.begin(), types.end(),
                        [&]( const std::string& type ) { return std::regex_match( type, valid_type_regex ); } ) )
    {
      std::string vehicle_namespace = match[1].str();

      // Skip subscribing to own namespace
      if( vehicle_namespace == std::string( get_namespace() ).substr( 1 ) )
      {
        continue;
      }

      // Check if already subscribed
      if( traffic_participant_subscribers.count( vehicle_namespace ) > 0 )
      {
        continue;
      }

      // Create a new subscription
      auto subscription = create_subscription<adore_ros2_msgs::msg::TrafficParticipant>(
        topic_name, 10,
        [this, vehicle_namespace]( const adore_ros2_msgs::msg::TrafficParticipant& msg ) { traffic_participant_callback( msg ); } );

      traffic_participant_subscribers[vehicle_namespace] = subscription;

      RCLCPP_INFO( get_logger(), "Subscribed to new vehicle namespace: %s", vehicle_namespace.c_str() );
    }
  }
}

void
DecisionMakerInfrastructure::publish_local_map()
{
  if( !road_map.has_value())
    return;
  auto local_map = road_map->get_submap( infrastructure_state, local_map_size, local_map_size );
  publisher_local_map->publish( map::conversions::to_ros_msg( local_map ) );
}

void
DecisionMakerInfrastructure::publish_infrastructure_position()
{
  adore_ros2_msgs::msg::VehicleStateDynamic dynamic_msg = dynamics::conversions::to_ros_msg( infrastructure_state );
  publisher_infrastructure_position->publish( dynamic_msg );
}

}; // namespace adore
