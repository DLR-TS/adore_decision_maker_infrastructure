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
  // Load map
  road_map = map::MapLoader::load_from_file( map_file_location );
  create_subscribers();
  create_publishers();
  print_init_info();
}

void
DecisionMakerInfrastructure::run()
{
  std::cerr << "RUN INFRA!!!!!!!!!!!!!!!!!!!!!1" << std::endl;
  update_dynamic_subscriptions();
  latest_traffic_participant_set.remove_old_participants( 1.0, this->now().seconds() );
  std::cerr << "traffic participants " << latest_traffic_participant_set.participants.size() << std::endl;

  if( road_map.has_value() )
  {
    compute_routes_for_traffic_participant_set( latest_traffic_participant_set, road_map.value() );
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
  auto start_time = std::chrono::high_resolution_clock::now(); // Start timer

  multi_agent_PID_planner.plan_trajectories( latest_traffic_participant_set );

  auto end_time = std::chrono::high_resolution_clock::now(); // End timer

  // Compute elapsed time in milliseconds
  double elapsed_time_ms = std::chrono::duration<double, std::milli>( end_time - start_time ).count();

  // Log the elapsed time
  RCLCPP_INFO( this->get_logger(), "Planning took %.3f ms", elapsed_time_ms );

  publisher_planned_traffic->publish( dynamics::conversions::to_ros_msg( latest_traffic_participant_set ) );
}

void
DecisionMakerInfrastructure::create_subscribers()
{

  main_timer                        = create_wall_timer( 100ms, std::bind( &DecisionMakerInfrastructure::run, this ) );
  subscriber_tracked_oriented_boxes = create_subscription<bob_perception_msgs::msg::TrackedOrientedBoxV2xArray>(
    "/madurban/track_level_fusion/fusion/fused", 10,
    std::bind( &DecisionMakerInfrastructure::tracked_oriented_boxes_callback, this, std::placeholders::_1 ) );
}

void
DecisionMakerInfrastructure::create_publishers()
{
  publisher_planned_traffic         = create_publisher<adore_ros2_msgs::msg::TrafficParticipantSet>( "traffic_participants", 10 );
  publisher_local_map               = create_publisher<adore_ros2_msgs::msg::Map>( "local_map", 1 );
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

  std::cerr << "traffic participant subscriptions: " << traffic_participant_subscribers.size() << std::endl;


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
    if( !no_goal && no_route )
    {
      participant.route = road_map.get_route( participant.state, participant.goal_point.value() );
      if( !participant.route.has_value() )
      {
        std::cerr << "No route found for traffic participant" << std::endl;
      }
    }
  }
}

void
DecisionMakerInfrastructure::traffic_participant_callback( const adore_ros2_msgs::msg::TrafficParticipant& msg )
{
  std::cerr << "received participant " << msg.tracking_id << std::endl;
  auto new_participant       = dynamics::conversions::to_cpp_type( msg );
  new_participant.state.time = now().seconds();
  latest_traffic_participant_set.update_traffic_participants( new_participant );

  std::cerr << "num participants  " << latest_traffic_participant_set.participants.size() << std::endl;
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
      // if( vehicle_namespace == std::string( get_namespace() ).substr( 1 ) )
      // {
      //   continue;
      // }

      // Check if already subscribed
      if( traffic_participant_subscribers.count( vehicle_namespace ) > 0 )
      {
        continue;
      }

      // Create a new subscription
      auto subscription = create_subscription<adore_ros2_msgs::msg::TrafficParticipant>(
        topic_name, 20,
        [this, vehicle_namespace]( const adore_ros2_msgs::msg::TrafficParticipant& msg ) { traffic_participant_callback( msg ); } );

      traffic_participant_subscribers[vehicle_namespace] = subscription;

      RCLCPP_INFO( get_logger(), "Subscribed to new vehicle namespace: %s", vehicle_namespace.c_str() );
    }
  }
}

void
DecisionMakerInfrastructure::tracked_oriented_boxes_callback( const bob_perception_msgs::msg::TrackedOrientedBoxV2xArray::SharedPtr msg )
{
  for( const auto& tracked_box : msg->tracks )
  {
    if( tracked_box.v2x_id != 0 )
      continue;
    adore_ros2_msgs::msg::TrafficParticipant traffic_participant;

    auto& state = traffic_participant.motion_state;

    state.x         = tracked_box.box.center_x;
    state.y         = tracked_box.box.center_y;
    state.yaw_angle = tracked_box.box.angle;

    double cos_yaw = std::cos( tracked_box.box.angle );
    double sin_yaw = std::sin( tracked_box.box.angle );

    state.vx = tracked_box.box.velocity_x * cos_yaw + tracked_box.box.velocity_y * sin_yaw;
    state.vy = -tracked_box.box.velocity_x * sin_yaw + tracked_box.box.velocity_y * cos_yaw;

    traffic_participant.classification.type_id = tracked_box.box.classification;

    traffic_participant.classification_certainty = tracked_box.box.classification_certainty;

    traffic_participant.physical_parameters.body_length = tracked_box.box.length;
    traffic_participant.physical_parameters.body_width  = tracked_box.box.width;
    traffic_participant.physical_parameters.body_height = tracked_box.box.height;

    traffic_participant.v2x_station_id = tracked_box.v2x_id;
    traffic_participant.tracking_id    = tracked_box.object_id;

    state.time = now().seconds();
    latest_traffic_participant_set.update_traffic_participants( dynamics::conversions::to_cpp_type( traffic_participant ) );

    std::cerr << "sent participant " << tracked_box.object_id << std::endl;
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
  obj.x     = infrastructure_pose.x;
  obj.y     = infrastructure_pose.y;
  obj.yaw   = infrastructure_pose.yaw;
  obj.z     = 0.0;
  obj.model = "low_poly_trailer_model.dae";
  publisher_infrastructure_position->publish( obj );
}

}; // namespace adore
