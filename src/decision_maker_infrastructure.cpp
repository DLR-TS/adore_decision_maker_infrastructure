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

  DecisionMakerInfrastructure::DecisionMakerInfrastructure() : Node("decision_maker_infrastructure")
  {
    load_parameters();
    create_subscribers();
    create_publishers();
    print_init_info();
  }

  void 
  DecisionMakerInfrastructure::run()
  {
    dynamics::remove_old_participants(latest_traffic_participant_set.value(), 0.1, this->now().seconds());
    all_vehicles_follow_routes(); 
  }

  void
  DecisionMakerInfrastructure::all_vehicles_follow_routes()
  {
    multi_agent_PID_planner.plan_trajectories(latest_traffic_participant_set.value(), command_limits);
    publisher_planned_traffic->publish( dynamics::conversions::to_ros_msg( latest_traffic_participant_set .value()) );
  }

  void
  DecisionMakerInfrastructure::create_subscribers()
  {
    subscriber_traffic_participant = create_subscription<adore_ros2_msgs::msg::TrafficParticipant>(
      "/traffic_participant", 1, std::bind( &DecisionMakerInfrastructure::traffic_participant_callback, this, std::placeholders::_1 ) );

    main_timer = this->create_wall_timer(100ms, std::bind(&DecisionMakerInfrastructure::run, this));
  }

  void
  DecisionMakerInfrastructure::create_publishers()
  {
    publisher_planned_traffic = create_publisher<adore_ros2_msgs::msg::TrafficParticipantSet>( "planned_traffic", 10 );
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
    if( latest_traffic_participant_set )
      std::cerr << "Traffic participant set available.\n";
    else
      std::cerr << "No traffic participant set available.\n";
    
    std::cerr << "------- ============================== -------" << std::endl;
  }

  void 
  DecisionMakerInfrastructure::compute_routes_for_traffic_participant_set( dynamics::TrafficParticipantSet& traffic_participant_set, map::Map& road_map )
  {
    for (auto& pair : traffic_participant_set)
    {
      if (pair.second.goal_point.has_value())
      {
        pair.second.route.value() = road_map.get_route( pair.second.state, pair.second.goal_point.value() );
      } else
      {
        std::cerr << "ERROR in decision maker infrastructure, one traffic participant has no goal point, route can not be computed" << std::endl;
        goal_points_present = false;
        return;
      }
    }
  }

  void
  DecisionMakerInfrastructure::traffic_participant_callback( const adore_ros2_msgs::msg::TrafficParticipant& msg )
  {
    auto new_participant = dynamics::conversions::to_cpp_type( msg );
    dynamics::update_traffic_participants(latest_traffic_participant_set.value(), new_participant);
  }

};
