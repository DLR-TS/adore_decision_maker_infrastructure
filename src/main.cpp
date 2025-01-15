#include "decision_maker_infrastructure.hpp"
#include "rclcpp/rclcpp.hpp"

int
main( int argc, char* argv[] )
{
  rclcpp::init( argc, argv );
  rclcpp::spin( std::make_shared<adore::DecisionMakerInfrastructure>() );
  rclcpp::shutdown();
  return 0;
}