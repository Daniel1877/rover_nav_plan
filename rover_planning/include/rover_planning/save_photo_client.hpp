#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"


#include <cstdlib>
#include <memory>


class ImageSaver : public rclcpp::Node
{
  //rclcpp::Client<std_srvs::srv::Empty>::SharedPtr client;

public:

  ImageSaver();

  ~ImageSaver();

  bool SavePhoto();

};
