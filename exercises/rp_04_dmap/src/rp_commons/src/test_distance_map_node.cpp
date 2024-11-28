#include <memory>

#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rp_commons/distance_map.h"
#include "rp_commons/distance_map_utils.h"
#include "rp_commons/grid.h"
#include "rp_commons/grid_map.h"

class DistanceMapNode : public rclcpp::Node {
 public:
  DistanceMapNode() : Node("distance_map_node") {
    // It is subscribed to the map topic: the simulator publishes the map and with this subscriber we capture it
    _map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "map", 10,
        std::bind(&DistanceMapNode::mapCallback, this, std::placeholders::_1));
    
    // Initialize publisher for metric dmap
    _metric_dmap_pub = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
        "distance_map", 10);
    
    // Initialize publisher for row derivatives
    _dmap_drows_pub = this -> create_publisher<nav_msgs::msg::OccupancyGrid>("drows", 10);

    // Initialize publisher for column derivatives
    _dmap_dcols_pub = this -> create_publisher<nav_msgs::msg::OccupancyGrid>("dcols", 10);

    // Initialize publisher for magnitudes
    _magnitudes_pub = this -> create_publisher<nav_msgs::msg::OccupancyGrid>("magnitudes", 10);
  }

 private:
  
  //This callback function is called when something is published on the "map" topic
  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Map size: [%dx%d] Creating distance map",
                msg->info.width, msg->info.height);
    // Load the occupancy grid msg into the grid map and distance map
    _grid_map.loadFromOccupancyGrid(*msg); // We fill our gridmap with the updated gridmap(it is inside "msg")
    _distance_map.loadFromOccupancyGrid(*msg); // We now compute the distance map (we have for each cell the pointer to its closest neighbor obstacle)
    // They are two different functions, because _grid_map and _distance_map are two different classes

    // Compute the distance map in metric space
    _distance_map.extractDistancesSquared(_metric_dmap, _distance_map.d2_max()); // In this function we pass "_metric_dmap" that will be filled with the distances squared of every cell
    for (auto& cell : _metric_dmap._cells) {
      // From pixel^2 to meters
      cell =  std::sqrt(cell) * _distance_map.resolution(); //cell is pixel^2 so I compute its square root, then I multiply it by the resolution of the distance map (resoultion is a function inherited from GridMapping)
    }

    RCLCPP_INFO(this->get_logger(), "Publishing distance map");
    publishDistanceMap(*msg);

    // Compute gradients of the distance map
    computeGradients(_metric_dmap, _dmap_drows, _dmap_dcols);

    RCLCPP_INFO(this->get_logger(), "Publishing derivative maps");
    // TODO: Publish row derivatives
    publishDerivatives(*msg, _dmap_drows, _dmap_drows_pub);
    // TODO: Publish column derivatives
    publishDerivatives(*msg, _dmap_dcols, _dmap_dcols_pub);

    // Compute magnitudes of the gradients
    computeMagnitudes(_dmap_drows, _dmap_dcols, _magnitudes);

    RCLCPP_INFO(this->get_logger(), "Publishing magnitudes");
    // TODO: Publish magnitudes
    publishDerivatives(*msg, _magnitudes, _magnitudes_pub);
  }

  void publishDistanceMap(const nav_msgs::msg::OccupancyGrid& map) {
    // Create a new occupancy grid message
    nav_msgs::msg::OccupancyGrid metric_dmap_msg;
    metric_dmap_msg.header = map.header;
    metric_dmap_msg.info = map.info;
    metric_dmap_msg.data.resize(_metric_dmap.size());

    // Initialize the distances vector
    std::vector<float> distances(_distance_map.size());

    // Fill the distances vector with the metric distances
    for (size_t r = 0; r < _distance_map.rows(); ++r) {
      for (size_t c = 0; c < _distance_map.cols(); ++c) {
        size_t index =
            _distance_map.cols() * (_distance_map.rows() - r - 1) + c;
        // If the cell has no parent or is unknown, set the distance to 0
        if (_distance_map.at(r,c).parent == nullptr or _grid_map.at(r,c) == GridMap::UNKNOWN) {
          distances[index] = 0.0;
          continue;
        }
        distances[index] = _metric_dmap.at(r, c);
      }
    }
                                                
    // Get the minimum and maximum distances
    const float min_dist = *std::min_element(distances.begin(), distances.end()); //These two functions returns the pointer to the element, so i use the * symbol
    const float max_dist = *std::max_element(distances.begin(), distances.end());

    // Normalize the distances and fill the occupancy grid message
    for (size_t i = 0; i < distances.size(); i++) {
      if (distances[i] == 0) {
        metric_dmap_msg.data[i] = 127;
      } else {
        // Normalize the distance
        float normalized_distance = (distances[i] - min_dist) / (max_dist - min_dist);

        // Fill the occupancy grid message
        //It is a "nav_msgs/OccupancyGridMessage" --> I have to check the documentation online to see its fields
        metric_dmap_msg.data[i] = static_cast<uint8_t>(normalized_distance * 100); //I have to multiply by 100 because (in the documentation says that) Occupancy probabilities are in the range [0, 100]
      }
    }

    _metric_dmap_pub->publish(metric_dmap_msg);
  }

  void publishDerivatives(
      const nav_msgs::msg::OccupancyGrid& map,
      const Grid_<float>& dmap_derivatives,
      rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
          dmap_derivatives_pub) {
    // Create a new occupancy grid message
    nav_msgs::msg::OccupancyGrid dmap_derivatives_msg;
    dmap_derivatives_msg.header = map.header;
    dmap_derivatives_msg.info = map.info;
    dmap_derivatives_msg.data.resize(dmap_derivatives.size());

    // Initialize the derivatives vector
    std::vector<float> derivatives(_distance_map.size());

    // Fill the derivatives vector with the distance map derivatives
    for (size_t r = 0; r < _distance_map.rows(); ++r) {
      for (size_t c = 0; c < _distance_map.cols(); ++c) {
        size_t index =
            _distance_map.cols() * (_distance_map.rows() - r - 1) + c;
        // If the cell has no parent or is unknown, set the derivative to 0
        if (_distance_map.at(r, c).parent == nullptr or
            _grid_map.at(r, c) == GridMap::UNKNOWN) {
          derivatives[index] = 0;
          continue;
        }
        derivatives[index] = dmap_derivatives.at(r, c);
      }
    }

    // Get the minimum and maximum derivatives
    const float min_dist =
        *std::min_element(derivatives.begin(), derivatives.end());
    const float max_dist =
        *std::max_element(derivatives.begin(), derivatives.end());

    // Normalize the derivatives and fill the occupancy grid message
    for (size_t r = 0; r < _distance_map.rows(); ++r) {
      for (size_t c = 0; c < _distance_map.cols(); ++c) {
        size_t index =
            _distance_map.cols() * (_distance_map.rows() - r - 1) + c;
        // If the cell has no parent or is unknown, set the derivative to 127
        if (_distance_map.at(r, c).parent == nullptr or
            _grid_map.at(r, c) == GridMap::UNKNOWN) {
          dmap_derivatives_msg.data[index] = 127;
        } else {
          // Normalize the derivative
          float normalized_distance =
              (derivatives[index] - min_dist) / (max_dist - min_dist);
          // Fill the occupancy grid message
          dmap_derivatives_msg.data[index] =
              static_cast<uint8_t>(normalized_distance * 100.);
        }
      }
    }

    dmap_derivatives_pub->publish(dmap_derivatives_msg);
  }

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr _map_sub;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr _metric_dmap_pub;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr _dmap_drows_pub;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr _dmap_dcols_pub;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr _magnitudes_pub;
  DistanceMap _distance_map;
  GridMap _grid_map;
  Grid_<float> _metric_dmap, _dmap_drows, _dmap_dcols, _magnitudes;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DistanceMapNode>());
  rclcpp::shutdown();
  return 0;
}