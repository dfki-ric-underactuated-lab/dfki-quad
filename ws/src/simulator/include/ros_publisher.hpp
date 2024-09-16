#pragma once

#include <Eigen/Core>
#include <type_traits>

#include "drake/common/value.h"
#include "drake/systems/framework/leaf_system.h"
#include "rclcpp/rclcpp.hpp"

/**
 * Provides a ROS 2 publisher that is also a Drake system.
 * It has vector input ports that can be connected in the drake simulation.
 * It provides a singe publisher which publishes the data from the input ports.
 * How the input ports' vectors are mapped to the ROS 2 message can be user defined.
 *
 * Example for creating a ROS 2 publisher that publishes an Imu message with 10Hz from two drake input ports, assuming
 * port sizes 4 and 3: \code ROSPub<sensor_msgs::msg::Imu,2> imu_pub(0.1, node_handle, "/imu", rclcpp::QoS(10),
 * std::array<int,2>{4,3}
 *  [](const std::array<Eigen::VectorXd,2> & vector_port_values, const drake::systems::Context<double> &
 * context,ROSMessageT & ros_msg){ ros_msg.orientation.w = vector_port_values[0][0]; ros_msg.orientation.x =
 * vector_port_values[0][1]; ros_msg.orientation.y = vector_port_values[0][2]; ros_msg.orientation.z =
 * vector_port_values[0][3]; ros_msg.angular_velocity.x = vector_port_values[1][0]; ros_msg.angular_velocity.y =
 * vector_port_values[1][1]; ros_msg.angular_velocity.z = vector_port_values[1][2];
 *  }
 *  )
 * \endcode
 *
 * @tparam ROSMessageT The ROS message type of the publisher
 * @tparam num_vector_ports the number of drake input ports
 * @tparam Abstract_portTypes pack of types of the abstract input ports. Each Type will create an abstract input port of
 * this type.
 */

template <typename ROSMessageT, int num_vector_ports, typename... Abstract_portTypes>
class ROSPub : public drake::systems::LeafSystem<double> {
  /**
   * Typedef for the function that specifies how the input ports translate to the ros message.
   * The user has to define a function with this signature that fills an empty ros message based on the drake vector
   * inputs.
   *
   * The function has the following parametersL
   * @param vector_port_values [input] an array that contains the input port vector values, where each Eigen::VectorXd
   * in the array refers to one input port
   * @param abstract_port_values [input] an array that contains the input port abstract values, where each AbstractValue
   * in the array refers to one input port
   * @param context [input] the drake simulation context
   * @param ros_msg [output] an empty ROS message that shall be filled by this function
   */
  typedef typename std::conditional<
      (sizeof...(Abstract_portTypes) <= 0),
      std::function<void(const std::array<Eigen::VectorXd, num_vector_ports> &vector_port_values,
                         const drake::systems::Context<double> &context,
                         ROSMessageT &ros_msg)>,
      std::function<void(
          const std::array<Eigen::VectorXd, num_vector_ports> &vector_port_values,
          const std::array<const drake::AbstractValue *, sizeof...(Abstract_portTypes)> &abstract_port_values,
          const drake::systems::Context<double> &context,
          ROSMessageT &ros_msg)> >::type PORT2MSGFUN;

 private:
  // ROS 2 node handle
  rclcpp::Node &node_;
  // ROS 2 publisher
  typename rclcpp::Publisher<ROSMessageT>::SharedPtr ros_pub_;
  // An array that contains the drake input ports
  std::array<drake::systems::InputPort<double> *, num_vector_ports> drake_vector_input_ports_;
  // An array that contains the drake abstract input ports
  std::array<drake::systems::InputPort<double> *, sizeof...(Abstract_portTypes)> drake_abstract_input_ports_;
  // the function which is called to translate from drake input ports to the ROS message
  PORT2MSGFUN port_to_msg_;

 public:
  /**
   * Constructs a new Drake ROS 2 publisher.
   *
   * @param period the rate in which the ROS 2 message is being published
   * @param node a node handle of the node at which the publisher will be registered
   * @param topic_name the ROS 2 topic name
   * @param ros_qos the ROS 2 QOS
   * @param port_vector_sizes a vector that contains the port vector sizes for all input ports of the drake system,
   * where each array entry refers to one port
   * @param port_to_msg a translation function which specifies how the drake vector inputs are translate to the ROS
   * message see typedef PORT2MSGFUN
   */
  ROSPub(double period,
         rclcpp::Node &node,
         const std::string &topic_name,
         const rclcpp::QoS &ros_qos,
         std::array<int, num_vector_ports> port_vector_sizes,
         PORT2MSGFUN port_to_msg)
      : node_(node), port_to_msg_(port_to_msg) {
    ros_pub_ = node_.create_publisher<ROSMessageT>(topic_name, ros_qos);
    for (unsigned int i = 0; i < num_vector_ports; i++) {
      drake_vector_input_ports_[i] =
          &this->DeclareVectorInputPort(topic_name + "Vector" + std::to_string(i), port_vector_sizes[i]);
    }
    // for(unsigned int i = 0; i < sizeof...(Abstract_portTypes); i++){
    {
      int i = 0;
      drake_abstract_input_ports_ = {
          &this->DeclareAbstractInputPort(topic_name + "Abstract" + std::to_string(i++),
                                          drake::Value<Abstract_portTypes>())...,
      };
    }
    //}
    this->DeclarePeriodicDiscreteUpdateEvent(
        period, 0.0, &ROSPub<ROSMessageT, num_vector_ports, Abstract_portTypes...>::UpdateEvent);
  };

  /**
   * This function is the callback which is called by the drake simulation by the rate specified as period in the
   * constructor. Internally this function reads the values of the vector inputs and then calls the port_to_msg_
   * function as specified in the constructor to map from the input ports to the ros message. This message is than being
   * published.
   *
   * @param context simulation context
   * @param discrete_val unused pointer to discrete state variables
   */
  void UpdateEvent(const drake::systems::Context<double> &context,
                   drake::systems::DiscreteValues<double> *discrete_val) const {
    (void)discrete_val;
    static std::array<Eigen::VectorXd, num_vector_ports> port_vals;
    for (unsigned int i = 0; i < num_vector_ports; i++) {
      port_vals[i] = drake_vector_input_ports_[i]->Eval(context);
    }
    static std::array<const drake::AbstractValue *, sizeof...(Abstract_portTypes)> abstract_port_vals;
    for (unsigned int i = 0; i < sizeof...(Abstract_portTypes); i++) {
      abstract_port_vals[i] = &drake_abstract_input_ports_[i]->template Eval<drake::AbstractValue>(context);
    }
    static ROSMessageT ros_msg;
    if constexpr (sizeof...(Abstract_portTypes) <= 0) {
      port_to_msg_(port_vals, context, ros_msg);
    } else {
      port_to_msg_(port_vals, abstract_port_vals, context, ros_msg);
    }
    ros_pub_->publish(ros_msg);
  };

  /**
   * Gets the current simulation time out of the drake simulator's context and converts it into a ROS timestamp.
   *
   * @param context the current drake simulator context
   * @return a ros timestamp representing the current simulator time
   */
  static inline rclcpp::Time GetRosTime(const drake::systems::Context<double> &context) {
    // Code snipped taken from
    // https://github.com/RobotLocomotion/drake-ros/blob/main/drake_ros/core/clock_system.cc#L16C1-L18
    rclcpp::Time now{0, 0, RCL_ROS_TIME};
    now += rclcpp::Duration::from_seconds(context.get_time());
    return now;
  };
};
