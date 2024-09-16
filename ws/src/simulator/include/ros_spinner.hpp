#pragma once

#include <Eigen/Core>

#include "drake/systems/framework/leaf_system.h"
#include "rclcpp/rclcpp.hpp"

/**
 * Provides a ROS spinner which is also a drake system and spins within a drake simulation at a desired frequency and
 * hence processes any messages published by instances of ROSPub. The user can register ROS subscriber at the spinner,
 * which receive ROS messages on a desired topic with the spinners frequency. Also this creates a drake vector output
 * port that can be connected to other drake systems. The mapping from the received message to the output port is done
 * via a user specified function.
 *
 * Example for a spinner that spins at 100Hz, with a subscriber to an IMU topic which is forwarded to a Drake Output
 * vector of size 4: \code ROSSpin spinner(0.01, node); auto out_put_port =
 * spinner.DeclareSubscriberAndPort<sensor_msgs::msg::Imu>("/imu", "imu_reading_port", rclcpp::QoS(0), 4,
 * Eigen::Vector4d(1,0,0,0),
 *                                  [](const ROSMessageT &ros_msg, Eigen::VectorXd &port_value)> msg_to_port_fun){
 *                                      port_value[0] =  ros_msg.orientation.w();
 *                                      port_value[1] =  ros_msg.orientation.x();
 *                                      port_value[2] =  ros_msg.orientation.y();
 *                                      port_value[3] =  ros_msg.orientation.z();
 *                                  });
 * \endcode
 */
class ROSSpin : public drake::systems::LeafSystem<double> {
 private:
  // ROS node handle
  rclcpp::Node::SharedPtr node_;
  // ROS callback group for this spinners
  rclcpp::CallbackGroup::SharedPtr call_back_group_;
  // ROS executor thst is called from within the simulation
  rclcpp::Executor::SharedPtr executor_;
  // vector/list of Eigen vectors which are updated once a new ROS message arrives.
  // Each vector/list entry refers to one registered subscriber
  std::vector<std::shared_ptr<Eigen::VectorXd>> discrete_states_from_messages_;
  std::vector<std::shared_ptr<drake::AbstractValue>> abstract_states_from_messages_;
  // vector/ist of the initial values that is used for the reset
  std::vector<Eigen::VectorXd> discrete_inital_values_;
  std::vector<std::shared_ptr<drake::AbstractValue>> abstract_inital_values_;
  // vector/list of ROS subscribers that have been registered
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscribers_;
  // a flag to stop the simulation
  std::atomic_bool stop_;

 public:
  /**
   * Creates a new ROS spinner which is also a drake system and once it is added a drake simulation spins at a desired
   * rate. Spinning refers to the ROS process of sending and receiving messages or doing timer callbacks of a node as
   * desribed in https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Executors.html
   *
   * @param period the spinning frequency, e.g. at which (simulation) frequency the ROS node is spined
   * @param node the ROS node to spin
   */
  ROSSpin(double period, rclcpp::Node::SharedPtr &node) : node_(node), stop_(false) {
    call_back_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_callback_group(call_back_group_, node_->get_node_base_interface());
    this->DeclarePeriodicUnrestrictedUpdateEvent(period, 0.0, &ROSSpin::UpdateEvent);
  };

  /**
   * Callback function that is called by the drake simulation at the desired spinning rate.
   * Internally it spins for the time needed to process all pending events and messages
   * then it puts any messages received by a subscriber registered at this spinner by DeclareSubscriberAndPort()
   * to the respective drake vector output port.
   *
   * @param context unused simulation context
   * @param state pointer of the drake state values of this system
   */
  drake::systems::EventStatus UpdateEvent(const drake::systems::Context<double> &context,
                                          drake::systems::State<double> *state) const {
    (void)context;
    executor_->spin_some();
    for (unsigned int i = 0; i < discrete_states_from_messages_.size(); i++) {
      state->get_mutable_discrete_state(i).SetFromVector(*(discrete_states_from_messages_[i]));
    }
    for (unsigned int i = 0; i < abstract_states_from_messages_.size(); i++) {
      state->get_mutable_abstract_state().get_mutable_value(i).SetFrom(*(abstract_states_from_messages_[i]));
    }
    if (!stop_.load()) {
      return drake::systems::EventStatus::Succeeded();
    } else {
      const_cast<std::atomic_bool &>(stop_).store(false);
      return drake::systems::EventStatus::ReachedTermination(this, "Simulation Stop");
    }
  };

  /**
   * Registers a ROS subscriber to a ROS topic and creates a drake output port on
   * which the messages received by the subscriber will be put.
   * The mapping from the ROS message type to an output vector is specified via a function.
   * Once a message is received this function being called and the vector output than put to the drake system.
   *
   * @tparam ROSMessageT the ROS message type of the subscriber
   * @param topic_name the ROS topic to subscribe to
   * @param port_name the name of the drake port where the ROS messages will be put to
   * @param ros_qos the QOS of the subscriber
   * @param port_vector_size the vector size of the drake port
   * @param init_val an inital value of the drake output port that is being used before a ROS message has been received
   * by the subscriber
   * @param msg_to_port_fun a function that is called to convert from the ROSMessageT to a drake vector.
   *                        The function as first parameter the ros_msg of type ROSMessageT the second parameter it the
   *                        port_value of type Eigen::VectorXd which has to be filles by the function
   * @return the drake output port
   */
  template <typename ROSMessageT>
  drake::systems::LeafOutputPort<double> &DeclareSubscriberAndPort(
      const std::string &topic_name,
      const std::string &port_name,
      const rclcpp::QoS &ros_qos,
      int port_vector_size,
      Eigen::VectorXd init_val,
      std::function<void(const ROSMessageT &ros_msg, Eigen::VectorXd &port_value)> msg_to_port_fun) {
    auto out_put_state = this->DeclareDiscreteState(port_vector_size);
    discrete_inital_values_.push_back(init_val);
    auto vec = std::make_shared<Eigen::VectorXd>(init_val);
    discrete_states_from_messages_.push_back(vec);
    rclcpp::SubscriptionOptions options;
    options.callback_group = call_back_group_;
    auto sub = node_->create_subscription<ROSMessageT>(
        topic_name,
        ros_qos,
        [vec, out_put_state, msg_to_port_fun](const typename ROSMessageT::SharedPtr msg) {
          msg_to_port_fun(*msg, *vec);
        },
        options);
    subscribers_.push_back(sub);
    return this->DeclareStateOutputPort(port_name, out_put_state);
  }

  /**
   * Registers a ROS subscriber to a ROS topic and creates an abstract drake output port on
   * which the messages received by the subscriber will be put.
   * The mapping from the ROS message type to an value of type PortT is specified via a function.
   * Once a message is received this function being called and the output value of type PortT than put to the drake
   * system.
   *
   * @tparam ROSMessageT the ROS message type of the subscriber
   * @tparam PortT the type of the abstract valued port
   * @param port_name the name of the drake port where the ROS messages will be put to
   * @param ros_qos the QOS of the subscriber
   * @param init_val an inital value of the drake output port that is being used before a ROS message has been received
   * by the subscriber
   * @param msg_to_port_fun a function that is called to convert from the ROSMessageT to a value of type PortT.
   *                        The function as first parameter the ros_msg of type ROSMessageT the second parameter is the
   *                        port_value of type PortT which has to be filled by the function
   * @return the drake abstract output port
   */
  template <typename ROSMessageT, typename PortT>
  drake::systems::LeafOutputPort<double> &DeclareSubscriberAndPort(
      const std::string &topic_name,
      const std::string &port_name,
      const rclcpp::QoS &ros_qos,
      const PortT &init_val,
      std::function<void(const ROSMessageT &ros_msg, PortT &port_value)> msg_to_port_fun) {
    auto output_state = this->DeclareAbstractState(drake::Value<PortT>(init_val));
    abstract_inital_values_.push_back(std::make_shared<drake::Value<PortT>>(init_val));
    auto abs_val = std::make_shared<drake::Value<PortT>>(init_val);
    abstract_states_from_messages_.push_back(abs_val);
    rclcpp::SubscriptionOptions options;
    options.callback_group = call_back_group_;
    auto sub = node_->create_subscription<ROSMessageT>(
        topic_name,
        ros_qos,
        [abs_val, output_state, msg_to_port_fun](const typename ROSMessageT::SharedPtr msg) {
          msg_to_port_fun(*msg, abs_val->get_mutable_value());
        },
        options);
    subscribers_.push_back(sub);
    return this->DeclareStateOutputPort(port_name, output_state);
  }

  /**
   * Can be called from another thread and will lead the spinner to stop the simulation.
   */
  void StopSimAsync() { stop_.store(true); }

  /**
   * Performs a reset, which means that each port will be set to its inital values defined by creation
   */
  void Reset() {
    for (unsigned int i = 0; i < discrete_states_from_messages_.size(); i++) {
      *discrete_states_from_messages_[i] = discrete_inital_values_[i];
    }
    for (unsigned int i = 0; i < abstract_states_from_messages_.size(); i++) {
      abstract_states_from_messages_[i]->SetFrom(*abstract_inital_values_[i]);
    }
  }

  /**
   * Sets the initial port value of a port which it will take after a Reset().
   * This method therefore replaces the initial value that has been set during DeclareSubscriberAndPort().
   * The initial port value is the value that a port has before a ROS 2 message for this port arrives.
   *
   * @param port_idx the index of the port
   * @param init_val the value
   */
  void SetDiscreteInitialPortValue(unsigned int port_idx, Eigen::VectorXd init_val) {
    discrete_inital_values_[port_idx] = init_val;
  }
};