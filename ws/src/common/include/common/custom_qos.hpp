#ifndef CUSTOM_QOS_H
#define CUSTOM_QOS_H

#include "rclcpp/rclcpp.hpp"

// Reliable no depth (subscriber and publisher)
// Profile designed for subscribers that always require the latest message
// without the message being dropped (reliable). Should be combined with a
// publisher with the same profile in order to always receive the absolute
// latest message.
static const rmw_qos_profile_t qos_profile_reliable_no_depth = {RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                                                1,
                                                                RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                                                                RMW_QOS_POLICY_DURABILITY_VOLATILE,
                                                                RMW_QOS_DEADLINE_DEFAULT,
                                                                RMW_QOS_LIFESPAN_DEFAULT,
                                                                RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
                                                                RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
                                                                false};

const rclcpp::QoS QOS_RELIABLE_NO_DEPTH =
    rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(qos_profile_reliable_no_depth), qos_profile_reliable_no_depth);

// Best effort no depth (subscriber and publisher)
// Profile designed for subscribers that always require the latest message, but
// do not mind if messages are dropped Should be combined with a publisher with
// the same profile in order to always receive the absolute latest message.
static const rmw_qos_profile_t qos_profile_best_effort_no_depth = {RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                                                                   1,
                                                                   RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                                                                   RMW_QOS_POLICY_DURABILITY_VOLATILE,
                                                                   RMW_QOS_DEADLINE_DEFAULT,
                                                                   RMW_QOS_LIFESPAN_DEFAULT,
                                                                   RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
                                                                   RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
                                                                   false};

const rclcpp::QoS QOS_BEST_EFFORT_NO_DEPTH = rclcpp::QoS(
    rclcpp::QoSInitialization::from_rmw(qos_profile_best_effort_no_depth), qos_profile_best_effort_no_depth);

#endif  // CUSTOM_QOS_H