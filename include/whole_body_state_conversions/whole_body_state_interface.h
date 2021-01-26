///////////////////////////////////////////////////////////////////////////////
// BSD 3-Clause License
//
// Copyright (c) 2020-2021, University of Oxford, University of Edinburgh
// Copyright note valid unless otherwise stated in individual files.
// All rights reserved.
///////////////////////////////////////////////////////////////////////////////

#ifndef WHOLE_BODY_STATE_MSGS_CONVERSIONS_H_
#define WHOLE_BODY_STATE_MSGS_CONVERSIONS_H_

#include <iostream>
#include <unordered_map>

#include <pinocchio/fwd.hpp>
#include <pinocchio/container/aligned-vector.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

#include <whole_body_state_msgs/WholeBodyState.h>

namespace whole_body_state_conversions {

struct ContactState {
 public:
  /**
   * @brief Initialize the data structure for the contact state
   */
  ContactState() {}

  /**
   * @copybrief ContactState()
   *
   * @param[in] name  Contact name
   */
  ContactState(const std::string &name) : name(name) {}
  ~ContactState() = default;

  std::string name;
  pinocchio::Force force = pinocchio::Force::Zero();
  Eigen::Vector3d surface_normal = Eigen::Vector3d(0, 0, 1);
  double surface_friction;                                 ///< Friction coefficient
  pinocchio::SE3 position = pinocchio::SE3::Identity();    // NOT YET SUPPORTED IN TRANSLATOR !!!
  pinocchio::Motion velocity = pinocchio::Motion::Zero();  // NOT YET SUPPORTED IN TRANSLATOR
  // TODO: Contact type
};

typedef std::unordered_map<std::string, whole_body_state_conversions::ContactState> ContactStateMap;

struct WholeBodyState {
 public:
  /**
   * @brief Initialize the data structure for the whole-body state
   */
  WholeBodyState() {}

  /**
   * @copybrief WholeBodyState()
   *
   * @param[in] nq  Dimension of the configuration tuple
   * @param[in] nv  Dimension of the velocity vector
   * @param[in] nc  Number of contacts
   */
  WholeBodyState(const std::size_t nq, const std::size_t nv, const std::size_t nu, const std::size_t nc = 0)
      : q(Eigen::VectorXd::Zero(nq)), v(Eigen::VectorXd::Zero(nv)), tau(Eigen::VectorXd(nu)) {
    contacts.reserve(nc);
  }
  ~WholeBodyState() = default;

  double t = 0.0;                                          ///< Time from start (if part of a trajectory)
  Eigen::VectorXd q;                                       ///< Configuration vector (size nq)
  Eigen::VectorXd v;                                       ///< Tangent vector (size nv)
  Eigen::VectorXd tau;                                     ///< Torque vector (size njoints-2)
  whole_body_state_conversions::ContactStateMap contacts;  ///< Contact state (p, pd, f, s)
                                                           // p, pd, f, s - or ContactState!
};

class WholeBodyStateInterface {
 public:
  /**
   * @brief Initialize the interface for the whole-body state
   *
   * @param[in] model     Pinocchio model
   * @param[in] frame_id  Frame name of the inertial system (default: odom)
   */
  WholeBodyStateInterface(pinocchio::Model &model, const std::string frame_id = "odom") : model_(model), data_(model_) {
    // Setup message
    msg_.header.frame_id = frame_id;
    njoints_ = model_.njoints - 2;
    msg_.joints.resize(njoints_);
    for (std::size_t j = 0; j < njoints_; ++j) {
      msg_.joints[j].name = model_.names[j + 2];
    }
  }

  /**
   * @brief Conversion from vectors to `whole_body_state_msgs::WholeBodyState`
   *
   * @param t[in]         Time in secs
   * @param q[in]         Configuration vector (dimension: model.nq)
   * @param v[in]         Velocity vector (dimension: model.nv; default: zero velocity)
   * @param tau[in]       Torque vector (dimension: model.nv; default: zero torque)
   * @param contacts[in]  Contact-state vector (optional: if we want to write the contact information)
   * @return The ROS message that contains the whole-body state
   * @note TODO: Contact type and contact location / velocity are not yet supported.
   */
  whole_body_state_msgs::WholeBodyState writeToMessage(double t, const Eigen::VectorXd &q,
                                                       const Eigen::VectorXd &v = Eigen::VectorXd(),
                                                       const Eigen::VectorXd &tau = Eigen::VectorXd(),
                                                       std::unordered_map<std::string, ContactState> contacts = {}) {
    toMsg(msg_, t, q, v, tau, contacts);
    return msg_;
  }

  /**
   * @brief Conversion from vectors to `whole_body_state_msgs::WholeBodyState`
   *
   * @param msg[out]      ROS message that containts the whole-body state
   * @param t[in]         Time in secs
   * @param q[in]         Configuration vector (dimension: model.nq)
   * @param v[in]         Velocity vector (dimension: model.nv; default: zero velocity)
   * @param tau[in]       Torque vector (dimension: model.nv; default: zero torque)
   * @param contacts[in]  Contact-state vector (optional: if we want to write the contact information)
   * @note TODO: Contact type and contact location / velocity are not yet supported.
   */
  void toMsg(whole_body_state_msgs::WholeBodyState &msg, const double t, const Eigen::VectorXd &q,
             const Eigen::VectorXd &v = Eigen::VectorXd(), const Eigen::VectorXd &tau = Eigen::VectorXd(),
             std::unordered_map<std::string, whole_body_state_conversions::ContactState> contacts = {}) {
    if (q.size() != model_.nq) {
      throw std::invalid_argument("Expected q to be " + std::to_string(model_.nq) + " but received " +
                                  std::to_string(q.size()));
    }

    bool has_velocity = v.size() != 0;
    if (v.size() != 0 && v.size() != model_.nv) {
      throw std::invalid_argument("Expected v to be 0 or " + std::to_string(model_.nv) + " but received " +
                                  std::to_string(v.size()));
    }

    bool has_torque = tau.size() != 0;
    if (tau.size() != 0 && static_cast<std::size_t>(tau.size()) != njoints_) {
      throw std::invalid_argument("Expected tau to be 0 or " + std::to_string(njoints_) + " but received " +
                                  std::to_string(tau.size()));
    }

    // Filling the time information
    msg.time = t;
    msg.header.stamp = ros::Time(t);

    // Filling the centroidal state
    if (has_velocity) {
      pinocchio::centerOfMass(model_, data_, q);
    } else {
      pinocchio::centerOfMass(model_, data_, q, v);
    }
    // Center of mass
    msg.centroidal.com_position.x = data_.com[0].x();
    msg.centroidal.com_position.y = data_.com[0].y();
    msg.centroidal.com_position.z = data_.com[0].z();
    msg.centroidal.com_velocity.x = has_velocity ? data_.vcom[0].x() : 0.0;
    msg.centroidal.com_velocity.y = has_velocity ? data_.vcom[0].y() : 0.0;
    msg.centroidal.com_velocity.z = has_velocity ? data_.vcom[0].z() : 0.0;
    // Base
    msg.centroidal.base_orientation.x = q(3);
    msg.centroidal.base_orientation.y = q(4);
    msg.centroidal.base_orientation.z = q(5);
    msg.centroidal.base_orientation.w = q(6);
    msg.centroidal.base_angular_velocity.x = has_velocity ? v(3) : 0.0;
    msg.centroidal.base_angular_velocity.y = has_velocity ? v(4) : 0.0;
    msg.centroidal.base_angular_velocity.z = has_velocity ? v(5) : 0.0;
    // Momenta
    const pinocchio::Force &momenta = pinocchio::computeCentroidalMomentum(model_, data_);
    msg.centroidal.momenta.linear.x = momenta.linear().x();
    msg.centroidal.momenta.linear.y = momenta.linear().y();
    msg.centroidal.momenta.linear.z = momenta.linear().z();
    msg.centroidal.momenta.angular.x = momenta.angular().x();
    msg.centroidal.momenta.angular.y = momenta.angular().y();
    msg.centroidal.momenta.angular.z = momenta.angular().z();
    const pinocchio::Force &momenta_rate = pinocchio::computeCentroidalMomentumTimeVariation(model_, data_);
    msg.centroidal.momenta_rate.linear.x = has_velocity ? momenta_rate.linear().x() : 0.0;
    msg.centroidal.momenta_rate.linear.y = has_velocity ? momenta_rate.linear().y() : 0.0;
    msg.centroidal.momenta_rate.linear.z = has_velocity ? momenta_rate.linear().z() : 0.0;
    msg.centroidal.momenta_rate.angular.x = has_velocity ? momenta_rate.angular().x() : 0.0;
    msg.centroidal.momenta_rate.angular.y = has_velocity ? momenta_rate.angular().y() : 0.0;
    msg.centroidal.momenta_rate.angular.z = has_velocity ? momenta_rate.angular().z() : 0.0;

    // Filling the joint state
    if (msg.joints.size() != njoints_) {
      msg.joints.resize(njoints_);
    }
    for (std::size_t j = 0; j < njoints_; ++j) {
      msg.joints[j].name = model_.names[2 + j];
      msg.joints[j].position = q(model_.joints[1].nq() + j);
      msg.joints[j].velocity = has_velocity ? v(model_.joints[1].nv() + j) : 0.0;
      msg.joints[j].effort = has_torque ? tau(j) : 0.0;
    }

    // Contacts
    const std::size_t ncontacts = contacts.size();
    msg.contacts.resize(ncontacts);
    std::size_t i = 0;
    for (const auto &contact_item : contacts) {
      const std::string &contact_name = contact_item.first;
      const ContactState &contact = contact_item.second;
      msg.contacts[i].name = contact_name;
      // Pose
      pinocchio::FrameIndex frame_id = model_.getFrameId(msg.contacts[i].name);
      if (static_cast<int>(frame_id) > model_.nframes) {
        throw std::runtime_error("Frame '" + contact_name + "' not found.");
      }
      // TODO: Option to retrieve the contact position from an argument (map)
      const pinocchio::SE3 &oMf = pinocchio::updateFramePlacement(model_, data_, frame_id);
      pinocchio::SE3::Quaternion oMf_quaternion(oMf.rotation());
      // TODO: Option to retrieve the contact velocity from an argument (map)
      pinocchio::Motion ovf =
          pinocchio::getFrameVelocity(model_, data_, frame_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
      // Storing the contact position and velocity inside the message
      msg.contacts[i].pose.position.x = oMf.translation().x();
      msg.contacts[i].pose.position.y = oMf.translation().y();
      msg.contacts[i].pose.position.z = oMf.translation().z();
      msg.contacts[i].pose.orientation.x = oMf_quaternion.x();
      msg.contacts[i].pose.orientation.y = oMf_quaternion.y();
      msg.contacts[i].pose.orientation.z = oMf_quaternion.z();
      msg.contacts[i].pose.orientation.w = oMf_quaternion.w();
      msg.contacts[i].velocity.linear.x = ovf.linear().x();
      msg.contacts[i].velocity.linear.y = ovf.linear().y();
      msg.contacts[i].velocity.linear.z = ovf.linear().z();
      msg.contacts[i].velocity.angular.x = ovf.angular().x();
      msg.contacts[i].velocity.angular.y = ovf.angular().y();
      msg.contacts[i].velocity.angular.z = ovf.angular().z();
      // Surface properties
      msg.contacts[i].friction_coefficient = contact.surface_friction;
      msg.contacts[i].surface_normal.x = contact.surface_normal.x();
      msg.contacts[i].surface_normal.y = contact.surface_normal.y();
      msg.contacts[i].surface_normal.z = contact.surface_normal.z();
      // Contact Force/Torque
      // msg.contacts[i].type = TODO: Type!
      msg.contacts[i].wrench.force.x = contact.force.linear().x();
      msg.contacts[i].wrench.force.y = contact.force.linear().y();
      msg.contacts[i].wrench.force.z = contact.force.linear().z();
      msg.contacts[i].wrench.torque.x = contact.force.angular().x();
      msg.contacts[i].wrench.torque.y = contact.force.angular().y();
      msg.contacts[i].wrench.torque.z = contact.force.angular().z();
      ++i;
    }
  }

  /**
   * @brief Conversion from whole_body_state_msgs::WholeBodyState to deserialized quantities
   *
   * @param msg[in]        ROS message that contains the whole-body state
   * @param t[out]         Time in secs
   * @param q[out]         Configuration vector (dimension: model.nq)
   * @param v[out]         Velocity vector (dimension: model.nv; default: zero velocity)
   * @param tau[out]       Torque vector (dimension: model.nv; default: zero torque)
   * @param contacts[out]  Contact-state vector(optional: if we want to write the contact information)
   * @note TODO: Contact type and contact location / velocity are not yet supported.
   */
  void fromMsg(const whole_body_state_msgs::WholeBodyState &msg, double &t, Eigen::Ref<Eigen::VectorXd> q,
               Eigen::Ref<Eigen::VectorXd> v, Eigen::Ref<Eigen::VectorXd> tau,
               std::unordered_map<std::string, whole_body_state_conversions::ContactState> &contacts) {
    t = msg.time;
    // TODO: Check dimensions
    // q.resize(model_.nq);
    // v.resize(model_.nv);
    // tau.resize(model_.njoints - 2);
    // p, pd, f, s

    // Retrieve the generalized position and velocity, and joint torques
    q(3) = msg.centroidal.base_orientation.x;
    q(4) = msg.centroidal.base_orientation.y;
    q(5) = msg.centroidal.base_orientation.z;
    q(6) = msg.centroidal.base_orientation.w;
    v(3) = msg.centroidal.base_angular_velocity.x;
    v(4) = msg.centroidal.base_angular_velocity.y;
    v(5) = msg.centroidal.base_angular_velocity.z;

    // TODO: Check msg.joints.size() !!
    for (std::size_t j = 0; j < msg.joints.size(); ++j) {
      // TODO: Generalize to different floating-base types!
      // TODO: Check if joint exists!
      auto jointId = model_.getJointId(msg.joints[j].name) - 2;
      q(jointId + 7) = msg.joints[j].position;
      v(jointId + 6) = msg.joints[j].velocity;
      tau(jointId) = msg.joints[j].effort;
    }
    pinocchio::centerOfMass(model_, data_, q, v);
    q(0) = msg.centroidal.com_position.x - data_.com[0](0);
    q(1) = msg.centroidal.com_position.y - data_.com[0](1);
    q(2) = msg.centroidal.com_position.z - data_.com[0](2);
    v(0) = msg.centroidal.com_velocity.x - data_.vcom[0](0);
    v(1) = msg.centroidal.com_velocity.y - data_.vcom[0](1);
    v(2) = msg.centroidal.com_velocity.z - data_.vcom[0](2);

    // Retrieve the contact information
    for (const auto &contact : msg.contacts) {
      // 'p': Contact pose
      contacts[contact.name].position =
          pinocchio::SE3(Eigen::Quaterniond(contact.pose.orientation.w, contact.pose.orientation.x,
                                            contact.pose.orientation.y, contact.pose.orientation.z),
                         Eigen::Vector3d(contact.pose.position.x, contact.pose.position.y, contact.pose.position.z));
      // 'pd': Contact velocity
      contacts[contact.name].velocity = pinocchio::Motion(
          Eigen::Vector3d(contact.velocity.linear.x, contact.velocity.linear.y, contact.velocity.linear.z),
          Eigen::Vector3d(contact.velocity.angular.x, contact.velocity.angular.y, contact.velocity.angular.z));
      // 'f': Contact wrench
      contacts[contact.name].force =
          pinocchio::Force(Eigen::Vector3d(contact.wrench.force.x, contact.wrench.force.y, contact.wrench.force.z),
                           Eigen::Vector3d(contact.wrench.torque.x, contact.wrench.torque.y, contact.wrench.torque.z));
      // 's': Surface normal and friction coefficient
      contacts[contact.name].surface_normal.x() = contact.surface_normal.x;
      contacts[contact.name].surface_normal.y() = contact.surface_normal.y;
      contacts[contact.name].surface_normal.z() = contact.surface_normal.z;
      contacts[contact.name].surface_friction = contact.friction_coefficient;
    }
  }

 private:
  pinocchio::Model model_;  ///< Pinocchio model
  pinocchio::Data data_;    ///< Pinocchio data
  std::size_t njoints_;     ///< Number of joints

  whole_body_state_msgs::WholeBodyState msg_;  ///< ROS message that contains the whole-body state
};

}  // namespace whole_body_state_conversions

#endif  // WHOLE_BODY_STATE_MSGS_CONVERSIONS_H_
