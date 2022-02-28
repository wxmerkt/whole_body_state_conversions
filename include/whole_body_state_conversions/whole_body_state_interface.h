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
#include <map>
#include <cmath>
#include <mutex>

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

// Resolves to int8
enum ContactTypeEnum { LOCOMOTION = 0, MANIPULATION = 1 };
enum ContactStateEnum { UNKNOWN = 0, OPEN = 1, CLOSED = 2, SLIPPING = 3 };

struct ContactState {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Initialize the data structure for the contact state
   */
  ContactState() {}

  /**
   * @copybrief ContactState()
   *
   * @param[in] name  Contact name
   */
  ContactState(const std::string name) : name_(name) {}

  /**
   * @copybrief ContactState()
   *
   * @param[in] name              Contact name
   * @param[in] position          Contact position
   * @param[in] velocity          Contact velocity
   * @param[in] force             Contact force
   * @param[in] surface_normal    Normal vector at the contact surface
   * @param[in] surface_friction  Friction coefficient of the contact surface
   */
  ContactState(const std::string name, const pinocchio::SE3 &position, const pinocchio::Motion &velocity,
               const pinocchio::Force &force, const Eigen::Vector3d &surface_normal, const double surface_friction)
      : name_(name),
        position(position),
        velocity(velocity),
        force(force),
        surface_normal(surface_normal),
        surface_friction(surface_friction) {}
  ~ContactState() = default;

  std::string name_;
  pinocchio::SE3 position =
      pinocchio::SE3(NAN * Eigen::Matrix3d::Identity(), Eigen::Vector3d(NAN, NAN, NAN));  ///< Contact position
  pinocchio::Motion velocity =
      pinocchio::Motion(Eigen::Vector3d(NAN, NAN, NAN), Eigen::Vector3d(NAN, NAN, NAN));  ///< Contact velocity
  pinocchio::Force force =
      pinocchio::Force(Eigen::Vector3d(NAN, NAN, NAN), Eigen::Vector3d(NAN, NAN, NAN));  ///< Contact force
  Eigen::Vector3d surface_normal = Eigen::Vector3d(NAN, NAN, NAN);  ///< Normal vector at the contact surface
  double surface_friction;                                          ///< Friction coefficient of the contact surface
  std::size_t type;                                                 ///< Contact type
  ContactStateEnum state = ContactStateEnum::UNKNOWN;               ///< Classified contact state
};

typedef std::map<std::string, whole_body_state_conversions::ContactState> ContactStateMap;

struct WholeBodyState {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Initialize the data structure for the whole-body state
   */
  WholeBodyState() {}

  /**
   * @copybrief WholeBodyState()
   *
   * @param[in] nq  Dimension of the configuration tuple
   * @param[in] nv  Dimension of the velocity vector
   * @param[in] nu  Dimension of the control vector
   */
  WholeBodyState(const std::size_t nq, const std::size_t nv, const std::size_t nu)
      : q(Eigen::VectorXd::Zero(nq)),
        v(Eigen::VectorXd::Zero(nv)),
        a(Eigen::VectorXd::Zero(nv)),
        tau(Eigen::VectorXd(nu)) {}
  ~WholeBodyState() = default;

  double t = 0.0;                                          ///< Time from start (if part of a trajectory)
  Eigen::VectorXd q;                                       ///< Configuration vector (size nq)
  Eigen::VectorXd v;                                       ///< Tangent vector (size nv)
  Eigen::VectorXd a;                                       ///< System acceleration vector (size nv)
  Eigen::VectorXd tau;                                     ///< Torque vector (size njoints-2)
  whole_body_state_conversions::ContactStateMap contacts;  ///< Contact state (p, pd, f, s)
                                                           // p, pd, f, s - or ContactState!
};

class WholeBodyStateInterface {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  /**
   * @brief Initialize the interface for the whole-body state
   *
   * @param[in] model     Pinocchio model
   * @param[in] frame_id  Frame name of the inertial system (default: odom)
   */
  WholeBodyStateInterface(pinocchio::Model &model, const std::string frame_id = "odom")
      : model_(model), data_(model_), frame_id_(frame_id) {
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
   * @param a[in]         Acceleration vector (dimension: model.nv; default: zero acceleration)
   * @param tau[in]       Torque vector (dimension: model.nv; default: zero torque)
   * @param contacts[in]  Contact-state vector (optional: if we want to write the contact information)
   * @return The ROS message that contains the whole-body state
   */
  const whole_body_state_msgs::WholeBodyState &writeToMessage(
      double t, const Eigen::Ref<const Eigen::VectorXd> &q,
      const Eigen::Ref<const Eigen::VectorXd> &v = Eigen::VectorXd(),
      const Eigen::Ref<const Eigen::VectorXd> &a = Eigen::VectorXd(),
      const Eigen::Ref<const Eigen::VectorXd> &tau = Eigen::VectorXd(),
      std::map<std::string, ContactState> contacts = {}) {
    toMsg(msg_, t, q, v, a, tau, contacts);
    return msg_;
  }

  /**
   * @brief Conversion from vectors to `whole_body_state_msgs::WholeBodyState`
   *
   * @param msg[out]      ROS message that containts the whole-body state
   * @param t[in]         Time in secs
   * @param q[in]         Configuration vector (dimension: model.nq)
   * @param v[in]         Velocity vector (dimension: model.nv; default: zero velocity)
   * @param a[in]         Acceleration vector (dimension: model.nv; default: zero acceleration)
   * @param tau[in]       Torque vector (dimension: model.nv; default: zero torque)
   * @param contacts[in]  Contact-state vector (optional: if we want to write the contact information)
   */
  void toMsg(whole_body_state_msgs::WholeBodyState &msg, const double t, const Eigen::Ref<const Eigen::VectorXd> &q,
             const Eigen::Ref<const Eigen::VectorXd> &v = Eigen::VectorXd(),
             const Eigen::Ref<const Eigen::VectorXd> &a = Eigen::VectorXd(),
             const Eigen::Ref<const Eigen::VectorXd> &tau = Eigen::VectorXd(),
             std::map<std::string, whole_body_state_conversions::ContactState> contacts = {}) {
    if (q.size() != model_.nq) {
      throw std::invalid_argument("Expected q to be " + std::to_string(model_.nq) + " but received " +
                                  std::to_string(q.size()));
    }
    bool has_velocity = v.size() != 0;
    if (has_velocity && v.size() != model_.nv) {
      throw std::invalid_argument("Expected v to be 0 or " + std::to_string(model_.nv) + " but received " +
                                  std::to_string(v.size()));
    }
    bool has_acceleration = a.size() != 0;
    if (has_acceleration && v.size() != model_.nv) {
      throw std::invalid_argument("Expected a to be 0 or " + std::to_string(model_.nv) + " but received " +
                                  std::to_string(a.size()));
    }
    bool has_torque = tau.size() != 0;
    if (has_torque && static_cast<std::size_t>(tau.size()) != njoints_) {
      throw std::invalid_argument("Expected tau to be 0 or " + std::to_string(njoints_) + " but received " +
                                  std::to_string(tau.size()));
    }

    std::lock_guard<std::mutex> guard(mutex_);

    // Filling the time information
    msg.time = t;
    msg.header.stamp = ros::Time(t);
    msg.header.frame_id = frame_id_;

    // pinocchio::normalize(model_, q); <-- const-ref, we can't do this!

    // Filling the centroidal state
    if (has_velocity) {
      pinocchio::centerOfMass(model_, data_, q, v);
    } else {
      pinocchio::centerOfMass(model_, data_, q);
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
      msg.joints[j].acceleration = has_acceleration ? a(model_.joints[1].nv() + j) : 0.0;
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
      if (contact.position.translation().allFinite()) {
        position_tmp_ = contact.position;
      } else {
        position_tmp_ = pinocchio::updateFramePlacement(model_, data_, frame_id);
      }
      if (contact.velocity.toVector().allFinite()) {
        velocity_tmp_ = contact.velocity;
      } else {
        velocity_tmp_ =
            pinocchio::getFrameVelocity(model_, data_, frame_id, pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);
      }
      force_tmp_ = pinocchio::Force::Zero();
      if (contact.force.toVector().allFinite()) {
        force_tmp_ = contact.force;
      }
      if (contact.surface_normal.allFinite()) {
        nsurf_tmp_ = contact.surface_normal;
      } else {
        nsurf_tmp_ = Eigen::Vector3d::UnitZ();
      }
      double surface_friction = 1.;
      if (std::isfinite(contact.surface_friction)) {
        surface_friction = contact.surface_friction;
      }
      // Storing the contact position and velocity inside the message
      pinocchio::SE3::Quaternion quaternion(position_tmp_.rotation());
      msg.contacts[i].pose.position.x = position_tmp_.translation().x();
      msg.contacts[i].pose.position.y = position_tmp_.translation().y();
      msg.contacts[i].pose.position.z = position_tmp_.translation().z();
      msg.contacts[i].pose.orientation.x = quaternion.x();
      msg.contacts[i].pose.orientation.y = quaternion.y();
      msg.contacts[i].pose.orientation.z = quaternion.z();
      msg.contacts[i].pose.orientation.w = quaternion.w();
      msg.contacts[i].velocity.linear.x = velocity_tmp_.linear().x();
      msg.contacts[i].velocity.linear.y = velocity_tmp_.linear().y();
      msg.contacts[i].velocity.linear.z = velocity_tmp_.linear().z();
      msg.contacts[i].velocity.angular.x = velocity_tmp_.angular().x();
      msg.contacts[i].velocity.angular.y = velocity_tmp_.angular().y();
      msg.contacts[i].velocity.angular.z = velocity_tmp_.angular().z();
      msg.contacts[i].type = contact.type;
      msg.contacts[i].wrench.force.x = force_tmp_.linear().x();
      msg.contacts[i].wrench.force.y = force_tmp_.linear().y();
      msg.contacts[i].wrench.force.z = force_tmp_.linear().z();
      msg.contacts[i].wrench.torque.x = force_tmp_.angular().x();
      msg.contacts[i].wrench.torque.y = force_tmp_.angular().y();
      msg.contacts[i].wrench.torque.z = force_tmp_.angular().z();
      msg.contacts[i].surface_normal.x = nsurf_tmp_.x();
      msg.contacts[i].surface_normal.y = nsurf_tmp_.y();
      msg.contacts[i].surface_normal.z = nsurf_tmp_.z();
      msg.contacts[i].friction_coefficient = surface_friction;

      if (contact.state == whole_body_state_conversions::ContactStateEnum::UNKNOWN) {
        msg.contacts[i].status = whole_body_state_msgs::ContactState::UNKNOWN;
      } else if (contact.state == whole_body_state_conversions::ContactStateEnum::CLOSED) {
        msg.contacts[i].status = whole_body_state_msgs::ContactState::ACTIVE;
      } else if (contact.state == whole_body_state_conversions::ContactStateEnum::OPEN) {
        msg.contacts[i].status = whole_body_state_msgs::ContactState::INACTIVE;
      } else if (contact.state == whole_body_state_conversions::ContactStateEnum::SLIPPING) {
        msg.contacts[i].status = whole_body_state_msgs::ContactState::SLIPPING;
      }

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
   * @param a[out]         Acceleration vector (dimension: model.nv; default: zero acceleration)
   * @param tau[out]       Torque vector (dimension: model.nv; default: zero torque)
   * @param contacts[out]  Contact-state vector(optional: if we want to write the contact information)
   * @note TODO: Contact type and contact location / velocity are not yet supported.
   */
  void fromMsg(const whole_body_state_msgs::WholeBodyState &msg, double &t, Eigen::Ref<Eigen::VectorXd> q,
               Eigen::Ref<Eigen::VectorXd> v, Eigen::Ref<Eigen::VectorXd> a, Eigen::Ref<Eigen::VectorXd> tau,
               std::map<std::string, whole_body_state_conversions::ContactState> &contacts) {
    // Check dimensions
    if (q.size() != model_.nq) {
      throw std::invalid_argument("Expected q to be " + std::to_string(model_.nq) + " but received " +
                                  std::to_string(q.size()));
    }
    if (v.size() != model_.nv) {
      throw std::invalid_argument("Expected v to be " + std::to_string(model_.nv) + " but received " +
                                  std::to_string(v.size()));
    }
    if (a.size() != model_.nv) {
      throw std::invalid_argument("Expected a to be " + std::to_string(model_.nv) + " but received " +
                                  std::to_string(v.size()));
    }
    if (tau.size() != static_cast<int>(njoints_)) {
      throw std::invalid_argument("Expected tau to be " + std::to_string(njoints_) + " but received " +
                                  std::to_string(tau.size()));
    }
    if (msg.joints.size() != static_cast<std::size_t>(njoints_)) {
      throw std::invalid_argument("Expected msg.joints to be " + std::to_string(njoints_) + " but received " +
                                  std::to_string(msg.joints.size()));
    }
    // NB: We do not want to check contacts - they will get inserted into the map.
    // if (msg.contacts.size() != contacts.size()) {
    //   throw std::invalid_argument("Expected msg.contacts to be " + std::to_string(contacts.size()) + " but received
    //   " +
    //                               std::to_string(msg.contacts.size()));
    // }

    t = msg.time;

    std::lock_guard<std::mutex> guard(mutex_);

    // Retrieve the generalized position and velocity, and joint torques
    q.head<3>().setZero();
    q(3) = msg.centroidal.base_orientation.x;
    q(4) = msg.centroidal.base_orientation.y;
    q(5) = msg.centroidal.base_orientation.z;
    q(6) = msg.centroidal.base_orientation.w;
    v.head<3>().setZero();
    v(3) = msg.centroidal.base_angular_velocity.x;
    v(4) = msg.centroidal.base_angular_velocity.y;
    v(5) = msg.centroidal.base_angular_velocity.z;

    for (std::size_t j = 0; j < njoints_; ++j) {
      // TODO: Generalize to different floating-base types!
      // TODO: Check if joint exists!
      auto jointId = model_.getJointId(msg.joints[j].name) - 2;
      q(jointId + 7) = msg.joints[j].position;
      v(jointId + 6) = msg.joints[j].velocity;
      a(jointId + 6) = msg.joints[j].acceleration;
      tau(jointId) = msg.joints[j].effort;
    }
    pinocchio::normalize(model_, q);
    pinocchio::centerOfMass(model_, data_, q, v);
    q(0) = msg.centroidal.com_position.x - data_.com[0](0);
    q(1) = msg.centroidal.com_position.y - data_.com[0](1);
    q(2) = msg.centroidal.com_position.z - data_.com[0](2);
    v(0) = msg.centroidal.com_velocity.x - data_.vcom[0](0);
    v(1) = msg.centroidal.com_velocity.y - data_.vcom[0](1);
    v(2) = msg.centroidal.com_velocity.z - data_.vcom[0](2);

    // Retrieve the contact information
    for (const auto &contact : msg.contacts) {
      // Contact pose
      contacts[contact.name].position =
          pinocchio::SE3(Eigen::Quaterniond(contact.pose.orientation.w, contact.pose.orientation.x,
                                            contact.pose.orientation.y, contact.pose.orientation.z),
                         Eigen::Vector3d(contact.pose.position.x, contact.pose.position.y, contact.pose.position.z));
      // Contact velocity
      contacts[contact.name].velocity = pinocchio::Motion(
          Eigen::Vector3d(contact.velocity.linear.x, contact.velocity.linear.y, contact.velocity.linear.z),
          Eigen::Vector3d(contact.velocity.angular.x, contact.velocity.angular.y, contact.velocity.angular.z));
      // Contact wrench
      contacts[contact.name].force =
          pinocchio::Force(Eigen::Vector3d(contact.wrench.force.x, contact.wrench.force.y, contact.wrench.force.z),
                           Eigen::Vector3d(contact.wrench.torque.x, contact.wrench.torque.y, contact.wrench.torque.z));
      // Surface normal and friction coefficient
      contacts[contact.name].surface_normal.x() = contact.surface_normal.x;
      contacts[contact.name].surface_normal.y() = contact.surface_normal.y;
      contacts[contact.name].surface_normal.z() = contact.surface_normal.z;
      contacts[contact.name].surface_friction = contact.friction_coefficient;
      if (contact.type == contact.LOCOMOTION) {
        contacts[contact.name].type = whole_body_state_conversions::ContactTypeEnum::LOCOMOTION;
      } else if (contact.type == contact.MANIPULATION) {
        contacts[contact.name].type = whole_body_state_conversions::ContactTypeEnum::MANIPULATION;
      }
      if (contact.status == contact.UNKNOWN) {
        contacts[contact.name].state = whole_body_state_conversions::ContactStateEnum::UNKNOWN;
      } else if (contact.status == contact.ACTIVE) {
        contacts[contact.name].state = whole_body_state_conversions::ContactStateEnum::CLOSED;
      } else if (contact.status == contact.INACTIVE) {
        contacts[contact.name].state = whole_body_state_conversions::ContactStateEnum::OPEN;
      } else if (contact.status == contact.SLIPPING) {
        contacts[contact.name].state = whole_body_state_conversions::ContactStateEnum::SLIPPING;
      }
    }
  }

  const std::string &get_frame_id() const { return frame_id_; }

 private:
  pinocchio::Model model_;  ///< Pinocchio model
  pinocchio::Data data_;    ///< Pinocchio data
  std::size_t njoints_;     ///< Number of joints
  std::mutex mutex_;        ///< Mutex to prevent race condition on data_
  std::string frame_id_;    ///< Fixed frame name

  whole_body_state_msgs::WholeBodyState msg_;  ///< ROS message that contains the whole-body state

  pinocchio::SE3 position_tmp_;     ///< Temporal variable that stores the contact position
  pinocchio::Motion velocity_tmp_;  ///< Temporal variable that stores the contact velocity
  pinocchio::Force force_tmp_;      ///< Temporal variable that stores the contact force
  Eigen::Vector3d nsurf_tmp_;       ///< Temporal variable that stores the surface normal
};

}  // namespace whole_body_state_conversions

#endif  // WHOLE_BODY_STATE_MSGS_CONVERSIONS_H_
