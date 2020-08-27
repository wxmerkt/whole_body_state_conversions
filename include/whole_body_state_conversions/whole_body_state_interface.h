//
// Copyright (c) 2020, University of Oxford
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//  * Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of  nor the names of its contributors may be used to
//    endorse or promote products derived from this software without specific
//    prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//

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

struct ContactState {
 public:
  ContactState() {}
  ContactState(const std::string &name_in) : name(name_in) {}
  ~ContactState() = default;

  std::string name;
  pinocchio::Force force = pinocchio::Force::Zero();
  Eigen::Vector3d surface_normal = Eigen::Vector3d(0, 0, 1);
  double surface_friction;
  pinocchio::SE3 position = pinocchio::SE3::Identity();  // NOT YET SUPPORTED IN TRANSLATOR !!!
};

class WholeBodyStateInterface {
 public:
  WholeBodyStateInterface(pinocchio::Model &model) : pinocchio_model_(model), pinocchio_data_(pinocchio_model_) {
    // Setup message
    msg_.header.frame_id = "world";
    njoints_ = pinocchio_model_.njoints - 2;
    msg_.joints.resize(njoints_);
    for (int i = 0; i < njoints_; ++i) {
      const std::string &joint_name = pinocchio_model_.names[i + 2];
      msg_.joints[i].name = joint_name;
    }
  }

  /**
   * @brief Conversion from vectors to whole_body_state_msgs::WholeBodyState
   *
   * @param t Timestep
   * @param q Configuration vector
   * @param v Velocity vector (optional)
   * @param tau Torque vector (optional)
   * @param contacts ContactState vector (optional)
   * @return whole_body_state_msgs::WholeBodyState
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
   * @brief Conversion from vectors to whole_body_state_msgs::WholeBodyState
   *
   * @param msg whole_body_state_msgs::WholeBodyState (reference, will be modified)
   * @param t Timestep
   * @param q Configuration vector
   * @param v Velocity vector (optional)
   * @param tau Torque vector (optional)
   * @param contacts ContactState vector (optional)
   * @note TODO: Contact type and contact location / velocity are not yet supported.
   */
  void toMsg(whole_body_state_msgs::WholeBodyState &msg, const double t, const Eigen::VectorXd &q,
             const Eigen::VectorXd &v = Eigen::VectorXd(), const Eigen::VectorXd &tau = Eigen::VectorXd(),
             std::unordered_map<std::string, ContactState> contacts = {}) {
    if (q.size() != pinocchio_model_.nq) {
      throw std::invalid_argument("Expected q to be " + std::to_string(pinocchio_model_.nq) + " but received " +
                                  std::to_string(q.size()));
    }

    bool has_velocity = v.size() != 0;
    if (v.size() != 0 && v.size() != pinocchio_model_.nv) {
      throw std::invalid_argument("Expected v to be 0 or " + std::to_string(pinocchio_model_.nv) + " but received " +
                                  std::to_string(v.size()));
    }

    bool has_torque = tau.size() != 0;
    if (tau.size() != 0 && tau.size() != njoints_) {
      throw std::invalid_argument("Expected tau to be 0 or " + std::to_string(njoints_) + " but received " +
                                  std::to_string(tau.size()));
    }

    // Filling the centroidal state
    if (has_velocity) {
      pinocchio::centerOfMass(pinocchio_model_, pinocchio_data_, q);
    } else {
      pinocchio::centerOfMass(pinocchio_model_, pinocchio_data_, q, v);
    }

    // Center of mass
    msg.centroidal.com_position.x = pinocchio_data_.com[0].x();
    msg.centroidal.com_position.y = pinocchio_data_.com[0].y();
    msg.centroidal.com_position.z = pinocchio_data_.com[0].z();
    msg.centroidal.com_velocity.x = has_velocity ? pinocchio_data_.vcom[0].x() : 0.0;
    msg.centroidal.com_velocity.y = has_velocity ? pinocchio_data_.vcom[0].y() : 0.0;
    msg.centroidal.com_velocity.z = has_velocity ? pinocchio_data_.vcom[0].z() : 0.0;
    // Base
    msg.centroidal.base_orientation.x = q(3);
    msg.centroidal.base_orientation.y = q(4);
    msg.centroidal.base_orientation.z = q(5);
    msg.centroidal.base_orientation.w = q(6);
    msg.centroidal.base_angular_velocity.x = has_velocity ? v(3) : 0.0;
    msg.centroidal.base_angular_velocity.y = has_velocity ? v(4) : 0.0;
    msg.centroidal.base_angular_velocity.z = has_velocity ? v(5) : 0.0;

    // Momenta
    const pinocchio::Force &momenta = pinocchio::computeCentroidalMomentum(pinocchio_model_, pinocchio_data_);
    msg.centroidal.momenta.linear.x = momenta.linear().x();
    msg.centroidal.momenta.linear.y = momenta.linear().y();
    msg.centroidal.momenta.linear.z = momenta.linear().z();
    msg.centroidal.momenta.angular.x = momenta.angular().x();
    msg.centroidal.momenta.angular.y = momenta.angular().y();
    msg.centroidal.momenta.angular.z = momenta.angular().z();
    const pinocchio::Force &momenta_rate =
        pinocchio::computeCentroidalMomentumTimeVariation(pinocchio_model_, pinocchio_data_);
    msg.centroidal.momenta_rate.linear.x = has_velocity ? momenta_rate.linear().x() : 0.0;
    msg.centroidal.momenta_rate.linear.y = has_velocity ? momenta_rate.linear().y() : 0.0;
    msg.centroidal.momenta_rate.linear.z = has_velocity ? momenta_rate.linear().z() : 0.0;
    msg.centroidal.momenta_rate.angular.x = has_velocity ? momenta_rate.angular().x() : 0.0;
    msg.centroidal.momenta_rate.angular.y = has_velocity ? momenta_rate.angular().y() : 0.0;
    msg.centroidal.momenta_rate.angular.z = has_velocity ? momenta_rate.angular().z() : 0.0;

    // Joints
    if (static_cast<int>(msg.joints.size()) != njoints_) msg.joints.resize(njoints_);
    for (int i = 0; i < njoints_; ++i) {
      msg.joints[i].name = pinocchio_model_.names[2 + i];
      msg.joints[i].position = q(pinocchio_model_.joints[1].nq() + i);
      msg.joints[i].velocity = has_velocity ? v(pinocchio_model_.joints[1].nv() + i) : 0.0;
      msg.joints[i].effort = has_torque ? tau(i) : 0.0;
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
      pinocchio::FrameIndex frame_id = pinocchio_model_.getFrameId(msg.contacts[i].name);
      if (static_cast<int>(frame_id) > pinocchio_model_.nframes) {
        throw std::runtime_error("Frame '" + contact_name + "' not found.");
      }

      // TODO: Option to retrieve the contact position from an argument (map)
      const pinocchio::SE3 &oMf = pinocchio::updateFramePlacement(pinocchio_model_, pinocchio_data_, frame_id);
      pinocchio::SE3::Quaternion oMf_quaternion(oMf.rotation());

      // TODO: Option to retrieve the contact velocity from an argument (map)
      pinocchio::Motion ovf = pinocchio::getFrameVelocity(pinocchio_model_, pinocchio_data_, frame_id,
                                                          pinocchio::ReferenceFrame::LOCAL_WORLD_ALIGNED);

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

 private:
  pinocchio::Model pinocchio_model_;
  pinocchio::Data pinocchio_data_;
  int njoints_;

  whole_body_state_msgs::WholeBodyState msg_;
};

#endif  // WHOLE_BODY_STATE_MSGS_CONVERSIONS_H_
