#include "convert/utils.hpp"
#include "ros_gz_bridge/convert/lrauv_msgs.hpp"

namespace ros_gz_bridge
{

// LRAUVInit
template<>
void
convert_ros_to_gz(
  const lrauv_msgs::msg::LRAUVInit & ros_msg,
  lrauv_gazebo_plugins::msgs::LRAUVInit & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));
  gz_msg.mutable_id_()->set_data(ros_msg.id);
  gz_msg.set_initlat_(ros_msg.init_lat);
  gz_msg.set_initlon_(ros_msg.init_lon);
  gz_msg.set_initz_(ros_msg.init_z);
  gz_msg.set_initpitch_(ros_msg.init_pitch);
  gz_msg.set_initroll_(ros_msg.init_roll);
  gz_msg.set_initheading_(ros_msg.init_heading);
  gz_msg.set_acommsaddress_(ros_msg.acomms_address);
}

template<>
void
convert_gz_to_ros(
  const lrauv_gazebo_plugins::msgs::LRAUVInit & gz_msg,
  lrauv_msgs::msg::LRAUVInit & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);
  ros_msg.id = gz_msg.id_().data();
  ros_msg.init_lat = gz_msg.initlat_();
  ros_msg.init_lon = gz_msg.initlon_();
  ros_msg.init_z = gz_msg.initz_();
  ros_msg.init_pitch = gz_msg.initpitch_();
  ros_msg.init_roll = gz_msg.initroll_();
  ros_msg.init_heading = gz_msg.initheading_();
  ros_msg.acomms_address = gz_msg.acommsaddress_();
}


// LRAUVState
template<>
void
convert_ros_to_gz(
  const lrauv_msgs::msg::LRAUVState & ros_msg,
  lrauv_gazebo_plugins::msgs::LRAUVState & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));
  gz_msg.set_propomega_(ros_msg.prop_omega);
  gz_msg.set_propthrust_(ros_msg.prop_thrust);
  gz_msg.set_proptorque_(ros_msg.prop_torque);
  gz_msg.set_rudderangle_(ros_msg.rudder_angle);
  gz_msg.set_elevatorangle_(ros_msg.elevator_angle);
  gz_msg.set_massposition_(ros_msg.mass_position);
  gz_msg.set_buoyancyposition_(ros_msg.buoyancy_position);
  gz_msg.set_depth_(ros_msg.depth);
  gz_msg.set_speed_(ros_msg.speed);
  gz_msg.set_latitudedeg_(ros_msg.latitude_deg);
  gz_msg.set_longitudedeg_(ros_msg.longitude_deg);
  gz_msg.set_netbuoy_(ros_msg.net_buoy);
  convert_ros_to_gz(ros_msg.pos, *(gz_msg.mutable_pos_()));
  convert_ros_to_gz(ros_msg.pos_rph, *(gz_msg.mutable_posrph_()));
  convert_ros_to_gz(ros_msg.pos_dot, *(gz_msg.mutable_posdot_()));
  convert_ros_to_gz(ros_msg.rate_uvw, *(gz_msg.mutable_rateuvw_()));
  convert_ros_to_gz(ros_msg.rate_pqr, *(gz_msg.mutable_ratepqr_()));
  gz_msg.set_northcurrent_(ros_msg.north_current);
  gz_msg.set_eastcurrent_(ros_msg.east_current);
  gz_msg.set_temperature_(ros_msg.temperature);
  gz_msg.set_salinity_(ros_msg.salinity);
  gz_msg.set_density_(ros_msg.density);
  gz_msg.set_batteryvoltage_(ros_msg.battery_voltage);
  gz_msg.set_batterycurrent_(ros_msg.battery_current);
  gz_msg.set_batterycharge_(ros_msg.battery_charge);
  gz_msg.set_batterypercentage_(ros_msg.battery_percentage);
}

template<>
void
convert_gz_to_ros(
  const lrauv_gazebo_plugins::msgs::LRAUVState & gz_msg,
  lrauv_msgs::msg::LRAUVState & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);
  
  ros_msg.prop_omega = gz_msg.propomega_();
  ros_msg.prop_thrust = gz_msg.propthrust_();
  ros_msg.prop_torque = gz_msg.proptorque_();
  ros_msg.rudder_angle = gz_msg.rudderangle_();
  ros_msg.elevator_angle = gz_msg.elevatorangle_();
  ros_msg.mass_position = gz_msg.massposition_();
  ros_msg.buoyancy_position = gz_msg.buoyancyposition_();
  ros_msg.depth = gz_msg.depth_();
  ros_msg.speed = gz_msg.speed_();
  ros_msg.latitude_deg = gz_msg.latitudedeg_();
  ros_msg.longitude_deg = gz_msg.longitudedeg_();
  ros_msg.net_buoy = gz_msg.netbuoy_();
  convert_gz_to_ros(gz_msg.pos_(), ros_msg.pos);
  convert_gz_to_ros(gz_msg.posrph_(), ros_msg.pos_rph);
  convert_gz_to_ros(gz_msg.posdot_(), ros_msg.pos_dot);
  convert_gz_to_ros(gz_msg.rateuvw_(), ros_msg.rate_uvw);
  convert_gz_to_ros(gz_msg.ratepqr_(), ros_msg.rate_pqr);
  ros_msg.north_current = gz_msg.northcurrent_();
  ros_msg.east_current = gz_msg.eastcurrent_();
  ros_msg.temperature = gz_msg.temperature_();
  ros_msg.salinity = gz_msg.salinity_();
  ros_msg.density = gz_msg.density_();
  ros_msg.battery_voltage = gz_msg.batteryvoltage_();
  ros_msg.battery_current = gz_msg.batterycurrent_();
  ros_msg.battery_charge = gz_msg.batterycharge_();
  ros_msg.battery_percentage = gz_msg.batterypercentage_();
}


// LRAUVCommand
template<>
void
convert_ros_to_gz(
  const lrauv_msgs::msg::LRAUVCommand & ros_msg,
  lrauv_gazebo_plugins::msgs::LRAUVCommand & gz_msg)
{
  convert_ros_to_gz(ros_msg.header, (*gz_msg.mutable_header()));
  gz_msg.set_propomegaaction_(ros_msg.prop_omega_action);
  gz_msg.set_rudderangleaction_(ros_msg.rudder_angle_action);
  gz_msg.set_elevatorangleaction_(ros_msg.elevator_angle_action);
  gz_msg.set_masspositionaction_(ros_msg.mass_position_action);
  gz_msg.set_buoyancyaction_(ros_msg.buoyancy_action);
}

template<>
void
convert_gz_to_ros(
  const lrauv_gazebo_plugins::msgs::LRAUVCommand & gz_msg,
  lrauv_msgs::msg::LRAUVCommand & ros_msg)
{
  convert_gz_to_ros(gz_msg.header(), ros_msg.header);
  ros_msg.prop_omega_action = gz_msg.propomegaaction_();
  ros_msg.rudder_angle_action = gz_msg.rudderangleaction_();
  ros_msg.elevator_angle_action = gz_msg.elevatorangleaction_();
  ros_msg.mass_position_action = gz_msg.masspositionaction_();
  ros_msg.buoyancy_action = gz_msg.buoyancyaction_();
}


// LRAUVRangeBearingRequest
template<>
void
convert_ros_to_gz(
  const lrauv_msgs::msg::LRAUVRangeBearingRequest & ros_msg,
  lrauv_gazebo_plugins::msgs::LRAUVRangeBearingRequest & gz_msg)
{
  gz_msg.set_to(ros_msg.to);
  gz_msg.set_req_id(ros_msg.req_id);
}

template<>
void
convert_gz_to_ros(
  const lrauv_gazebo_plugins::msgs::LRAUVRangeBearingRequest & gz_msg,
  lrauv_msgs::msg::LRAUVRangeBearingRequest & ros_msg)
{
  ros_msg.to = gz_msg.to();
  ros_msg.req_id = gz_msg.req_id();
}


// LRAUVRangeBearingResponse
template<>
void
convert_ros_to_gz(
  const lrauv_msgs::msg::LRAUVRangeBearingResponse & ros_msg,
  lrauv_gazebo_plugins::msgs::LRAUVRangeBearingResponse & gz_msg)
{
  gz_msg.set_req_id(ros_msg.req_id);
  convert_ros_to_gz(ros_msg.bearing, *(gz_msg.mutable_bearing()));
  gz_msg.set_range(ros_msg.range);
}

template<>
void
convert_gz_to_ros(
  const lrauv_gazebo_plugins::msgs::LRAUVRangeBearingResponse & gz_msg,
  lrauv_msgs::msg::LRAUVRangeBearingResponse & ros_msg)
{
  ros_msg.req_id = gz_msg.req_id();
  convert_gz_to_ros(gz_msg.bearing(), ros_msg.bearing);
  ros_msg.range = gz_msg.range();
}

}  // namespace ros_gz_bridge
