/**
* This file is part of gnss_comm.
*
* Copyright (C) 2021 Aerial Robotics Group, Hong Kong University of Science and Technology
* Author: CAO Shaozu (shaozu.cao@gmail.com)
*
* gnss_comm is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*/

#ifndef GNSS_ROS_HPP_
#define GNSS_ROS_HPP_

#include <gnss_comm/msg/gnss_time_msg.hpp>
#include <gnss_comm/msg/gnss_ephem_msg.hpp>
#include <gnss_comm/msg/gnss_glo_ephem_msg.hpp>
#include <gnss_comm/msg/gnss_obs_msg.hpp>
#include <gnss_comm/msg/gnss_meas_msg.hpp>
#include <gnss_comm/msg/gnss_best_xyz_msg.hpp>
#include <gnss_comm/msg/gnss_pvt_soln_msg.hpp>
#include <gnss_comm/msg/gnss_svs_msg.hpp>
#include <gnss_comm/msg/gnss_time_pulse_info_msg.hpp>
#include <gnss_comm/msg/stamped_float64_array.hpp>

#include "gnss_constant.hpp"
#include "gnss_utility.hpp"

namespace gnss_comm
{
    using GnssTimeMsg = msg::GnssTimeMsg;
    using GnssTimeMsgConstPtr = GnssTimeMsg::ConstSharedPtr;
    using GnssEphemMsg = msg::GnssEphemMsg;
    using GnssEphemMsgConstPtr = GnssEphemMsg::ConstSharedPtr;
    using GnssGloEphemMsg = msg::GnssGloEphemMsg;
    using GnssGloEphemMsgConstPtr = GnssGloEphemMsg::ConstSharedPtr;
    using GnssObsMsg = msg::GnssObsMsg;
    using GnssObsMsgConstPtr = GnssObsMsg::ConstSharedPtr;
    using GnssMeasMsg = msg::GnssMeasMsg;
    using GnssMeasMsgConstPtr = GnssMeasMsg::ConstSharedPtr;
    using GnssBestXYZMsg = msg::GnssBestXYZMsg;
    using GnssBestXYZMsgConstPtr = GnssBestXYZMsg::ConstSharedPtr;
    using GnssPVTSolnMsg = msg::GnssPVTSolnMsg;
    using GnssPVTSolnMsgConstPtr = GnssPVTSolnMsg::ConstSharedPtr;
    using GnssSvsMsg = msg::GnssSvsMsg;
    using GnssSvsMsgConstPtr = GnssSvsMsg::ConstSharedPtr;
    using GnssTimePulseInfoMsg = msg::GnssTimePulseInfoMsg;
    using GnssTimePulseInfoMsgConstPtr = GnssTimePulseInfoMsg::ConstSharedPtr;
    using StampedFloat64Array = msg::StampedFloat64Array;
    using StampedFloat64ArrayConstPtr = StampedFloat64Array::ConstSharedPtr;

    GnssEphemMsg ephem2msg(const EphemPtr &ephem_ptr);
    EphemPtr msg2ephem(const GnssEphemMsgConstPtr &gnss_ephem_msg);

    GnssGloEphemMsg glo_ephem2msg(const GloEphemPtr &glo_ephem_ptr);
    GloEphemPtr msg2glo_ephem(const GnssGloEphemMsgConstPtr &gnss_glo_ephem_msg);

    GnssMeasMsg meas2msg(const std::vector<ObsPtr> &meas);
    std::vector<ObsPtr> msg2meas(const GnssMeasMsgConstPtr &gnss_meas_msg);

    GnssTimePulseInfoMsg tp_info2msg(const TimePulseInfoPtr &tp_info);
    TimePulseInfoPtr msg2tp_info(const GnssTimePulseInfoMsgConstPtr &gnss_tp_info_msg);

    GnssBestXYZMsg best_xyz2msg(const BestXYZPtr &best_xyz);
    GnssPVTSolnMsg pvt2msg(const PVTSolutionPtr &pvt_soln);
    PVTSolutionPtr msg2pvt(const GnssPVTSolnMsgConstPtr &pvt_msg);

    GnssSvsMsg svs2msg(const std::vector<SvInfo> &svs);
}   // namespace gnss_comm

#endif
