// clang-format off
/*
** Copyright (2020-2021) Robotics Systems Lab - ETH Zurich:
** Linghao Zhang, Jonas Junger, Lennart Nachtigall
**
** This file is part of the maxon_epos_ethercat_sdk.
** The maxon_epos_ethercat_sdk is free software: you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation, either version 3 of the License, or
** (at your option) any later version.
**
** The maxon_epos_ethercat_sdk is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with the maxon_epos_ethercat_sdk. If not, see <https://www.gnu.org/licenses/>.
*/
// clang-format on

#pragma once

#include <cstdint>
#include <iostream>

namespace maxon
{
enum class ModeOfOperationEnum : int8_t
{
  NA = 0,
  ProfiledPositionMode = 1,
  ProfiledVelocityMode = 3,
  HomingMode = 6,
  CyclicSynchronousPositionMode = 8,
  CyclicSynchronousVelocityMode = 9,
  CyclicSynchronousTorqueMode = 10
};

}  // namespace maxon

std::ostream& operator << (std::ostream& os, const maxon::ModeOfOperationEnum modeOfOperation);
