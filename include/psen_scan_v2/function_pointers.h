// Copyright (c) 2020 Pilz GmbH & Co. KG
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#ifndef PSEN_SCAN_V2_FUNCTION_POINTERS_H
#define PSEN_SCAN_V2_FUNCTION_POINTERS_H

#include <functional>

namespace psen_scan_v2
{
using SendRequestCallback = std::function<void()>;
using StartReplyCallback = std::function<void()>;
using StopReplyCallback = std::function<void()>;
using ErrorCallback = std::function<void(const std::string&)>;
}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_FUNCTION_POINTERS_H
