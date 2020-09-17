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
#ifndef PSEN_SCAN_V2_MSG_DECODER_H
#define PSEN_SCAN_V2_MSG_DECODER_H

#include <functional>
#include <array>
#include <sstream>
#include <string>

#include "psen_scan_v2/function_pointers.h"
#include "psen_scan_v2/raw_scanner_data.h"
#include "psen_scan_v2/scanner_reply_msg.h"
#include "psen_scan_v2/crc_mismatch_exception.h"
#include "psen_scan_v2/monitoring_frame_msg.h"

namespace psen_scan_v2
{
enum class ScannerMsgType
{
  Reply,
  MonitoringFrame,
  Unknown
};

/**
 * @brief Deserializes raw byte data from the scanner.
 */
class MsgDecoder
{
public:
  /**
   * @brief Constructor.
   *
   * @param start_reply_callback Callback called whenever a StartReply is processed by the decodeAndDispatch method.
   * @param error_callback Callback called whenever an error occurs during deserialization.
   */
  MsgDecoder(const StartReplyCallback& start_reply_callback,
             const ErrorCallback& error_callback,
             const MonitoringFrameCallback& monitoring_frame_callback);

  /**
   * @brief decodeAndDispatch Deserializes the specified data and calls the appropriate callback to inform the user
   * about the type of the received data.
   *
   * @param data Container holding the received data from the scanner.
   * @param bytes_received Numbers of data received from the scanner.
   */
  void decodeAndDispatch(const RawScannerData& data, const std::size_t& bytes_received);

private:
  ScannerMsgType determineMsgType(const RawScannerData& data, const std::size_t& bytes_received);
  bool rawDataMatchesReply(const RawScannerData& data, const std::size_t& bytes_received);
  bool rawDataMatchesMonitoringFrame(const RawScannerData& data, const std::size_t& bytes_received);

  void handleReplyMsg(const RawScannerData& data, const std::size_t& bytes_received);
  void handleMonitoringFrameMsg(const RawScannerData& data, const std::size_t& bytes_received);
  void handleUnknownMsg(const RawScannerData& data, const std::size_t& bytes_received);

  StartReplyCallback start_reply_callback_;
  ErrorCallback error_callback_;
  MonitoringFrameCallback monitoring_frame_callback_;
};

inline MsgDecoder::MsgDecoder(const StartReplyCallback& start_reply_callback,
                              const ErrorCallback& error_callback,
                              const MonitoringFrameCallback& monitoring_frame_callback)
  : start_reply_callback_(start_reply_callback)
  , error_callback_(error_callback)
  , monitoring_frame_callback_(monitoring_frame_callback)
{
}

inline void MsgDecoder::decodeAndDispatch(const RawScannerData& data, const std::size_t& bytes_received)
{
  ScannerMsgType scanner_msg = determineMsgType(data, bytes_received);
  switch (scanner_msg)
  {
    case ScannerMsgType::Reply:
      handleReplyMsg(data, bytes_received);
      break;
    case ScannerMsgType::MonitoringFrame:
      handleMonitoringFrameMsg(data, bytes_received);
      break;
    case ScannerMsgType::Unknown:
      handleUnknownMsg(data, bytes_received);
      break;
  }
}

inline ScannerMsgType MsgDecoder::determineMsgType(const RawScannerData& data, const std::size_t& bytes_received)
{
  if (rawDataMatchesReply(data, bytes_received))
  {
    return ScannerMsgType::Reply;
  }
  if (rawDataMatchesMonitoringFrame(data, bytes_received))
  {
    return ScannerMsgType::MonitoringFrame;
  }
  return ScannerMsgType::Unknown;
}

inline bool MsgDecoder::rawDataMatchesReply(const RawScannerData& data, const std::size_t& bytes_received)
{
  return bytes_received == REPLY_MSG_FROM_SCANNER_SIZE;
}

inline bool MsgDecoder::rawDataMatchesMonitoringFrame(const RawScannerData& data, const std::size_t& bytes_received)
{
  // TODO
  return true;
}

inline void MsgDecoder::handleReplyMsg(const RawScannerData& data, const std::size_t& bytes_received)
{
  ScannerReplyMsg frame{ ScannerReplyMsg::fromRawData(data) };  // TODO how to handle throw?

  if (frame.type() == ScannerReplyMsgType::Start)
  {
    start_reply_callback_();
  }
  else
  {
    // TODO: Replace with stop reply callback in future.
    error_callback_("Unknown message type (Size " + std::to_string(bytes_received) + ")");
  }
}

inline void MsgDecoder::handleMonitoringFrameMsg(const RawScannerData& data, const std::size_t& bytes_received)
{
  MonitoringFrameMsg frame{ MonitoringFrameMsg::fromRawData(data) };
  monitoring_frame_callback_(frame);
}

inline void MsgDecoder::handleUnknownMsg(const RawScannerData& data, const std::size_t& bytes_received)
{
  // TODO: Replace with monitoring frame callback in future.
  error_callback_("Unknown message type (Size " + std::to_string(bytes_received) + ")");
}


}  // namespace psen_scan_v2

#endif  // PSEN_SCAN_V2_MSG_DECODER_H
