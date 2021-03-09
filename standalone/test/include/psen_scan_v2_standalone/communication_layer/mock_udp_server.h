// Copyright (c) 2020-2021 Pilz GmbH & Co. KG
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

#ifndef PSEN_SCAN_V2_STANDALONE_TEST_MOCK_UDP_SERVER_H
#define PSEN_SCAN_V2_STANDALONE_TEST_MOCK_UDP_SERVER_H

#include <functional>
#include <string>
#include <thread>

#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>

#include "psen_scan_v2_standalone/data_conversion_layer/raw_scanner_data.h"

using boost::asio::ip::udp;

namespace psen_scan_v2_standalone_test
{
static const std::string MOCK_IP_ADDRESS{ "127.0.0.1" };

/**
 * @brief Class for the UDP communication with the scanner.
 *
 */
class MockUDPServer
{
public:
  /**
   * @brief Lists the different possible receive modi.
   */
  enum class ReceiveMode
  {
    //! @brief Wait for one message and then stop listening.
    single,
    //! @brief Continuously wait for new messages. In other words, after a message is received, automatically start
    //! listening for the next message.
    continuous
  };

public:
  using NewDataHandler =
      std::function<void(const udp::endpoint&, const psen_scan_v2_standalone::data_conversion_layer::RawData&)>;

public:
  ~MockUDPServer();

public:
  MockUDPServer(const unsigned short port, const NewDataHandler& new_data_handler);

public:
  void asyncReceive(const ReceiveMode& modi = ReceiveMode::single);

  void asyncSend(const udp::endpoint& receiver_of_data,
                 const psen_scan_v2_standalone::data_conversion_layer::RawData& data);

private:
  void handleSend(const boost::system::error_code& error, std::size_t bytes_transferred);
  void handleReceive(const ReceiveMode& modi,
                     const boost::system::error_code& error,
                     std::size_t /*bytes_transferred*/);

private:
  udp::endpoint remote_endpoint_;

  psen_scan_v2_standalone::data_conversion_layer::RawData recv_buffer_;

  boost::asio::io_service io_service_;
  // Prevent the run() method of the io_service from returning when there is no more work.
  boost::asio::io_service::work work_{ io_service_ };
  std::thread io_service_thread_;

  udp::socket socket_;

  NewDataHandler new_data_handler_;
};

}  // namespace psen_scan_v2_standalone_test

#endif  // PSEN_SCAN_V2_STANDALONE_TEST_MOCK_UDP_SERVER_H
