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

#ifndef PSEN_SCAN_V2_TEST_MOCK_UDP_SERVER_H
#define PSEN_SCAN_V2_TEST_MOCK_UDP_SERVER_H

#include <memory>
#include <thread>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/array.hpp>

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include "psen_scan_v2/raw_scanner_data.h"

using boost::asio::ip::udp;

namespace psen_scan_v2_test
{
static const std::string MOCK_IP_ADDRESS{ "127.0.0.1" };

/**
 * @brief Class for the UDP communication with the scanner.
 *
 */
class MockUDPServer
{
public:
  ~MockUDPServer();

public:
  MockUDPServer(const unsigned short port);

  MOCK_CONST_METHOD1(receivedUdpMsg, void(const udp::endpoint&));

public:
  void asyncReceive();

  template <unsigned int N>
  void asyncSend(const udp::endpoint& receiver_of_data, const psen_scan_v2::RawDataContainer<N>& data);

private:
  void handleSend(const boost::system::error_code& error, std::size_t bytes_transferred);
  void handleReceive(const boost::system::error_code& error, std::size_t /*bytes_transferred*/);

private:
  udp::endpoint remote_endpoint_;

  psen_scan_v2::RawScannerData recv_buffer_;

  boost::asio::io_service io_service_;
  // Prevent the run() method of the io_service from returning when there is no more work.
  boost::asio::io_service::work work_{ io_service_ };
  std::thread io_service_thread_;

  udp::socket socket_;
};

MockUDPServer::MockUDPServer(const unsigned short port)
  : socket_(io_service_, udp::endpoint(boost::asio::ip::address_v4::from_string(MOCK_IP_ADDRESS), port))
{
  io_service_thread_ = std::thread([this]() { io_service_.run(); });
}

MockUDPServer::~MockUDPServer()
{
  io_service_.stop();
  if (io_service_thread_.joinable())
  {
    io_service_thread_.join();
  }
  socket_.close();
}

void MockUDPServer::handleSend(const boost::system::error_code& error, std::size_t bytes_transferred)
{
  if (error || bytes_transferred == 0)
  {
    std::cerr << "UDP server mock failed to send data. Error msg: " << error.message() << std::endl;
  }
}

template <unsigned int N>
void MockUDPServer::asyncSend(const udp::endpoint& receiver_of_data, const psen_scan_v2::RawDataContainer<N>& data)
{
  io_service_.post([this, receiver_of_data, data]() {
    socket_.async_send_to(boost::asio::buffer(data),
                          receiver_of_data,
                          boost::bind(&MockUDPServer::handleSend,
                                      this,
                                      boost::asio::placeholders::error,
                                      boost::asio::placeholders::bytes_transferred));
  });
}

void MockUDPServer::handleReceive(const boost::system::error_code& error, std::size_t bytes_received)
{
  if (error || bytes_received == 0)
  {
    std::cerr << "UDP server mock failed to receive data. Error msg: " << error.message() << std::endl;
    return;
  }
  receivedUdpMsg(remote_endpoint_);
}

void MockUDPServer::asyncReceive()
{
  io_service_.post([this]() {
    socket_.async_receive_from(boost::asio::buffer(recv_buffer_),
                               remote_endpoint_,
                               boost::bind(&MockUDPServer::handleReceive,
                                           this,
                                           boost::asio::placeholders::error,
                                           boost::asio::placeholders::bytes_transferred));
  });
}

}  // namespace psen_scan_v2_test

#endif  // PSEN_SCAN_V2_TEST_MOCK_UDP_SERVER_H
