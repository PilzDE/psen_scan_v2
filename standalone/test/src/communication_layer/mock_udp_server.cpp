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

#include <iostream>
#include <stdexcept>
#include <thread>

#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/system/error_code.hpp>

#include "psen_scan_v2_standalone/communication_layer/mock_udp_server.h"

#include "psen_scan_v2_standalone/data_conversion_layer/raw_scanner_data.h"

namespace psen_scan_v2_standalone_test
{
MockUDPServer::MockUDPServer(const unsigned short port, const NewMessageCallback& new_msg_callback)
  : socket_(io_service_, udp::endpoint(boost::asio::ip::address_v4::from_string(MOCK_IP_ADDRESS), port))
  , new_msg_callback_(new_msg_callback)
{
  if (!new_msg_callback_)
  {
    throw std::invalid_argument("New message callback must not be null");
  }

  recv_buffer_.resize(psen_scan_v2_standalone::data_conversion_layer::MAX_UDP_PAKET_SIZE);
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
  if (error)
  {
    std::cerr << "UDP server mock failed to send data. Error msg: " << error.message() << std::endl;
  }
  std::cout << "MockUDPServer: Data successfully send" << std::endl;
}

void MockUDPServer::asyncSend(const udp::endpoint& receiver_of_data,
                              const psen_scan_v2_standalone::data_conversion_layer::RawData& data)
{
  io_service_.post([this, receiver_of_data, data]() {
    socket_.async_send_to(boost::asio::buffer(data.data(), data.size()),
                          receiver_of_data,
                          boost::bind(&MockUDPServer::handleSend,
                                      this,
                                      boost::asio::placeholders::error,
                                      boost::asio::placeholders::bytes_transferred));
  });
}

void MockUDPServer::handleReceive(const ReceiveMode& modi,
                                  const boost::system::error_code& error,
                                  std::size_t bytes_received)
{
  if (error)
  {
    std::cerr << "UDP server mock failed to receive data. Error msg: " << error.message() << std::endl;
    return;
  }

  if (bytes_received == 0)
  {
    std::cerr << __FUNCTION__ << ": Received UDP msg contained no data." << std::endl;
    return;
  }

  const psen_scan_v2_standalone::data_conversion_layer::RawData recv_data(recv_buffer_.cbegin(),
                                                                          recv_buffer_.cbegin() + bytes_received);
  new_msg_callback_(remote_endpoint_, recv_data);
  if (modi == ReceiveMode::continuous)
  {
    asyncReceive(ReceiveMode::continuous);
  }
}

void MockUDPServer::asyncReceive(const ReceiveMode& modi)
{
  io_service_.post([this, modi]() {
    socket_.async_receive_from(boost::asio::buffer(recv_buffer_),
                               remote_endpoint_,
                               boost::bind(&MockUDPServer::handleReceive,
                                           this,
                                           modi,
                                           boost::asio::placeholders::error,
                                           boost::asio::placeholders::bytes_transferred));
  });
}

}  // namespace psen_scan_v2_standalone_test
