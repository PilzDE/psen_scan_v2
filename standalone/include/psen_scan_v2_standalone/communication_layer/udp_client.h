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
#ifndef PSEN_SCAN_V2_STANDALONE_UDP_CLIENT_H
#define PSEN_SCAN_V2_STANDALONE_UDP_CLIENT_H

#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <future>

#ifdef __linux__
#include <arpa/inet.h>
#endif

#ifdef _WIN32
#include <WinSock2.h>
#endif

#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include "psen_scan_v2_standalone/data_conversion_layer/raw_scanner_data.h"
#include "psen_scan_v2_standalone/util/logging.h"
#include "psen_scan_v2_standalone/util/timestamp.h"

namespace psen_scan_v2_standalone
{
/**
 * @brief Namespace for the primitive communication layer implementation.
 */
namespace communication_layer
{
using NewMessageCallback =
    std::function<void(const data_conversion_layer::RawDataConstPtr&, const std::size_t&, const int64_t& timestamp)>;
using ErrorCallback = std::function<void(const std::string&)>;

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

/**
 * @brief Helper for asynchronously sending and receiving data via UDP.
 *
 * This class provides methods for opening and closing the UDP connection, writing and receiving data via UDP. The data
 * is represented by @ref data_conversion_layer::RawData. Upon receiving data a @ref NewMessageCallback is invoked. When
 * something goes wrong an
 * @ref ErrorCallback is invoked.
 *
 * The ScannerV2 constructs two UDP clients, which are then used by the scanner_protocol::ScannerProtocolDef.
 */
class UdpClientImpl
{
public:
  /**
   * @brief Exception thrown if the UDP socket cannot be closed.
   */
  class CloseConnectionFailure : public std::runtime_error
  {
  public:
    CloseConnectionFailure(const std::string& msg = "Failure while closing connection");
  };

  /**
   * @brief Exception thrown if the UDP socket cannot be opened.
   */
  class OpenConnectionFailure : public std::runtime_error
  {
  public:
    OpenConnectionFailure(const std::string& msg = "Failure while opening connection");
  };

public:
  /**
   * @brief Opens an UDP connection.
   *
   * @note The client does not start listing for new messages until explicitly triggered via
   * startAsyncReceiving().
   *
   * @param msg_callback Callback called whenever new messages are received.
   * @param error_callback Handler called whenever something wents wrong while receiving data.
   * @param host_port Port from which data are sent and received.
   * @param endpoint_ip IP address of the endpoint from which data are received and sent too.
   * @param endpoint_port Port on which the other endpoint is sending and receiving data.
   */
  UdpClientImpl(const NewMessageCallback& msg_callback,
                const ErrorCallback& error_callback,
                const unsigned short& host_port,
                const unsigned int& endpoint_ip,
                const unsigned short& endpoint_port);

  /**
   * @brief Closes the UDP connection and stops all pending asynchronous operation.
   */
  ~UdpClientImpl();

public:
  /**
   * @brief Starts an asynchronous process listing to (a) new message(s) from the other endpoint. If the function
   * continuously listens to new messages or only listens for one message is specified by the receive modi specified.
   * - If a new message is received, the message callback is called.
   * - If an error occurs while receiving data, the error callback is called.
   *
   * @param modi Specifies if the function continuously listens to new messages or or not.
   */
  void startAsyncReceiving(const ReceiveMode& modi = ReceiveMode::continuous);

  /**
   * @brief Asynchronously sends the specified data to the other endpoint.
   *
   * @param data Data which have to be send to the other endpoint.
   */
  void write(const data_conversion_layer::RawData& data);

  /**
   * @brief Stops the underlying io_service so that no messages are received anymore.
   *
   * Note: In contrary to close() this is non-blocking but note that after calling stop()
   * calls to the msg_callback are still possible due to pending messages.
   */
  void stop();

  /**
   * @brief Closes the UDP connection and stops all pending asynchronous operation.
   */
  void close();

  /**
   * @brief Returns local ip address of current socket connection.
   * deprecated: use boost::asio::ip::address_v4 hostIp() instead
   */
  [[deprecated("use boost::asio::ip::address_v4 hostIp() instead")]] boost::asio::ip::address_v4 getHostIp();

  /**
   * @brief Returns local ip address of current socket connection.
   */
  boost::asio::ip::address_v4 hostIp();

private:
  void asyncReceive(const ReceiveMode& modi);

  void sendCompleteHandler(const boost::system::error_code& error, std::size_t bytes_transferred);

private:
  boost::asio::io_service io_service_;
  // Prevent the run() method of the io_service from returning when there is no more work.
  boost::asio::io_service::work work_{ io_service_ };
  std::thread io_service_thread_;

  data_conversion_layer::RawDataPtr received_data_;

  NewMessageCallback message_callback_;
  ErrorCallback error_callback_;

  boost::asio::ip::udp::socket socket_;
  boost::asio::ip::udp::endpoint endpoint_;
};

inline communication_layer::UdpClientImpl::UdpClientImpl(const NewMessageCallback& message_callback,
                                                         const ErrorCallback& error_callback,
                                                         const unsigned short& host_port,
                                                         const unsigned int& endpoint_ip,
                                                         const unsigned short& endpoint_port)
  : message_callback_(message_callback)
  , error_callback_(error_callback)
  , socket_(io_service_, boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), host_port))
  , endpoint_(boost::asio::ip::address_v4(endpoint_ip), endpoint_port)
{
  if (!message_callback)
  {
    throw std::invalid_argument("New message callback is invalid");
  }

  if (!error_callback)
  {
    throw std::invalid_argument("Error callback is invalid");
  }

  received_data_.reset(new data_conversion_layer::RawData(data_conversion_layer::MAX_UDP_PAKET_SIZE));
  try
  {
    socket_.connect(endpoint_);
  }
  // LCOV_EXCL_START
  // No coverage check because testing the socket is not the objective here.
  catch (const boost::system::system_error& ex)
  {
    throw OpenConnectionFailure(ex.what());
  }
  // LCOV_EXCL_STOP

  assert(!io_service_thread_.joinable() && "io_service_thread_ is joinable!");
  io_service_thread_ = std::thread([this]() { io_service_.run(); });
}

inline void UdpClientImpl::stop()
{
  io_service_.stop();
}

inline void UdpClientImpl::close()
{
  io_service_.stop();
  if (io_service_thread_.joinable())
  {
    io_service_thread_.join();
  }

  try
  {
    // Function is intended to be called from the main thread,
    // therefore, the following socket operation happens on the main thread.
    // To avoid concurrency issues, it has to be ensured that the io_service thread
    // is finished before the socket close operation is performed.
    socket_.close();
  }
  // LCOV_EXCL_START
  // No coverage check because testing the socket is not the objective here.
  catch (const boost::system::system_error& ex)
  {
    throw CloseConnectionFailure(ex.what());
  }
  // LCOV_EXCL_STOP
}

inline boost::asio::ip::address_v4 UdpClientImpl::hostIp()
{
  return socket_.local_endpoint().address().to_v4();
}

inline boost::asio::ip::address_v4 UdpClientImpl::getHostIp()
{
  return this->hostIp();
}

inline UdpClientImpl::~UdpClientImpl()
{
  try
  {
    close();
  }
  // LCOV_EXCL_START
  // No coverage check because testing the socket is not the objective here.
  catch (const CloseConnectionFailure& ex)
  {
    PSENSCAN_ERROR("UdpClient", ex.what());
  }
  // LCOV_EXCL_STOP
}

inline void UdpClientImpl::sendCompleteHandler(const boost::system::error_code& error, std::size_t bytes_transferred)
{
  // LCOV_EXCL_START
  // No coverage check because testing the if-loop is extremly difficult.
  if (error || bytes_transferred == 0)
  {
    PSENSCAN_ERROR("UdpClient", "Failed to send data. Error message: {}", error.message());
  }

  // LCOV_EXCL_STOP
  PSENSCAN_DEBUG("UdpClient", "Data successfully send.");
}

inline void UdpClientImpl::write(const data_conversion_layer::RawData& data)
{
  io_service_.post([this, data]() {
    socket_.async_send(boost::asio::buffer(data.data(), data.size()),
                       boost::bind(&UdpClientImpl::sendCompleteHandler,
                                   this,
                                   boost::asio::placeholders::error,
                                   boost::asio::placeholders::bytes_transferred));
  });
}

inline void UdpClientImpl::startAsyncReceiving(const ReceiveMode& modi)
{
  std::promise<void> post_done_barrier;
  const auto post_done_future{ post_done_barrier.get_future() };
  // Function is intended to be called from main thread.
  // To ensure that socket operations only happen on one strand (in this case an implicit one),
  // the asyncReceive() operation is scheduled as task to the io_service thread.
  io_service_.post([this, modi, &post_done_barrier]() {
    asyncReceive(modi);
    post_done_barrier.set_value();
  });
  post_done_future.wait();
}

inline void UdpClientImpl::asyncReceive(const ReceiveMode& modi)
{
  socket_.async_receive(boost::asio::buffer(*received_data_, received_data_->size()),
                        [this, modi](const boost::system::error_code& error_code, const std::size_t& bytes_received) {
                          if (error_code || bytes_received == 0)
                          {
                            error_callback_(error_code.message());
                          }
                          else
                          {
                            message_callback_(received_data_, bytes_received, util::getCurrentTime());
                          }
                          if (modi == ReceiveMode::continuous)
                          {
                            asyncReceive(modi);
                          }
                        });
}

inline UdpClientImpl::OpenConnectionFailure::OpenConnectionFailure(const std::string& msg) : std::runtime_error(msg)
{
}

inline UdpClientImpl::CloseConnectionFailure::CloseConnectionFailure(const std::string& msg) : std::runtime_error(msg)
{
}
}  // namespace communication_layer
}  // namespace psen_scan_v2_standalone
#endif  // PSEN_SCAN_V2_STANDALONE_UDP_CLIENT_H
