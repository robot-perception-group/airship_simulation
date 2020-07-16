// Copyright (c) 2003-2012 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)


#ifndef NEURAL_NETWORK_DETECTOR_BOOSTTCPCLIENT_H
#define NEURAL_NETWORK_DETECTOR_BOOSTTCPCLIENT_H

#include <boost/asio/connect.hpp>
#include <boost/asio/deadline_timer.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/read_until.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/streambuf.hpp>
#include <boost/system/system_error.hpp>
#include <boost/asio/write.hpp>
#include <cstdlib>
#include <iostream>
#include <string>
#include <boost/lambda/bind.hpp>
#include <boost/lambda/lambda.hpp>

using boost::asio::deadline_timer;
using boost::asio::ip::tcp;
using boost::lambda::bind;
using boost::lambda::var;

//----------------------------------------------------------------------

//
// This class manages socket timeouts by applying the concept of a deadline.
// Each asynchronous operation is given a deadline by which it must complete.
// Deadlines are enforced by an "actor" that persists for the lifetime of the
// client object:
//
//  +----------------+
//  |                |
//  | check_deadline |<---+
//  |                |    |
//  +----------------+    | async_wait()
//              |         |
//              +---------+
//
// If the actor determines that the deadline has expired, the socket is closed
// and any outstanding operations are consequently cancelled. The socket
// operations themselves use boost::lambda function objects as completion
// handlers. For a given socket operation, the client object runs the
// io_service to block thread execution until the actor completes.
//
class BoostTCPClient
{
public:
  BoostTCPClient()
    : socket_(io_service_),
      deadline_(io_service_)
  {
    // No deadline is required until the first socket operation is started. We
    // set the deadline to positive infinity so that the actor takes no action
    // until a specific deadline is set.
    deadline_.expires_at(boost::posix_time::pos_infin);

    // Start the persistent actor that checks for deadline expiry.
    check_deadline();
  }

  void connect(const std::string& host, const std::string& service,
               boost::posix_time::time_duration timeout)
  {
    // Resolve the host name and service to a list of endpoints.
    tcp::resolver::query query(host, service);
    tcp::resolver::iterator iter = tcp::resolver(io_service_).resolve(query);

    // Set a deadline for the asynchronous operation. As a host name may
    // resolve to multiple endpoints, this function uses the composed operation
    // async_connect. The deadline applies to the entire operation, rather than
    // individual connection attempts.
    deadline_.expires_from_now(timeout);

    // Set up the variable that receives the result of the asynchronous
    // operation. The error code is set to would_block to signal that the
    // operation is incomplete. Asio guarantees that its asynchronous
    // operations will never fail with would_block, so any other value in
    // ec indicates completion.
    boost::system::error_code ec = boost::asio::error::would_block;

    // Start the asynchronous operation itself. The boost::lambda function
    // object is used as a callback and will update the ec variable when the
    // operation completes. The blocking_udp_client.cpp example shows how you
    // can use boost::bind rather than boost::lambda.
    boost::asio::async_connect(socket_, iter, var(ec) = boost::lambda::_1);

    // Block until the asynchronous operation has completed.
    do io_service_.run_one(); while (ec == boost::asio::error::would_block);

    // Determine whether a connection was successfully established. The
    // deadline actor may have had a chance to run and close our socket, even
    // though the connect operation notionally succeeded. Therefore we must
    // check whether the socket is still open before deciding if we succeeded
    // or failed.
    if (ec || !socket_.is_open())
      throw boost::system::system_error(
        ec ? ec : boost::asio::error::operation_aborted);
  }

  std::string read_line(boost::posix_time::time_duration timeout)
  {
    // Set a deadline for the asynchronous operation. Since this function uses
    // a composed operation (async_read_until), the deadline applies to the
    // entire operation, rather than individual reads from the socket.
    deadline_.expires_from_now(timeout);

    // Set up the variable that receives the result of the asynchronous
    // operation. The error code is set to would_block to signal that the
    // operation is incomplete. Asio guarantees that its asynchronous
    // operations will never fail with would_block, so any other value in
    // ec indicates completion.
    boost::system::error_code ec = boost::asio::error::would_block;

    // Start the asynchronous operation itself. The boost::lambda function
    // object is used as a callback and will update the ec variable when the
    // operation completes. The blocking_udp_client.cpp example shows how you
    // can use boost::bind rather than boost::lambda.
    boost::asio::async_read_until(socket_, input_buffer_, '\n', var(ec) = boost::lambda::_1);

    // Block until the asynchronous operation has completed.
    do io_service_.run_one(); while (ec == boost::asio::error::would_block);

    if (ec)
      throw boost::system::system_error(ec);

    std::string line;
    std::istream is(&input_buffer_);
    std::getline(is, line);
    return line;
  }

  uint8_t* read_bytes(size_t length, boost::posix_time::time_duration timeout)
  {
    uint8_t* data = new uint8_t[length];
    deadline_.expires_from_now(timeout);
    boost::system::error_code ec = boost::asio::error::would_block;
    boost::asio::async_read(socket_, boost::asio::buffer(data, length), var(ec) = boost::lambda::_1);
    do io_service_.run_one(); while (ec == boost::asio::error::would_block);

    if (ec)
      throw boost::system::system_error(ec);

    return data;
  }

  uint8_t* read_bytes(uint8_t* data, size_t length, boost::posix_time::time_duration timeout)
  {
    if (!data) return 0;
    deadline_.expires_from_now(timeout);
    boost::system::error_code ec = boost::asio::error::would_block;
    boost::asio::async_read(socket_, boost::asio::buffer(data, length), var(ec) = boost::lambda::_1);
    do io_service_.run_one(); while (ec == boost::asio::error::would_block);

    if (ec)
      throw boost::system::system_error(ec);

    return data;
  }

  void write_line(const std::string& line,
                  boost::posix_time::time_duration timeout)
  {
    std::string data = line + "\n";

    // Set a deadline for the asynchronous operation. Since this function uses
    // a composed operation (async_write), the deadline applies to the entire
    // operation, rather than individual writes to the socket.
    deadline_.expires_from_now(timeout);

    // Set up the variable that receives the result of the asynchronous
    // operation. The error code is set to would_block to signal that the
    // operation is incomplete. Asio guarantees that its asynchronous
    // operations will never fail with would_block, so any other value in
    // ec indicates completion.
    boost::system::error_code ec = boost::asio::error::would_block;

    // Start the asynchronous operation itself. The boost::lambda function
    // object is used as a callback and will update the ec variable when the
    // operation completes. The blocking_udp_client.cpp example shows how you
    // can use boost::bind rather than boost::lambda.
    boost::asio::async_write(socket_, boost::asio::buffer(data), var(ec) = boost::lambda::_1);

    // Block until the asynchronous operation has completed.
    do io_service_.run_one(); while (ec == boost::asio::error::would_block);

    if (ec)
      throw boost::system::system_error(ec);
  }

  void write_bytes(const uint8_t* data, const size_t length,
                   boost::posix_time::time_duration timeout)
  {
    deadline_.expires_from_now(timeout);
    boost::system::error_code ec = boost::asio::error::would_block;
    boost::asio::async_write(socket_, boost::asio::buffer(data, length), var(ec) = boost::lambda::_1);

    do io_service_.run_one(); while (ec == boost::asio::error::would_block);

    if (ec)
      throw boost::system::system_error(ec);
  }

private:
  void check_deadline()
  {
    // Check whether the deadline has passed. We compare the deadline against
    // the current time since a new asynchronous operation may have moved the
    // deadline before this actor had a chance to run.
    if (deadline_.expires_at() <= deadline_timer::traits_type::now())
    {
      // The deadline has passed. The socket is closed so that any outstanding
      // asynchronous operations are cancelled. This allows the blocked
      // connect(), read_line() or write_line() functions to return.
      boost::system::error_code ignored_ec;
      socket_.close(ignored_ec);

      // There is no longer an active deadline. The expiry is set to positive
      // infinity so that the actor takes no action until a new deadline is set.
      deadline_.expires_at(boost::posix_time::pos_infin);
    }

    // Put the actor back to sleep.
    deadline_.async_wait(bind(&BoostTCPClient::check_deadline, this));
  }

  boost::asio::io_service io_service_;
  tcp::socket socket_;
  deadline_timer deadline_;
  boost::asio::streambuf input_buffer_;
};

#endif //NEURAL_NETWORK_DETECTOR_BOOSTTCPCLIENT_H
