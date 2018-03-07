#include <iostream>
#include <boost/asio.hpp> 

#include "tcp_connection.hpp"

using boost::asio::ip::tcp;

namespace Cedes {

typedef std::vector<uint8_t> Packet;

TcpConnection::TcpConnection(boost::asio::io_service& ioService)
  : resolver(ioService), socket(ioService), state(STATE_DISCONNECTED) {
  connect();
}

TcpConnection::~TcpConnection() {
  try {
    disconnect();
  } catch (boost::system::system_error e) {
    std::cerr << e.what() << std::endl;
  }
}

void TcpConnection::connect() {
  if (isConnected()) return;

  updateState(STATE_CONNECTING);
  tcp::resolver::query query(HOST, PORT);
  tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
  tcp::resolver::iterator end;

  boost::system::error_code error = boost::asio::error::host_not_found; 
  while (error && endpoint_iterator != end) {
    socket.close();
    socket.connect(*endpoint_iterator++, error);
  }
  if (error) {
    throw::boost::system::system_error(error);
  }
  updateState(STATE_CONNECTED);
}

void TcpConnection::disconnect() {
  if (isDisconnected()) return;

  updateState(STATE_CLOSING);

  boost::system::error_code error;
  socket.shutdown(boost::asio::ip::tcp::socket::shutdown_both, error);  
  if (error) {
    revertState();
    throw boost::system::system_error(error);
  }
  updateState(STATE_DISCONNECTED);
}

bool TcpConnection::isConnected() const {
  return state == STATE_CONNECTED;
}

bool TcpConnection::isDisconnected() const {
  return state == STATE_DISCONNECTED;
}

void TcpConnection::sendCommand(const std::vector<uint8_t>& data) {

  if (!isConnected()) return;

  uint32_t data_len = data.size();
  size_t buf_size = MARKER_SIZE + sizeof(data_len) + data_len + MARKER_SIZE;
  
  boost::asio::streambuf buf;
  std::ostream os(&buf);

  os << START_MARKER;
  os << static_cast<uint8_t>((data_len & 0xff000000) >> 24);
  os << static_cast<uint8_t>((data_len & 0x00ff0000) >> 16);
  os << static_cast<uint8_t>((data_len & 0x0000ff00) >> 8);
  os << static_cast<uint8_t>((data_len & 0x000000ff) >> 0);
  for (int i = 0; i < data_len; ++i) {
    os << static_cast<uint8_t>(data[i]);
  }
  os << END_MARKER;

  std::istream is(&buf);
  std::string s;
  is >> s;
  buf.consume(buf_size);

  boost::system::error_code error;
  socket.write_some(boost::asio::buffer(s, s.size()), error);
  if (error) {
    throw boost::system::system_error(error);
  }
  waitAck();
}

void TcpConnection::waitAck() {
  Packet buf(128);
  boost::system::error_code error;

  this->updateState(STATE_WAIT_ACK);
  size_t len = socket.read_some(boost::asio::buffer(buf), error);
  if (error) {
    throw boost::system::system_error(error);
  }
  this->revertState();
}

void TcpConnection::updateState(State state_) const {
  previousState = state;
  state = state_;
}

void TcpConnection::revertState() const {
  state = previousState;
}
}
