#include <iostream>
#include <boost/asio.hpp> 

#include "tcp_connection.hpp"

using boost::asio::ip::tcp;

namespace Cedes {

void TcpConnection::connect() {
  if (isConnected()) return;

  updateState(STATE_CONNECTING);
  try {
    tcp::resolver::query query(HOST, PORT);
    tcp::resolver::iterator endpoint_iterator = resolver.resolve(query);
    tcp::resolver::iterator end;

    error = boost::asio::error::host_not_found; 
    while (error && endpoint_iterator != end) {
      socket.close();
      socket.connect(*endpoint_iterator++, error);
    }
    if (error) {
      throw::boost::system::system_error(error);
    }
    updateState(STATE_CONNECTED);
  } catch (std::exception& e) {
    revertState();
    std::cerr << e.what() << std::endl;
  }  
}

void TcpConnection::disconnect() {
  if (isDisconnected()) return;
  State prev_state = state;
  updateState(STATE_CLOSING);
  try {
    socket.close(); 
    updateState(STATE_DISCONNECTED);
  } catch (std::exception& e) {
    std::cerr << e.what() << std::endl;
    revertState();
  }
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

  socket.write_some(boost::asio::buffer(s, s.size()), error);
  if (error) {
    throw boost::system::system_error(error);
  }
  waitAck();
}

void TcpConnection::waitAck() {
  std::vector<uint8_t> buf(128);

  updateState(STATE_WAIT_ACK);
  size_t len = socket.read_some(boost::asio::buffer(buf), error);
  if (error) {
    throw boost::system::system_error(error);
  }
  revertState();
}

void TcpConnection::updateState(State state_) {
  previousState = state;
  state = state_;
}

void TcpConnection::revertState() {
  state = previousState;
}

}
