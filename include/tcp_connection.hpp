#ifndef __CEDES_TCPCONNECTION_H__
#define __CEDES_TCPCONNECTION_H__

#include <boost/asio.hpp> 

using boost::asio::ip::tcp;

namespace Cedes {

class TcpConnection {
public:
  enum State {
    STATE_CONNECTING,
    STATE_DISCONNECTED,
    STATE_CONNECTED,
    STATE_CLOSING,
    STATE_WAIT_ACK
  };

  TcpConnection(boost::asio::io_service& io_service)
    : resolver(io_service), socket(io_service), state(STATE_DISCONNECTED) {}

  void connect();
  void disconnect();
  void sendCommand(const std::vector<uint8_t> &);

  bool isConnected() const;
  bool isDisconnected() const;

private:
  static const int MARKER_SIZE = 4;
  static constexpr const char* PORT = "50660";
  static constexpr const char* HOST = "10.10.31.180";
  static constexpr const char* END_MARKER = "\xff\xff\x55\xaa";
  static constexpr const char* START_MARKER = "\xff\xff\xaa\x55";

  State state, previousState;
  tcp::socket socket;
  tcp::resolver resolver;
  boost::system::error_code error;

  void waitAck();
  void updateState(State);
  void revertState();
};
}
#endif // __CEDES_TCPCONNECTION_H__