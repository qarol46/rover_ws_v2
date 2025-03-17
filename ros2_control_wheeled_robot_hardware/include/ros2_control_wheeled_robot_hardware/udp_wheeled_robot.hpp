#ifndef UDP_WHEELED_ROBOT_HPP_
#define UDP_WHEELED_ROBOT_HPP_

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

class Eth_Socket
{
public:
  Eth_Socket();
  ~Eth_Socket();

  // Отправка целевых скоростей колес
  void SendWheelSpeeds(double wheel_speeds[6]);

  // Получение текущих скоростей колес
  void GetWheelSpeeds(double wheel_speeds[6]);

private:
  int sock;  // Дескриптор сокета
  struct sockaddr_in cliaddr;  // Адрес для приема данных
  struct sockaddr_in servaddr;  // Адрес для отправки данных
};

#endif  // UDP_WHEELED_ROBOT_HPP_