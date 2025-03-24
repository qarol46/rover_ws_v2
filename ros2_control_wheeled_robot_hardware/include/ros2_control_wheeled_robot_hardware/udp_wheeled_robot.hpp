#ifndef UDP_WHEELED_ROBOT_HPP_
#define UDP_WHEELED_ROBOT_HPP_

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

// Структура для передачи данных о состояниях колес
struct wheel_state_message {
  double wheel_speeds[6];    // Скорости колес (рад/с)
  double wheel_positions[6]; // Позиции колес (рад)
  uint8_t operating_mode;    // Режим работы
};

class Eth_Socket
{
public:
  Eth_Socket();
  ~Eth_Socket();

  // Отправка целевых скоростей колес
  void SendWheelSpeeds(double wheel_speeds[6]);

  // Получение текущих скоростей колес (для обратной совместимости)
  void GetWheelSpeeds(double wheel_speeds[6]);

  // Получение текущих состояний (скоростей и позиций)
  void GetWheelStates(double wheel_speeds[6], double wheel_positions[6]);

private:
  int sock;  // Дескриптор сокета
  struct sockaddr_in cliaddr;  // Адрес для приема данных
  struct sockaddr_in servaddr;  // Адрес для отправки данных
};

#endif  // UDP_WHEELED_ROBOT_HPP_