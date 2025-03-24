#include "ros2_control_wheeled_robot_hardware/udp_wheeled_robot.hpp"

Eth_Socket::Eth_Socket()
{
  // Создание UDP-сокета
  sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock == -1) {
    perror("Socket creation failed");
    exit(EXIT_FAILURE);
  }

  // Настройка для приема данных
  memset(&cliaddr, 0, sizeof(cliaddr));
  cliaddr.sin_family = AF_INET;
  cliaddr.sin_port = htons(5011);
  cliaddr.sin_addr.s_addr = inet_addr("127.0.0.1");

  // Привязка сокета к адресу
  if (bind(sock, (struct sockaddr *)&cliaddr, sizeof(cliaddr)) < 0) {
    perror("Bind failed");
    close(sock);
    exit(EXIT_FAILURE);
  }

  // Настройка для отправки данных
  memset(&servaddr, 0, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_port = htons(5012);
  servaddr.sin_addr.s_addr = inet_addr("127.0.0.1");

  // Установка таймаута на прием данных
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 500000;  // 500 мс
  if (setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) < 0) {
    perror("Setsockopt failed");
  }
}

Eth_Socket::~Eth_Socket()
{
  if (close(sock) == -1) {
    perror("Socket close failed");
  }
}

void Eth_Socket::SendWheelSpeeds(double wheel_speeds[6])
{
  wheel_state_message wm;
  memcpy(wm.wheel_speeds, wheel_speeds, sizeof(double) * 6);
  wm.operating_mode = 0x01;

  if (sendto(sock, &wm, sizeof(wm), MSG_CONFIRM, 
      (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
    perror("Send failed");
  }
}

void Eth_Socket::GetWheelSpeeds(double wheel_speeds[6])
{
  double dummy_positions[6];
  GetWheelStates(wheel_speeds, dummy_positions);
}

void Eth_Socket::GetWheelStates(double wheel_speeds[6], double wheel_positions[6])
{
  wheel_state_message wm;
  socklen_t len = sizeof(cliaddr);
  
  int received = recvfrom(sock, &wm, sizeof(wm), MSG_WAITALL,
                     (struct sockaddr *)&cliaddr, &len);

  if (received == sizeof(wm)) {
    memcpy(wheel_speeds, wm.wheel_speeds, sizeof(wm.wheel_speeds));
    memcpy(wheel_positions, wm.wheel_positions, sizeof(wm.wheel_positions));
  } else {
    perror("Receive failed or incomplete data");
    memset(wheel_speeds, 0, sizeof(double) * 6);
    memset(wheel_positions, 0, sizeof(double) * 6);
  }
}