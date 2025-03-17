#include "ros2_control_wheeled_robot_hardware/udp_wheeled_robot.hpp"

#include <iostream>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>

// Структура для передачи данных о скоростях колес
struct wheel_message {
  double wheel_speeds[6];  // Скорости колес (в радианах/секунду)
  uint8_t operating_mode;  // Режим работы
};

Eth_Socket::Eth_Socket()
{
  // Создание UDP-сокета
  sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock == -1) {
    perror("Socket creation failed");
    exit(EXIT_FAILURE);
  }

  // Настройка для приема данных с порта 5011 (127.0.0.1)
  bzero(&cliaddr, sizeof(cliaddr));
  cliaddr.sin_family = AF_INET;
  cliaddr.sin_port = htons(5011);
  cliaddr.sin_addr.s_addr = inet_addr("127.0.0.1");

  // Привязка сокета к адресу
  if (bind(sock, (struct sockaddr *)&cliaddr, sizeof(cliaddr)) < 0) {
    perror("Bind failed");
    close(sock);
    exit(EXIT_FAILURE);
  }

  // Настройка для отправки данных на порт 5012 (127.0.0.1)
  bzero(&servaddr, sizeof(servaddr));
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

// Функция отправки целевых скоростей колес
void Eth_Socket::SendWheelSpeeds(double wheel_speeds[6])
{
  wheel_message wm;
  memcpy(wm.wheel_speeds, wheel_speeds, sizeof(double) * 6);  // Исправлено
  wm.operating_mode = 0x01;  // Пример режима работы

  unsigned char w_pak[sizeof(wm)];
  memcpy(w_pak, &wm, sizeof(wm));

  if (sendto(sock, w_pak, sizeof(wm), MSG_CONFIRM, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
    perror("Send failed");
  }
}

// Функция получения текущих скоростей колес
void Eth_Socket::GetWheelSpeeds(double wheel_speeds[6])
{
  unsigned char r_pak[sizeof(wheel_message)];
  int received = recvfrom(sock, r_pak, sizeof(wheel_message), MSG_WAITALL, (struct sockaddr *)NULL, NULL);

  if (received > 0) {
    wheel_message wm = *reinterpret_cast<wheel_message *>(r_pak);
    memcpy(wheel_speeds, wm.wheel_speeds, sizeof(wm.wheel_speeds));
  } else {
    perror("Receive failed");
  }
}