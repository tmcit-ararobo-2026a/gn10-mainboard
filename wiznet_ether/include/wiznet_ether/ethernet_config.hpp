#pragma once
#include "wiznet_ether/wizchip_conf.hpp"

namespace ethernet_config {
namespace main_board {
wiz_NetInfo netInfo = {
    .mac  = {0x99, 0x99, 0x99, 0x99, 0x99, 0x99}, // MACアドレス
    .ip   = {192, 168, 3, 100}, // IPアドレス
    .sn   = {255, 255, 255, 0}, // サブネットマスク
    .gw   = {192, 168, 3, 1}, // ゲートウェイIPアドレス
    .dns  = {192, 168, 3, 1}, // DNSサーバIPアドレス
    .dhcp = NETINFO_STATIC  // DHCPモード（静的）
};
uint16_t port_operation = 5000;  // 操作用ポート番号
uint16_t port_feedback  = 5001;  // フィードバック用ポート番号
}  // namespace main_board

namespace pc {
uint8_t ip[4]                 = {192, 168, 3, 99};  // PCのIPアドレス
uint8_t ip_wifi[4]            = {192, 168, 2, 99};  // PCのIPアドレス
uint16_t port_operation       = 5000;               // PCの操作用ポート番号
uint16_t port_feedback        = 5001;               // PCのフィードバック用ポート番号
uint16_t port_controller_wifi = 5002;               // コントローラー用ポート番号 (WiFi)
}  // namespace pc

namespace controller {
wiz_NetInfo netInfo = {
    .mac  = {0x66, 0x66, 0x66, 0x66, 0x66, 0x66}, // MACアドレス
    .ip   = {192, 168, 2, 111}, // IPアドレス
    .sn   = {255, 255, 255, 0}, // サブネットマスク
    .gw   = {192, 168, 2, 1}, // ゲートウェイIPアドレス
    .dns  = {192, 168, 2, 1}, // DNSサーバIPアドレス
    .dhcp = NETINFO_STATIC  // DHCPモード（静的）
};

uint16_t port_controller = 5002;  // コントローラー用ポート
}  // namespace controller
}  // namespace ethernet_config