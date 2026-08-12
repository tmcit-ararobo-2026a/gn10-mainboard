#include "gn10_mainboard/robot_ethernet.hpp"

#include <cstring>

#include "gn10_mainboard/serial_printf.hpp"
#include "gpio.h"
#include "wiznet_ether/socket.hpp"
#include "wiznet_ether/w5500_spi.hpp"

bool RobotEthernet::init()
{
    // ハードウェア初期化
    if (W5500Init()) {
    } else {
        // 失敗したらW5500が利用できない
        HAL_GPIO_WritePin(LED_RAD_GPIO_Port, LED_RAD_Pin, GPIO_PIN_SET);
        return false;
    }
    uint8_t ver = getVERSIONR();  // または WIZCHIP_READ(VERSIONR);
    serial_printf("W5500 Version: 0x%02X\n", ver);

    // ネットワーク情報の設定
    wiz_NetInfo_t net_info = {
        .mac  = {0x48, 0x47, 0x85, 0xA3, 0x8B, 0xF2},
        .ip   = *robot_config::ip::mainboard,
        .sn   = {255, 255, 255, 0},
        .gw   = *robot_config::ip::pc_robot, // ロボット内部LANのゲートウェイはPC
        .dns  = {0, 0, 0, 0},
        .dhcp = NETINFO_STATIC
    };
    wizchip_setnetinfo(&net_info);

    // ネットワーク情報の確認
    wiz_NetInfo tmpNetInfo;
    wizchip_getnetinfo(&tmpNetInfo);
    serial_printf(
        "MAC: %02X:%02X:%02X:%02X:%02X:%02X\n",
        tmpNetInfo.mac[0],
        tmpNetInfo.mac[1],
        tmpNetInfo.mac[2],
        tmpNetInfo.mac[3],
        tmpNetInfo.mac[4],
        tmpNetInfo.mac[5]
    );
    serial_printf(
        "IP: %d.%d.%d.%d\n", tmpNetInfo.ip[0], tmpNetInfo.ip[1], tmpNetInfo.ip[2], tmpNetInfo.ip[3]
    );

    // 通信レイテンシ向上のため再送信回数を1回、タイムアウトを100msに設定
    setRCR(1);
    setRTR(100);

    // ソケット作成
    socket(socket_cmd_, Sn_MR_UDP, robot_config::port::cmd, SF_IO_NONBLOCK);
    socket(socket_teleop_, Sn_MR_UDP, robot_config::port::teleop, SF_IO_NONBLOCK);
    socket(socket_debug_, Sn_MR_UDP, robot_config::port::debug, SF_IO_NONBLOCK);

    return true;
}

bool RobotEthernet::receive_operation_data(robot_config::operation_t& data)
{
    robot_config::operation_u rx_data;
    uint8_t source_address[4];
    uint16_t source_port;

    int32_t ret = recvfrom(
        socket_cmd_, rx_data.binary, sizeof(robot_config::operation_u), source_address, &source_port
    );
    // データ整合性チェック
    if (ret != sizeof(robot_config::operation_u)) return false;
    if (rx_data.value.header != robot_config::header::operation) return false;
    // 送信元チェック
    if (std::memcmp(source_address, robot_config::ip::pc_robot, 4) != 0) return false;
    if (source_port != robot_config::port::cmd) return false;

    data = rx_data.value;
    return true;
}

bool RobotEthernet::send_feedback_data(const robot_config::feedback_t& data)
{
    robot_config::feedback_u tx_data;
    tx_data.value        = data;
    tx_data.value.header = robot_config::header::feedback;

    int32_t len = sendto(
        socket_cmd_,
        tx_data.binary,
        sizeof(tx_data),
        robot_config::ip::pc_robot,
        robot_config::port::cmd
    );

    if (len != sizeof(robot_config::feedback_u)) return false;
    return true;
}

bool RobotEthernet::send_pc_debug_data(const robot_config::debug_pc_t& data)
{
    robot_config::debug_pc_u tx_data;
    tx_data.value        = data;
    tx_data.value.header = robot_config::header::pc_debug;

    int32_t len = sendto(
        socket_debug_,
        tx_data.binary,
        sizeof(tx_data),
        robot_config::ip::pc_robot,
        robot_config::port::debug
    );

    if (len != sizeof(robot_config::debug_pc_u)) return false;
    return true;
}