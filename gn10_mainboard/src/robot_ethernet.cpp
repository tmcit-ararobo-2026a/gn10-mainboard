#include "gn10_mainboard/robot_ethernet.hpp"

#include "gn10_mainboard/serial_printf.hpp"
#include "gpio.h"
#include "wiznet_ether/socket.hpp"
#include "wiznet_ether/w5500_spi.hpp"

bool RobotEthernet::init()
{
    if (W5500Init()) {
    } else {
        HAL_GPIO_WritePin(LED_RAD_GPIO_Port, LED_RAD_Pin, GPIO_PIN_SET);
        return false;
    }
    uint8_t ver = getVERSIONR();  // または WIZCHIP_READ(VERSIONR);
    serial_printf("W5500 Version: 0x%02X\n", ver);

    wiz_NetInfo_t net_info = {
        .mac  = {0x48, 0x47, 0x85, 0xA3, 0x8B, 0xF2},
        .ip   = *robot_network_config::ip::mainboard,
        .sn   = {0xFF, 0xFF, 0xFF, 0},
        .gw   = *robot_network_config::ip::pc_robot,
        .dns  = *robot_network_config::ip::pc_robot,
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

    setRCR(1);
    setRTR(100);

    socket(socket_cmd_, Sn_MR_UDP, robot_network_config::port::cmd, SF_IO_NONBLOCK);
    setSn_CR(socket_cmd_, Sn_CR_RECV);
    if (getSn_SR(socket_cmd_) == SOCK_UDP) {
    } else {
        return false;
    }

    socket(socket_teleop_, Sn_MR_UDP, robot_network_config::port::teleop, SF_IO_NONBLOCK);
    setSn_CR(socket_teleop_, Sn_CR_RECV);
    if (getSn_SR(socket_teleop_) == SOCK_UDP) {
    } else {
        return false;
    }

    socket(socket_debug_, Sn_MR_UDP, robot_network_config::port::debug, SF_IO_NONBLOCK);
    setSn_CR(socket_debug_, Sn_CR_RECV);
    if (getSn_SR(socket_debug_) == SOCK_UDP) {
    } else {
        return false;
    }
    return true;
}

void RobotEthernet::send_feedback_data(feedback_data_t data)
{
    data.header          = feedback_data_header;
    feedback_union_.data = data;
    sendto(
        socket_cmd_,
        operation_union_.code,
        sizeof(operation_data_u),
        ethernet_config::pc::ip,
        ethernet_config::pc::port_feedback
    );
}

void RobotEthernet::send_pc_debug_data(pc_debug_t data)
{
    data.header       = 1;
    debug_union_.data = data;
    sendto(
        socket_debug_,
        debug_union_.code,
        sizeof(pc_debug_t),
        ethernet_config::pc::ip,
        ethernet_config::pc::port_debug
    );
}

bool RobotEthernet::receive_operation_data(operation_data_t& data)
{
    int32_t ret = recvfrom(
        socket_cmd_,
        operation_union_.code,
        sizeof(operation_data_u),
        ethernet_config::pc::ip,
        &ethernet_config::pc::port_operation
    );
    if (ret == sizeof(operation_data_u) && operation_union_.data.header == operation_data_header) {
        data = operation_union_.data;
        return true;
    }
    return false;
}