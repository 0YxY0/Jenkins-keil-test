/********************************** (C) COPYRIGHT *******************************
* File Name          : CH395_CMD.H
* Author             : LHQ
* Version            : V1.1
* Date               : 2022/11/29
* Description        : CH395芯片命令接口文件,常用子程序外部调用
*******************************************************************************/

#ifndef __CH395_CMD_H__
#define __CH395_CMD_H__

#include <rtthread.h>
#include <rtdevice.h>

void ch395_init_dev(struct rt_spi_device* device);                                                                          /* 初始化 */

rt_size_t ch395_reset(void);                                                                                              /* 复位 */

uint8_t ch395_get_ver(void);                                                                                                /* 获取芯片及固件版本号 */

uint8_t ch395_check_exist(uint8_t testdata);                                                                                /* 测试命令 */

uint8_t ch395_get_phy_status(void);                                                                                         /* 获取PHY状态 */

uint8_t ch395_get_cmd_status(void);                                                                                         /* 获取命令执行状态 */

uint8_t ch395_init(void);                                                                                                   /* 初始化CH395 */

rt_size_t ch395_set_ip_addr(uint8_t *ipaddr, rt_size_t buff_size);                                                          /* 设置CH395的IP地址 */

rt_size_t ch395_set_gw_ip_addr(uint8_t *gwipaddr, rt_size_t buff_size);                                                     /* 设置CH395的网关IP地址 */

rt_size_t ch395_set_mask_addr(uint8_t *maskaddr, rt_size_t buff_size);                                                      /* 设置子网掩码 */

rt_err_t ch395_get_unreach_ippt(uint8_t *list, rt_size_t buff_size);                                                        /* 获取不可达地址以及端口 */

rt_err_t ch395_get_remote_ipp(uint8_t sockindex, uint8_t* list, rt_size_t buff_size);                                       /* 获取远端IP和端口，一般在TCP Server下使用 */

rt_size_t ch395_set_socket_prot_type(uint8_t sockindex,uint8_t prottype);                                                   /* 设置socket n的协议类型 */

rt_size_t ch395_set_socket_sour_port(uint8_t sockindex, uint16_t surprot);                                                  /* 设置socket n的源端口 */

rt_err_t ch395_send_data(uint8_t sockindex, uint8_t* databuf, uint16_t len);                                                /* 向socket n的发送缓冲区写数据 */

uint16_t ch395_get_recv_length(uint8_t sockindex);                                                                          /* 获取socket n的接收长度 */

rt_err_t ch395_get_recv_data(uint8_t sockindex, uint16_t len, uint8_t *pbuf);                                               /* 获取接收数据 */

rt_err_t ch395_get_socket_status(uint8_t sockindex, uint8_t *status) ;                                                      /* 获取socket n的状态 */

uint8_t  ch395_open_socket(uint8_t sockindex);                                                                              /* 打开socket n*/

uint8_t  ch395_close_socket(uint8_t sockindex);                                                                             /* 关闭socket n*/

uint8_t ch395_tcp_listen(uint8_t sockindex);                                                                                /* TCP监听 */

uint8_t ch395_get_socket_int(uint8_t sockindex);                                                                            /* 获取socket n的中断状态 */

uint8_t ch395_dhcp_enable(uint8_t flag);                                                                                    /* 使能DHCP */

uint8_t ch395_get_dhcp_status(void);                                                                                        /* 获取DHCP状态 */

rt_err_t ch395_get_ip_inf(uint8_t *addr, rt_size_t buf_size);                                                               /* 获取IP，子网掩码和网关地址   */

rt_size_t ch395_set_socket_recv_buf(uint8_t sockindex, uint8_t startblk, uint8_t blknum);                                   /* 设置Socket接收缓冲区 */

rt_size_t ch395_set_socket_send_buf(uint8_t sockindex, uint8_t startblk, uint8_t blknum);                                   /* 设置Socket发送缓冲区 */

rt_size_t ch395_set_start_para(uint32_t mdata);                                                                             /* 设置CH395启动参数  */

uint16_t ch395_get_glob_int_status_all(void);                                                                               /* 获取全局中断状态  */





/* 以下接口未经过测试，暂不对外提供 */

//rt_size_t ch395_sleep(void);                                                                                                /* 睡眠 */

//rt_size_t ch395_set_phy(uint8_t phystat);                                                                                   /* 设置PHY状态  */

//uint8_t ch395_get_glob_int_status(void);                                                                                    /* 获取CH395全局中断状态 */

//rt_size_t ch395_set_uart_baudrate(uint32_t baudrate);                                                                       /* 设置波特率 */

//rt_size_t ch395_set_mac_addr(uint8_t *macaddr, rt_size_t buff_size);                                                        /* 设置CH395的MAC地址 */

//rt_err_t ch395_get_mac_addr(uint8_t *macaddr, rt_size_t buff_size);                                                         /* 获取MAC地址 */

//rt_size_t ch395_set_mac_filt(uint8_t filtype,uint32_t table0, uint32_t table1);                                             /* 设置CH395的MAC过滤 */

//rt_size_t ch395_set_socket_des_ip(uint8_t sockindex, uint8_t *ipaddr, rt_size_t buff_size);                                 /* 设置scoket n的目的IP地址 */

//rt_size_t ch395_set_socket_des_Port(uint8_t sockindex, uint16_t desprot);                                                   /* 设置socket n的目的端口 */

//rt_size_t ch395_set_socket_ipraw_proto(uint8_t sockindex, uint8_t prototype);                                               /* 在IPRAW模式下，设置socket n的IP包协议字段 */

//rt_size_t ch395_enable_ping(uint8_t enable);                                                                                /* 使能ping */

//rt_size_t ch395_clear_recv_buf(uint8_t sockindex);                                                                          /* 清除socket n的接收缓冲区 */

//rt_size_t ch395_set_retry_count(uint8_t count);                                                                             /* 设置最大重试次数 */

//rt_size_t ch395_set_retry_period(uint16_t period);                                                                          /* 设置最大重试周期 单位 毫秒*/

//uint8_t ch395_tcp_connect(uint8_t sockindex);                                                                               /* TCP连接 */

//uint8_t ch395_tcp_disconnect(uint8_t sockindex);                                                                            /* TCP断开连接 */

//uint8_t ch395_crc_ret_6bit(uint8_t *mac_addr);                                                                              /* 多播地址CRC32，用于HASH过滤 */

//rt_size_t ch395_write_gpio_addr(uint8_t regadd, uint8_t regval);                                                            /* 写GPIO寄存器 */

//uint8_t ch395_read_gpio_addr(uint8_t regadd);                                                                               /* 读GPIO寄存器 */

//uint8_t ch395_eeprom_erase(void);                                                                                           /* 擦除EEPROM */

//uint8_t ch395_eeprom_write(uint16_t eepaddr, uint8_t* buf, uint8_t len);                                                    /* 写EEPROM */

//rt_err_t ch395_eeprom_eead(uint16_t eepaddr, uint8_t* buf, uint8_t len);                                                    /* 读EEPROM */

//rt_size_t ch395_set_tcp_mss(uint16_t tcpmss);                                                                               /* 设置TCP MSS值 */

//void ch395_udp_send_To(uint8_t* buf,uint32_t len, uint8_t* ip, rt_size_t ip_size, uint16_t port, uint8_t sockindex);        /* UDP向指定的IP和端口发送数据 */

#endif
/**************************** endfile *************************************/
