/********************************** (C) COPYRIGHT *******************************
* File Name          : CH395CMD.C
* Author             : WCH
* Version            : V1.1
* Date               : 2014/8/1
* Description        : CH395芯片命令接口文件
*
*******************************************************************************/

/* 头文件包含*/
#include "ch395_inc.h"
#include "ch395_cmd.h"

struct rt_spi_device* spi_dev = RT_NULL;

/********************************************************************************
* Function Name  : ch395_init_dev
* Description    : 初始化ch395设备句柄
* Input          : device--SPI设备句柄
* Output         : None
* Return         : None
*******************************************************************************/
void ch395_init_dev(struct rt_spi_device* device)
{
    RT_ASSERT(device != RT_NULL);

    spi_dev = device;
}

/********************************************************************************
* Function Name  : ch395_reset
* Description    : 复位CH395芯片
* Input          : None
* Output         : None
* Return         : 0--设置失败   非0--设置成功
*******************************************************************************/
rt_size_t ch395_reset(void)
{
    rt_size_t val;
    uint8_t reset = CMD00_RESET_ALL;

    /* 发送复位命令 */
    val = rt_spi_send(spi_dev, &reset, 1);

    return val;
}

/*******************************************************************************
* Function Name  : ch395_sleep
* Description    : 使CH395进入睡眠状态
* Input          : None
* Output         : None
* Return         : 0--设置失败   非0--设置成功
*******************************************************************************/
rt_size_t ch395_sleep(void)
{
    rt_size_t val;
    uint8_t sleep = CMD00_ENTER_SLEEP;

    /* 发送睡眠命令 */
    val = rt_spi_send(spi_dev, &sleep, 1);

    return val;
}

/********************************************************************************
* Function Name  : ch395_get_ver
* Description    : 获取芯片以及固件版本号，1字节，高四位表示芯片版本，低四位表示固件版本
* Input          : None
* Output         : None
* Return         : 1字节芯片及固件版本号--成功   RT_NULL--失败
*******************************************************************************/
uint8_t ch395_get_ver(void)
{
    uint8_t ch395_id = CMD01_GET_IC_VER, rev_id = 0;
    rt_err_t ret;

    ret = rt_spi_send_then_recv(spi_dev, &ch395_id, 1, &rev_id, 1);

    if(ret == RT_EOK)
    {
       return rev_id;
    }
   else{
       return RT_NULL;
   }
}

/********************************************************************************
* Function Name  : ch395_check_exist
* Description    : 测试命令，用于测试通讯接口和工作状态
* Input          : testdata--1字节测试数据
* Output         : None
* Return         : 硬件OK，返回 testdata按位取反
*******************************************************************************/
uint8_t ch395_check_exist(uint8_t testdata)
{
    uint8_t check_exist[2] = {CMD11_CHECK_EXIST, testdata};
    uint8_t ret_data = 0;
    rt_err_t ret;

    ret = rt_spi_send_then_recv(spi_dev, check_exist, 2, &ret_data, 1);

    if(ret == RT_EOK)
    {
       return ret_data;
    }
   else{
       return RT_NULL;
   }
}

/********************************************************************************
* Function Name  : ch395_set_phy
* Description    : 设置CH395以太网PHY的连接方式，默认为自动协商方式
* Input          : phystat--1字节连接状态代码
                   01H 时表示 PHY 连接断开
                   02H 时表示 PHY 连接为 10M 全双工
                   04H 时表示 PHY 连接为 10M 半双工
                   08H 时表示 PHY 连接为 100M 全双工
                   10H 时表示 PHY 连接为 100M 半双工
* Notice         : CH395 收到此命令后会复位 MAC 和 PHY，按照新设置的连接方式重新进行连接,
                                                如果以太网已经连接，则会断开并重新连接
* Output         : None
* Return         : 0--设置失败   非0--设置成功
*******************************************************************************/
rt_size_t ch395_set_phy(uint8_t phystat)
{
    if(phystat != 0x01 && phystat != 0x02 && phystat != 0x04 && phystat != 0x08 && phystat != 0x10)
    {
        rt_kprintf("set PHY error!\r\n");
        return 0;
    }

    uint8_t sead_data[2] = {CMD10_SET_PHY, phystat};
    rt_size_t ret;

    ret = rt_spi_send(spi_dev, &sead_data, 2);

    return ret;
}

/*******************************************************************************
* Function Name  : ch395_get_phy_status
* Description    : 获取PHY的状态
* Input          : None
* Output         : None
* Return         : 当前CH395PHY状态，参考PHY参数/状态定义
*******************************************************************************/
uint8_t ch395_get_phy_status(void)
{
    uint8_t cmd = CMD01_GET_PHY_STATUS, ret_data = 0;
    rt_err_t ret;

    ret = rt_spi_send_then_recv(spi_dev, &cmd, 1, &ret_data, 1);

    if(ret == RT_EOK)
    {
       return ret_data;
    }
   else{
       return RT_NULL;
   }
}

/*******************************************************************************
* Function Name  : ch395_get_glob_int_status
* Description    : 获取全局中断状态，收到此命令CH395自动取消中断，0x43及以下版本使用
* Input          : None
* Output         : None
* Return         : 返回当前的全局中断状态
*******************************************************************************/
uint8_t ch395_get_glob_int_status(void)
{
    uint8_t cmd = CMD01_GET_GLOB_INT_STATUS, init_status = 0;
    rt_err_t ret;

    ret = rt_spi_send_then_recv(spi_dev, &cmd, 1, &init_status, 1);

    if(ret == RT_EOK)
    {
       return init_status;
    }
   else{
       return RT_NULL;
   }
}

/*******************************************************************************
* Function Name  : ch395_get_cmd_status
* Description    : 获取命令执行状态，某些命令需要等待命令执行结果
* Input          : None
* Output         : None
* Return         : 返回上一条命令执行状态
*******************************************************************************/
uint8_t ch395_get_cmd_status(void)
{
    uint8_t cmd = CMD01_GET_CMD_STATUS, prv_status = 0;
    rt_err_t ret;

    ret = rt_spi_send_then_recv(spi_dev, &cmd, 1, &prv_status, 1);

    if(ret == RT_EOK)
    {
       return prv_status;
    }
   else{
       return RT_NULL;
   }
}

/********************************************************************************
* Function Name  : ch395_init
* Description    : 初始化CH395芯片,包括初始化 CH395 的 MAC，PHY 以及 TCP/IP 协议栈
                                               该命令约350ms执行完毕，可通过发送CMD01_GET_CMD_STATUS来查询是否执行完毕和执行状态
* Input          : None
* Output         : None
* Return         : 返回执行结果
*******************************************************************************/
uint8_t ch395_init(void)
{
    uint8_t cmd = CMD0W_INIT_CH395 , status, count = 0;
    rt_size_t ret;

    ret = rt_spi_send(spi_dev, &cmd, 1);

    if(ret > 0)
    {
        while(1)
        {
           rt_thread_mdelay(10);                        /* 延时查询，建议2ms以上*/
           status = ch395_get_cmd_status();                /* 不能过于频繁查询*/

           if(status !=CH395_ERR_BUSY)                  /* 如果CH395芯片返回忙状态*/
           {
               break;
           }

           if(count++ > 200)
           {
               return CH395_ERR_UNKNOW;                     /* 超时退出,本函数需要500ms以上执行完毕 */
           }
        }

        return status;
    }
    else{
        return CH395_ERR_UNKNOW;
    }
}

/********************************************************************************
* Function Name  : ch395_set_uart_baudrate
* Description    : 设置CH395串口波特率，仅在串口模式下有效
* Input          : baudrate---串口波特率
                   0x0012C0代表4800波特率
                   0x002580代表9600波特率
                   0x004B00代表19200波特率
                   0x009600代表38400波特率
                   0x00E100代表57600波特率
                   0x012C00代表76800波特率
                   0x01C200代表115200波特率
                   0x070800代表460800波特率
                   0x0E1000代表921600波特率
                   0x0186A0代表100000波特率
                   0x0F4240代表1000000波特率
                   0x2DC6C0代表3000000波特率
* Output         : None
* Return         : 0--设置失败   非0--设置成功
*******************************************************************************/
rt_size_t ch395_set_uart_baudrate(uint32_t baudrate)
{
    uint8_t send_data[4] = {CMD31_SET_BAUDRATE, (uint8_t)baudrate, (uint8_t)((uint16_t)baudrate >> 8), (uint8_t)(baudrate >> 16)};
    rt_size_t ret;

    if(baudrate != 0x0012C0 && baudrate != 0x002580 && baudrate != 0x004B00 && baudrate != 0x009600 &&
       baudrate != 0x00E100 && baudrate != 0x012C00 && baudrate != 0x01C200 && baudrate != 0x070800 &&
       baudrate != 0x0E1000 && baudrate != 0x0186A0 && baudrate != 0x0F4240 && baudrate != 0x2DC6C0)
    {
        rt_kprintf("baudrate input error......\r\n");
        return 0;
    }

    ret = rt_spi_send(spi_dev, send_data, 4);

    return ret;
}

/********************************************************************************
* Function Name  : ch395_set_ip_addr
* Description    : 设置CH395的IP地址
* Input          : ipaddr--IP地址    buff_size--必须填入实际的缓冲区大小
* Output         : None
* Return         : 0--设置失败   非0--设置成功
*******************************************************************************/
rt_size_t ch395_set_ip_addr(uint8_t *ipaddr, rt_size_t buff_size)
{
    RT_ASSERT(ipaddr != RT_NULL);

    if(buff_size != 4)
    {
        rt_kprintf("IP Addr input error......");
        return 0;
    }

    uint8_t send_data[5] = {CMD40_SET_IP_ADDR, ipaddr[0], ipaddr[1], ipaddr[2], ipaddr[3]};
    rt_size_t ret;

    ret = rt_spi_send(spi_dev, send_data, 5);

    return ret;
}

/********************************************************************************
* Function Name  : ch395_set_gw_ip_addr
* Description    : 设置CH395的网关IP地址
* Input          : ipaddr--网关IP地址   buff_size--必须填入实际的缓冲区大小
* Output         : None
* Return         : 0--设置失败   非0--设置成功
*******************************************************************************/
rt_size_t ch395_set_gw_ip_addr(uint8_t *gwipaddr, rt_size_t buff_size)
{
    RT_ASSERT(gwipaddr != RT_NULL);

    if(buff_size != 4)
    {
       rt_kprintf("GW IP Addr input error......");
       return 0;
    }

    uint8_t send_data[5] = {CMD40_SET_GWIP_ADDR, gwipaddr[0], gwipaddr[1], gwipaddr[2], gwipaddr[3]};
    rt_size_t ret;

    ret = rt_spi_send(spi_dev, send_data, 5);

    return ret;
}

/********************************************************************************
* Function Name  : ch395_set_mask_addr
* Description    : 设置CH395的子网掩码，默认为255.255.255.0
* Input          : maskaddr--子网掩码地址   buff_size--必须填入实际的缓冲区大小
* Output         : None
* Return         : 0--设置失败   非0--设置成功
*******************************************************************************/
rt_size_t ch395_set_mask_addr(uint8_t *maskaddr, rt_size_t buff_size)
{
   RT_ASSERT(maskaddr != RT_NULL);

   if(buff_size != 4)
   {
      rt_kprintf("MASK Addr input error......");
      return 0;
   }

   uint8_t send_data[5] = {CMD40_SET_MASK_ADDR, maskaddr[0], maskaddr[1], maskaddr[2], maskaddr[3]};
   rt_size_t ret;

   ret = rt_spi_send(spi_dev, send_data, 5);

   return ret;
}

/********************************************************************************
* Function Name  : ch395_set_mac_addr
* Description    : 设置CH395的MAC地址
* Input          : macaddr--MAC地址指针   buff_size--必须填入实际的缓冲区大小
* Notice         : CH395芯片出厂时已经烧录了由 IEEE 分配的 MAC 地址，如非必要请勿设置 MAC 地址
* Output         : None
* Return         : 0--设置失败   非0--设置成功
*******************************************************************************/
rt_size_t ch395_set_mac_addr(uint8_t *macaddr, rt_size_t buff_size)
{
    RT_ASSERT(macaddr != RT_NULL);

   if(buff_size != 6)
   {
      rt_kprintf("MAC Addr input error......");
      return 0;
   }

    uint8_t send_data[] = {CMD60_SET_MAC_ADDR, macaddr[0], macaddr[1], macaddr[2], macaddr[3], macaddr[4], macaddr[5]};
    rt_size_t ret;

    ret = rt_spi_send(spi_dev, send_data, sizeof(send_data));
    rt_thread_mdelay(100);

    return ret;
}

/********************************************************************************
* Function Name  : ch395_get_mac_addr
* Description    : 获取CH395的MAC地址。
* Input          : amcaddr--MAC地址指针   buff_size--必须填入实际的缓冲区大小
* Output         : None
* Return         : RT_EOK--成功  -RT_EIO--失败
*******************************************************************************/
rt_err_t ch395_get_mac_addr(uint8_t *macaddr, rt_size_t buff_size)
{
    RT_ASSERT(macaddr != RT_NULL);

    if(buff_size != 6)
    {
      rt_kprintf("The buffer space is not enough......");
      return 0;
    }

    uint8_t cmd = CMD06_GET_MAC_ADDR;
    rt_err_t ret;

    ret = rt_spi_send_then_recv(spi_dev, &cmd, 1, macaddr, buff_size);

    return ret;
}

/*******************************************************************************
* Function Name  : ch395_set_mac_filt
* Description    : 设置MAC过滤。
* Input          : filtype--过滤模式
                   table0--Hash0
                   table1--Hash1
* Output         : None
* Return         : 0--设置失败   非0--设置成功
*******************************************************************************/
rt_size_t ch395_set_mac_filt(uint8_t filtype,uint32_t table0, uint32_t table1)
{
    uint8_t send_data[] = {CMD90_SET_MAC_FILT, filtype, (uint8_t)table0, (uint8_t)((uint16_t)table0 >> 8),
                           (uint8_t)(table0 >> 16), (uint8_t)(table0 >> 24), (uint8_t)table1, (uint8_t)((uint16_t)table1 >> 8),
                           (uint8_t)(table1 >> 16), (uint8_t)(table1 >> 24)};
    rt_size_t ret;

    ret = rt_spi_send(spi_dev, send_data, sizeof(send_data));

    return ret;
}

/********************************************************************************
* Function Name  : ch395_get_unreach_ippt
* Description    : 获取不可达 IP，端口以及协议类型
* Input          : list--保存获取到的不可达   buff_size--必须填入实际的缓冲区大小
                                                第1个字节为不可达代码，请参考 不可达代码(CH395INC.H)
                                                第2个字节为IP包协议类型
                                                第3-4字节为端口号
                                                第4-8字节为IP地址
* Output         : None
* Return         : RT_EOK--成功  -RT_EIO--失败
*******************************************************************************/
rt_err_t ch395_get_unreach_ippt(uint8_t* list, rt_size_t buff_size)
{
    RT_ASSERT(list != RT_NULL);

    if(buff_size != 8)
    {
        rt_kprintf("The buffer space is not enough......");
        return 0;
    }

    uint8_t cmd = CMD08_GET_UNREACH_IPPORT;
    rt_err_t ret;

    ret = rt_spi_send_then_recv(spi_dev, &cmd, 1, list, buff_size);

    return ret;
}

/********************************************************************************
* Function Name  : ch395_get_remote_ipp
* Description    : 获取远端的IP和端口地址，一般在TCP Server模式下使用
* Input          : sockindex--Socket索引
                   list--保存IP和端口
                   buff_size--必须填入实际的缓冲区大小
* Output         : None
* Return         : RT_EOK--成功  -RT_EIO--失败
*******************************************************************************/
rt_err_t ch395_get_remote_ipp(uint8_t sockindex, uint8_t* list, rt_size_t buff_size)
{
    RT_ASSERT(list != RT_NULL);

    if(buff_size != 6)
    {
      rt_kprintf("The buffer space is not enough......\r\n");
      return 0;
    }

    uint8_t sead_data[] = {CMD06_GET_REMOT_IPP_SN, sockindex};
    rt_err_t ret;

    ret = rt_spi_send_then_recv(spi_dev, sead_data, 2, list, buff_size);

    return ret;
}

/*******************************************************************************
* Function Name  : ch395_set_socket_des_ip
* Description    : 设置socket n的目的IP地址
* Input          : sockindex--Socket索引
                   ipaddr--指向IP地址
                   buff_size--必须填入实际的缓冲区大小
* Notice         : 在 Socket 工作在 IPRAW、UDP、TCP Client 模式下，
                                                必须在发送 CMD_OPEN_SOCKET_SN 命令前之前设置目的 IP。
* Output         : None
* Return         : 0--设置失败   非0--设置成功
*******************************************************************************/
rt_size_t ch395_set_socket_des_ip(uint8_t sockindex, uint8_t *ipaddr, rt_size_t buff_size)
{
    RT_ASSERT(ipaddr != RT_NULL);

    if(buff_size != 4)
    {
      rt_kprintf("The buffer space error......");
      return 0;
    }

    uint8_t send_data[] = {CMD50_SET_IP_ADDR_SN, sockindex, ipaddr[0], ipaddr[1], ipaddr[2], ipaddr[3]};
    rt_size_t ret;

    ret = rt_spi_send(spi_dev, send_data, sizeof(send_data));

    return ret;
}

/*******************************************************************************
* Function Name  : ch395_set_socket_prot_type
* Description    : 设置socket 的协议类型
* Input          : sockindex--Socket索引
                   prottype--协议类型
                   03H--TCP 模式
                   02H--UDP 模式
                   01H--MAC原始报文模式
                   00H--原始报文模式
* Output         : None
* Return         : 0--设置失败   非0--设置成功
*******************************************************************************/
rt_size_t ch395_set_socket_prot_type(uint8_t sockindex,uint8_t prottype)
{
    uint8_t send_data[] = {CMD20_SET_PROTO_TYPE_SN, sockindex, prottype};
    rt_size_t ret;

    if(prottype != 0x03 && prottype != 0x02 && prottype != 0x01 && prottype != 0x00)
    {
        rt_kprintf("set socket prot type error!\r\n");
        return 0;
    }

    ret = rt_spi_send(spi_dev, send_data, sizeof(send_data));

    return ret;
}

/*******************************************************************************
* Function Name  : ch395_set_socket_des_Port
* Description    : 设置 Socket 目的端口
* Input          : sockindex--Socket索引
                   desprot--2字节目的端口
* Output         : None
* Return         : 0--设置失败   非0--设置成功
*******************************************************************************/
rt_size_t ch395_set_socket_des_Port(uint8_t sockindex, uint16_t desprot)
{
    uint8_t send_data[] = {CMD30_SET_DES_PORT_SN, sockindex, (uint8_t)desprot, (uint8_t)(desprot >> 8)};
    rt_size_t ret;

    ret = rt_spi_send(spi_dev, send_data, sizeof(send_data));

    return ret;
}

/*******************************************************************************
* Function Name  : ch395_set_socket_sour_port
* Description    : 设置 Socket 源端口
* Input          : sockindex--Socket索引
                   desprot--2字节源端口
* Notice         : 如果两个或者多个 Socket 都采用相同的模式，则源端口号不得相同。
                                                例如 Socket 0 为 UDP 模式，源端口为 600，Socket 1 同样为 UDP 模式，不可以再使用源端口 600
* Output         : None
* Return         : 0--设置失败   非0--设置成功
*******************************************************************************/
rt_size_t ch395_set_socket_sour_port(uint8_t sockindex, uint16_t surprot)
{
    uint8_t send_data[] = {CMD30_SET_SOUR_PORT_SN, sockindex, (uint8_t)surprot, (uint8_t)(surprot >> 8)};
    rt_size_t ret;

    ret = rt_spi_send(spi_dev, send_data, sizeof(send_data));

    return ret;
}

/******************************************************************************
* Function Name  : ch395_set_socket_ipraw_proto
* Description    : IP模式下，socket IP包协议字段
* Input          : sockindex--Socket索引
                   prototype--IPRAW模式1字节协议字段
* Notice         : 该函数仅在 IPRAW 模式下有效，如果多个 Socket 均采用了 IPRAW 模式，则协议码不得重复使用
                   IPRAW 数据处理优先级高于 UDP 和 TCP，所以协议码也必须不得与其他 Socket 相同
* Output         : None
* Return         : 0--设置失败   非0--设置成功
*******************************************************************************/
rt_size_t ch395_set_socket_ipraw_proto(uint8_t sockindex, uint8_t prototype)
{
    uint8_t send_data[] = {CMD20_SET_IPRAW_PRO_SN, sockindex, prototype};
    rt_size_t ret;

    ret = rt_spi_send(spi_dev, send_data, sizeof(send_data));

    return ret;
}

/********************************************************************************
* Function Name  : ch395_enable_ping
* Description    : 开启/关闭 PING
* Input          : enable : 1  开启PING
                            0  关闭PING
* Output         : None
* Return         : 0--设置失败   非0--设置成功
*******************************************************************************/
rt_size_t ch395_enable_ping(uint8_t enable)
{
    uint8_t send_data[] = {CMD01_PING_ENABLE, enable};
    rt_size_t ret;

    if(enable != 1 && enable != 0)
    {
        rt_kprintf("enable ping error!\r\n");
        return 0;
    }

    ret = rt_spi_send(spi_dev, send_data, sizeof(send_data));

    return ret;
}

/********************************************************************************
* Function Name  : ch395_send_data
* Description    : 向发送缓冲区写数据
* Input          : sockindex--Socket索引
                   databuf--数据缓冲区
                   len--长度      输入数据的长度不得大于数据缓冲区的大小
* Notice         : 在 MACRAW模式下，输入的数据的长度最大只能为 1514
* Output         : None
* Return         : RT_EOK--成功  -RT_EIO--失败
*******************************************************************************/
rt_err_t ch395_send_data(uint8_t sockindex, uint8_t* databuf, uint16_t len)
{
    uint8_t send_data[] = {CMD30_WRITE_SEND_BUF_SN, (uint8_t)sockindex, (uint8_t)len, (uint8_t)(len>>8)};
    rt_err_t ret;

    RT_ASSERT(databuf != RT_NULL);

    ret = rt_spi_send_then_send(spi_dev, send_data, sizeof(send_data), databuf, len);

    return ret;
}

/*******************************************************************************
* Function Name  : ch395_get_recv_length
* Description    : 获取接收缓冲区长度
* Input          : sockindex--Socket索引
* Output         : None
* Return         : 返回接收缓冲区有效长度
*******************************************************************************/
uint16_t ch395_get_recv_length(uint8_t sockindex)
{

    uint8_t sead_data[] = {CMD12_GET_RECV_LEN_SN, (uint8_t)sockindex};
    uint8_t recv_data[2] = {0};
    uint16_t ret_data;
    rt_err_t ret;

    ret = rt_spi_send_then_recv(spi_dev, sead_data, sizeof(sead_data), recv_data, 2);

    if(ret == RT_EOK)
    {
       ret_data = (uint16_t)(recv_data[1] << 8) + recv_data[0];
       return ret_data;
    }
    else{
       return RT_NULL;
    }
}

/*******************************************************************************
* Function Name  : ch395_clear_recv_buf
* Description    : 清除接收缓冲区
* Input          : sockindex Socket索引
* Output         : None
* Return         : 0--清除失败   非0--清除成功
*******************************************************************************/
rt_size_t ch395_clear_recv_buf(uint8_t sockindex)
{
    uint8_t send_data[] = {CMD10_CLEAR_RECV_BUF_SN, (uint8_t)sockindex};
    rt_size_t ret;

    ret = rt_spi_send(spi_dev, send_data, sizeof(send_data));

    return ret;
}

/********************************************************************************
* Function Name  : ch395_get_recv_data
* Description    : 读取接收缓冲区数据
* Input          : sockindex--Socket索引
                   len--长度
                   pbuf--缓冲区
* Output         : None
* Return         : RT_EOK--成功  -RT_EIO--失败
*******************************************************************************/
rt_err_t ch395_get_recv_data(uint8_t sockindex, uint16_t len, uint8_t *pbuf)
{
    if(!len)
    {
        rt_kprintf("recv data len is 0!\r\n");
        return -RT_EIO;
    }

    RT_ASSERT(pbuf != RT_NULL);

    uint8_t sead_data[] = {CMD30_READ_RECV_BUF_SN, sockindex, (uint8_t)len, (uint8_t)(len>>8)};
    rt_err_t ret;

    ret = rt_spi_send_then_recv(spi_dev, sead_data, sizeof(sead_data), pbuf, len);

    return ret;
}

/********************************************************************************
* Function Name  : ch395_set_retry_count
* Description    : 设置重试次数
* Input          : count--重试值
* Notice         : 允许重试最大值为 20，输入数据大于 20，则会按 20 处理。默认重试次数为 12 次，重试仅在 TCP 模式下有效。
* Output         : None
* Return         : 0--设置失败   非0--设置成功
********************************************************************************/
rt_size_t ch395_set_retry_count(uint8_t count)
{
    uint8_t send_data[] = {CMD10_SET_RETRAN_COUNT, count};
    rt_size_t ret;

    ret = rt_spi_send(spi_dev, send_data, sizeof(send_data));

    return ret;
}

/********************************************************************************
* Function Name  : ch395_set_retry_period
* Description    : 设置重试周期
* Input          : period 重试周期单位为毫秒，最大1000ms
* Output         : None
* Return         : 0--设置失败   非0--设置成功
*******************************************************************************/
rt_size_t ch395_set_retry_period(uint16_t period)
{
    uint8_t send_data[] = {CMD20_SET_RETRAN_PERIOD, (uint8_t)period, (uint8_t)(period>>8)};
    rt_size_t ret;

    ret = rt_spi_send(spi_dev, send_data, sizeof(send_data));

    return ret;
}

/********************************************************************************
* Function Name  : ch395_get_socket_status
* Description    : 用于获取 Socket 的状态
* Input          : sockindex--socket索引
                   status--保存状态的缓冲区
* Output         : socket n的状态信息
                                                第1字节为socket 打开或者关闭
                                                第2字节为TCP状态
* Return         : RT_EOK--成功  -RT_EIO--失败
*******************************************************************************/
rt_err_t ch395_get_socket_status(uint8_t sockindex, uint8_t *status)
{
    RT_ASSERT(status != RT_NULL);

    uint8_t sead_data[] = {CMD12_GET_SOCKET_STATUS_SN, sockindex};
    rt_err_t ret;

    ret = rt_spi_send_then_recv(spi_dev, sead_data, sizeof(sead_data), status, 2);

    return ret;
}

/*******************************************************************************
* Function Name  : ch395_open_socket
* Description    : 打开socket，此命令需要等待执行成功
* Input          : sockindex--Socket索引
* Output         : None
* Return         : 返回执行结果
*******************************************************************************/
uint8_t ch395_open_socket(uint8_t sockindex)
{
    uint8_t sead_data[] = {CMD1W_OPEN_SOCKET_SN, sockindex};
    uint8_t status, count = 0;
    rt_size_t ret;

    ret = rt_spi_send(spi_dev, sead_data, sizeof(sead_data));

    if(ret > 0)
    {
        while(1)
        {
           rt_thread_mdelay(5);                        /* 延时查询，建议2ms以上*/
           status = ch395_get_cmd_status();                /* 不能过于频繁查询*/

           if(status !=CH395_ERR_BUSY)                  /* 如果CH395芯片返回忙状态*/
           {
               break;
           }

           if(count++ > 200)
           {
               return CH395_ERR_UNKNOW;                     /* 超时退出,本函数需要500ms以上执行完毕 */
           }
        }

        return status;
    }
    else{
        return CH395_ERR_UNKNOW;
    }
}

/*******************************************************************************
* Function Name  : CH395OpenSocket
* Description    : 关闭socket
* Input          : sockindex Socket索引
* Output         : None
* Return         : 返回执行结果
*******************************************************************************/
uint8_t  ch395_close_socket(uint8_t sockindex)
{
    uint8_t sead_data[] = {CMD1W_CLOSE_SOCKET_SN, sockindex};
    uint8_t status, count = 0;
    rt_size_t ret;

    ret = rt_spi_send(spi_dev, sead_data, sizeof(sead_data));

    if(ret > 0)
    {
        while(1)
        {
           rt_thread_mdelay(5);                        /* 延时查询，建议2ms以上*/
           status = ch395_get_cmd_status();                /* 不能过于频繁查询*/

           if(status !=CH395_ERR_BUSY)                  /* 如果CH395芯片返回忙状态*/
           {
               break;
           }

           if(count++ > 200)
           {
               return CH395_ERR_UNKNOW;                     /* 超时退出,本函数需要500ms以上执行完毕 */
           }
        }

        return status;
    }
    else{
        return CH395_ERR_UNKNOW;
    }
}

/********************************************************************************
* Function Name  : ch395_tcp_connect
* Description    : TCP连接，仅在TCP模式下有效，此命令需要等待执行成功
* Input          : sockindex Socket索引
* Output         : None
* Return         : 返回执行结果
*******************************************************************************/
uint8_t ch395_tcp_connect(uint8_t sockindex)
{
    uint8_t sead_data[] = {CMD1W_TCP_CONNECT_SN, sockindex};
    uint8_t status, count = 0;
    rt_size_t ret;

    ret = rt_spi_send(spi_dev, sead_data, sizeof(sead_data));

    if(ret > 0)
    {
        while(1)
        {
           rt_thread_mdelay(5);                        /* 延时查询，建议2ms以上*/
           status = ch395_get_cmd_status();                /* 不能过于频繁查询*/

           if(status !=CH395_ERR_BUSY)                  /* 如果CH395芯片返回忙状态*/
           {
               break;
           }

           if(count++ > 200)
           {
               return CH395_ERR_UNKNOW;                     /* 超时退出,本函数需要500ms以上执行完毕 */
           }
        }

        return status;
    }
    else{
        return CH395_ERR_UNKNOW;
    }
}

/******************************************************************************
* Function Name  : CH395TCPListen
* Description    : TCP监听，仅在TCP模式下有效，此命令需要等待执行成功
* Input          : sockindex Socket索引
* Output         : None
* Return         : 返回执行结果
*******************************************************************************/
uint8_t ch395_tcp_listen(uint8_t sockindex)
{
    uint8_t sead_data[] = {CMD1W_TCP_LISTEN_SN, sockindex};
    uint8_t status, count = 0;
    rt_size_t ret;

    ret = rt_spi_send(spi_dev, sead_data, sizeof(sead_data));

    if(ret > 0)
    {
        while(1)
        {
           rt_thread_mdelay(5);                        /* 延时查询，建议2ms以上*/
           status = ch395_get_cmd_status();                /* 不能过于频繁查询*/

           if(status !=CH395_ERR_BUSY)                  /* 如果CH395芯片返回忙状态*/
           {
               break;
           }

           if(count++ > 200)
           {
               return CH395_ERR_UNKNOW;                     /* 超时退出,本函数需要500ms以上执行完毕 */
           }
        }

        return status;
    }
    else{
        return CH395_ERR_UNKNOW;
    }
}

/********************************************************************************
* Function Name  : ch395_tcp_disconnect
* Description    : TCP断开，仅在TCP模式下有效，此命令需要等待执行成功
* Input          : sockindex Socket索引
* Output         : None
* Return         : 返回执行结果
*******************************************************************************/
uint8_t ch395_tcp_disconnect(uint8_t sockindex)
{
    uint8_t sead_data[] = {CMD1W_TCP_DISNCONNECT_SN, sockindex};
    uint8_t status, count = 0;
    rt_size_t ret;

    ret = rt_spi_send(spi_dev, sead_data, sizeof(sead_data));

    if(ret > 0)
    {
        while(1)
        {
           rt_thread_mdelay(5);                        /* 延时查询，建议2ms以上*/
           status = ch395_get_cmd_status();                /* 不能过于频繁查询*/

           if(status !=CH395_ERR_BUSY)                  /* 如果CH395芯片返回忙状态*/
           {
               break;
           }

           if(count++ > 200)
           {
               return CH395_ERR_UNKNOW;                     /* 超时退出,本函数需要500ms以上执行完毕 */
           }
        }

        return status;
    }
    else{
        return CH395_ERR_UNKNOW;
    }
}

/*******************************************************************************
* Function Name  : ch395_get_socket_int
* Description    : 获取socket n的中断状态
* Input          : sockindex--socket索引
* Output         : None
* Return         : 中断状态--成功   0--失败
*******************************************************************************/
uint8_t ch395_get_socket_int(uint8_t sockindex)
{
    uint8_t sead_data[] = {CMD11_GET_INT_STATUS_SN, sockindex};
    uint8_t intstatus;
    rt_err_t ret;

    ret = rt_spi_send_then_recv(spi_dev, sead_data, sizeof(sead_data), &intstatus, 1);

    if(ret == RT_EOK)
    {
        return intstatus;
    }
    else{
        return 0;
    }
}

/*******************************************************************************
* Function Name  : ch395_crc_ret_6bit
* Description    : 对多播地址进行CRC运算，并取高6位。
* Input          : mac_addr   MAC地址
* Output         : None
* Return         : 返回CRC32的高6位
*******************************************************************************/
uint8_t ch395_crc_ret_6bit(uint8_t *mac_addr)
{
    RT_ASSERT(mac_addr != RT_NULL);

    int32_t perByte;
    int32_t perBit;
    const uint32_t poly = 0x04C11DB7;
    uint32_t crc_value = 0xFFFFFFFF;
    uint8_t c;
    for ( perByte = 0; perByte < 6; perByte ++ )
    {
        c = *(mac_addr++);
        for ( perBit = 0; perBit < 8; perBit++ )
        {
            crc_value = (crc_value << 1)^((((crc_value >> 31)^c) & 0x01)?poly:0);
            c >>= 1;
        }
    }
    crc_value = crc_value >> 26;

    return ((uint8_t)crc_value);
}

/******************************************************************************
* Function Name  : ch395_dhcp_enable
* Description    : 启动/停止DHCP
* Input          : flag   1:启动DHCP;0：停止DHCP
* Output         : None
* Return         : 执行状态
*******************************************************************************/
uint8_t ch395_dhcp_enable(uint8_t flag)
{
    uint8_t sead_data[] = {CMD10_DHCP_ENABLE, flag};
    uint8_t status, count = 0;
    rt_size_t ret;

    if(flag != 1 && flag != 0)
    {
        rt_kprintf("enable dhcp error!\r\n");
        return CH395_ERR_UNKNOW;
    }

    ret = rt_spi_send(spi_dev, sead_data, sizeof(sead_data));

    if(ret > 0)
    {
        while(1)
        {
           rt_thread_mdelay(20);                        /* 延时查询，建议2ms以上*/
           status = ch395_get_cmd_status();                /* 不能过于频繁查询*/

           if(status !=CH395_ERR_BUSY)                  /* 如果CH395芯片返回忙状态*/
           {
               break;
           }

           if(count++ > 200)
           {
               return CH395_ERR_UNKNOW;                     /* 超时退出,本函数需要500ms以上执行完毕 */
           }
        }

        return status;
    }
    else{
        return CH395_ERR_UNKNOW;
    }
}

/******************************************************************************
* Function Name  : ch395_get_dhcp_status
* Description    : 获取DHCP状态
* Input          : None
* Output         : None
* Return         : DHCP状态，0为成功，其他值表示错误
*******************************************************************************/
uint8_t ch395_get_dhcp_status(void)
{
    uint8_t cmd = CMD01_GET_DHCP_STATUS;
    uint8_t status;
    rt_err_t ret;

    ret = rt_spi_send_then_recv(spi_dev, &cmd, 1, &status, 1);

    if(ret == RT_EOK)
    {
       return status;
    }
    else{
       return 2;
    }
}

/*******************************************************************************
* Function Name  : ch395_get_ip_inf
* Description    : 获取IP，子网掩码和网关地址
* Input          : addr--接收缓冲区    buf_size--设置的缓冲区大小必须为20个字节
* Output         : 4 字节 IP 地址、4 字节网关 IP、4 字节子网掩码、4 字节 DNS1(主 DNS)、4 字节DNS2(次 DNS)
* Return         : RT_EOK--成功  -RT_EIO--失败
*******************************************************************************/
rt_err_t ch395_get_ip_inf(uint8_t *addr, rt_size_t buf_size)
{
    RT_ASSERT(addr != RT_NULL);
    if(buf_size != 20)
    {
        rt_kprintf("The buffer space error......");
        return -RT_EIO;
    }

    uint8_t cmd = CMD014_GET_IP_INF;
    rt_err_t ret;

    ret = rt_spi_send_then_recv(spi_dev, &cmd, 1, addr, buf_size);

    return ret;
}

/*******************************************************************************
* Function Name  : ch395_write_gpio_addr
* Description    : 写GPIO寄存器
* Input          : regadd    寄存器地址
                   regval    寄存器值
* Output         : None
* Return         : 0--写失败   非0--写成功
*******************************************************************************/
rt_size_t ch395_write_gpio_addr(uint8_t regadd, uint8_t regval)
{
    uint8_t send_data[] = {CMD20_WRITE_GPIO_REG, regadd, regval};
    rt_size_t ret;

    ret = rt_spi_send(spi_dev, send_data, sizeof(send_data));

    return ret;
}

/*******************************************************************************
* Function Name  : CH395ReadGPIOAddr
* Description    : 读GPIO寄存器
* Input          : regadd   寄存器地址
* Output         : None
* Return         : 寄存器的值
*******************************************************************************/
uint8_t ch395_read_gpio_addr(uint8_t regadd)
{
    uint8_t sead_data[] = {CMD10_READ_GPIO_REG, regadd};
    uint8_t val;
    rt_err_t ret;

    ret = rt_spi_send_then_recv(spi_dev, &sead_data, sizeof(sead_data), &val, 1);

    if(ret == RT_EOK)
    {
        return val;
    }
    else{
        return 0;
    }
}

/*******************************************************************************
* Function Name  : ch395_eeprom_erase
* Description    : 擦除EEPROM
* Input          : None
* Output         : None
* Return         : 执行状态
*******************************************************************************/
uint8_t ch395_eeprom_erase(void)
{
    uint8_t cmd = CMD00_EEPROM_ERASE;
    int8_t status = 0, count = 0;
    rt_size_t ret;

    ret = rt_spi_send(spi_dev, &cmd, 1);

    if(ret > 0)
    {
        while(1)
        {
           rt_thread_mdelay(20);                        /* 延时查询，建议2ms以上*/
           status = ch395_get_cmd_status();                /* 不能过于频繁查询*/

           if(status !=CH395_ERR_BUSY)                  /* 如果CH395芯片返回忙状态*/
           {
               break;
           }

           if(count++ > 200)
           {
               return CH395_ERR_UNKNOW;                     /* 超时退出,本函数需要500ms以上执行完毕 */
           }
        }

        return status;
    }
    else{
        return CH395_ERR_UNKNOW;
    }
}

/*******************************************************************************
* Function Name  : ch395_eeprom_write
* Description    : 写EEPROM
* Input          : eepaddr  EEPROM地址
                   buf      缓冲区地址
                   len      长度         字节流的长度不得大于 64 字节
* Output         : None
* Return         : 执行状态
*******************************************************************************/
uint8_t ch395_eeprom_write(uint16_t eepaddr, uint8_t* buf, uint8_t len)
{
    RT_ASSERT(buf != RT_NULL);

    if(len > 64)
    {
        return CH395_ERR_UNKNOW;
    }

    uint8_t sead_data[] = {CMD30_EEPROM_WRITE, (uint8_t)(eepaddr), (uint8_t)(eepaddr >> 8), len};
    int8_t status = 0, count = 0;
    rt_size_t ret;

    ret = rt_spi_send_then_send(spi_dev, sead_data, sizeof(sead_data), buf, len);

    if(ret == RT_EOK)
    {
        while(1)
        {
           rt_thread_mdelay(20);                        /* 延时查询，建议2ms以上*/
           status = ch395_get_cmd_status();                /* 不能过于频繁查询*/

           if(status !=CH395_ERR_BUSY)                  /* 如果CH395芯片返回忙状态*/
           {
               break;
           }

           if(count++ > 200)
           {
               return CH395_ERR_UNKNOW;                 /* 超时退出,本函数需要500ms以上执行完毕 */
           }
        }

        return status;
    }
    else{
        return CH395_ERR_UNKNOW;
    }
}

/*******************************************************************************
* Function Name  : ch395_eeprom_eead
* Description    : 写EEPROM
* Input          : eepaddr  EEPROM地址
*                ：buf      缓冲区地址
*                ：len      长度        字节流的长度不得大于 64 字节
* Output         : None
* Return         : RT_EOK--成功   -RT_EIO--失败
*******************************************************************************/
rt_err_t ch395_eeprom_eead(uint16_t eepaddr, uint8_t* buf, uint8_t len)
{
    RT_ASSERT(buf != RT_NULL);

    if(len > 64)
    {
       return -RT_EIO;
    }

    uint8_t sead_data[] = {CMD30_EEPROM_READ, (uint8_t)(eepaddr), (uint8_t)(eepaddr >> 8), len};
    rt_err_t ret;

    ret = rt_spi_send_then_recv(spi_dev, sead_data, sizeof(sead_data), buf, len);

    return ret;
}

/*******************************************************************************
* Function Name  : ch395_set_tcp_mss
* Description    : 设置TCP MSS值
* Input          : tcpmss  最大值为 1460，最小值为 60
* Output         : None
* Return         : 0--设置失败   非0--设置成功
*******************************************************************************/
rt_size_t ch395_set_tcp_mss(uint16_t tcpmss)
{
    if(tcpmss < 60 && tcpmss > 1460)
    {
        rt_kprintf("tcpmss val error......\r\n");
        return 0;
    }

    uint8_t sead_data[] = {CMD20_SET_TCP_MSS, (uint8_t)(tcpmss), (uint8_t)(tcpmss >> 8)};
    rt_size_t ret;

    ret = rt_spi_send(spi_dev, sead_data, sizeof(sead_data));

    return ret;
}

/*******************************************************************************
* Function Name  : ch395_set_socket_recv_buf
* Description    : 设置Socket接收缓冲区
* Input          : sockindex  socket索引
                   startblk   起始地址
                   blknum     单位缓冲区个数 ，单位为512字节
* Output         : None
* Return         : 0--设置失败   非0--设置成功
*******************************************************************************/
rt_size_t ch395_set_socket_recv_buf(uint8_t sockindex, uint8_t startblk, uint8_t blknum)
{
    uint8_t sead_data[] = {CMD30_SET_RECV_BUF, sockindex, startblk, blknum};
    rt_size_t ret;

    ret = rt_spi_send(spi_dev, sead_data, sizeof(sead_data));

    return ret;
}

/*******************************************************************************
* Function Name  : ch395_set_socket_send_buf
* Description    : 设置Socket发送缓冲区
* Input          : sockindex  socket索引
                   startblk   起始地址
                   blknum     单位缓冲区个数 ，单位为512字节
* Output         : None
* Return         : 0--设置失败   非0--设置成功
*******************************************************************************/
rt_size_t ch395_set_socket_send_buf(uint8_t sockindex, uint8_t startblk, uint8_t blknum)
{
    uint8_t sead_data[] = {CMD30_SET_SEND_BUF, sockindex, startblk, blknum};
    rt_size_t ret;

    ret = rt_spi_send(spi_dev, sead_data, sizeof(sead_data));

    return ret;
}

/*******************************************************************************
* Function Name  : ch395_udp_send_To
* Description    : UDP向指定的IP和端口发送数据
* Input          : buf      发送数据缓冲区
                   len      发送数据长度
                   ip       目标IP
                   port     目标端口
                   sockeid  socket索引值
* Output         : None
* Return         : None
*******************************************************************************/
void ch395_udp_send_To(uint8_t* buf,uint32_t len, uint8_t* ip, rt_size_t ip_size, uint16_t port, uint8_t sockindex)
{
    rt_size_t ret;
    rt_err_t val;

    ret = ch395_set_socket_des_ip(sockindex, ip, ip_size);           /* 设置scoket n的目的IP地址 */
    if(ret == 0)
    {
        rt_kprintf("set socket des ip error......\r\n");
        return ;
    }

    ret = ch395_set_socket_des_Port(sockindex, port);               /* 设置socket n的目的端口 */
    if(ret == 0)
    {
        rt_kprintf("set socket des Port error......\r\n");
        return ;
    }

    val = ch395_send_data(sockindex, buf, len);                    /* 向socket n的发送缓冲区写数据 */
    if(val == -RT_EIO)
    {
        rt_kprintf("send_data error......\r\n");
        return ;
    }
}

/*******************************************************************************
* Function Name  : ch395_set_start_para
* Description    : 设置CH395启动参数
* Input          : mdata
* Output         : None
* Return         : 0--设置失败   非0--设置成功
*******************************************************************************/
rt_size_t ch395_set_start_para(uint32_t mdata)
{
   uint8_t sead_data[] = {CMD40_SET_FUN_PARA, (uint8_t)mdata, (uint8_t)(mdata >> 8), (uint8_t)(mdata >> 16), (uint8_t)(mdata >> 24)};
   rt_size_t ret;

   ret = rt_spi_send(spi_dev, sead_data, sizeof(sead_data));

   return ret;
}

/*******************************************************************************
* Function Name  : ch395_get_glob_int_status_all
* Description    : 获取全局中断状态，收到此命令CH395自动取消中断,0x44及以上版本使用
* Input          : None
* Output         : None
* Return         : 返回当前的全局中断状态
*******************************************************************************/
uint16_t ch395_get_glob_int_status_all(void)
{
    uint8_t cmd = CMD02_GET_GLOB_INT_STATUS_ALL;
    uint8_t recv_data[2] = {0};
    uint16_t init_status;
    rt_err_t ret;

    ret = rt_spi_send_then_recv(spi_dev, &cmd, 1, recv_data, 2);

    if(ret == RT_EOK)
    {
      init_status = (uint16_t)(recv_data[1] << 8) + recv_data[0];
      return init_status;
    }
    else{
      return RT_NULL;
    }

}
///**************************** endfile *************************************/
