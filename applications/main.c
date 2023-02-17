/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 *   Date           Author           Notes
 * 2022-11-25     lianghaoqing    first version
 */

#include <rtthread.h>

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#include <board.h>
#include <rtdevice.h>
#include <drv_spi.h>

#include "ch395_inc.h"
#include "ch395_cmd.h"

#define SPI_DEVICE_NAME     "spi10"

typedef enum{
	DEF_DEV_INIT,
	DEF_PYH_CON,
	DEF_SOCK_CON
}CH395_RUN_STATUE;

/* CH395相关定义 */
uint8_t CH395IPAddr[4] = {192,168,1,200};                         /* CH395IP地址 */
uint8_t CH395GWIPAddr[4] = {192,168,1,1};                        /* CH395网关 */
uint8_t CH395IPMask[4] = {255,255,255,0};                        /* CH395子网掩码 */

/* socket 相关定义*/
uint16_t SocketServerPort   = 8080;           /* Socket 目的端口 */

/*存储网络接收的数据*/
#define recv_buff_len 1500
uint8_t recv_buff[recv_buff_len];

/* 版本号 */
uint8_t ch395_version = 0;
uint8_t buf[6] = {0};

//INT  -- 连接模块INT引脚 (检测到中断信号之后再获取数据)
#define CH395_INT_PORT  GPIOC
#define CH395_INT_PIN   GPIO_PIN_13

#define CH395_INT_PIN_INPUT()    (CH395_INT_PORT->IDR & CH395_INT_PIN)   /* 获取INT电平 */


/* 挂载   CH395 到 SPI 总线 */
static int rt_hw_spi_CH395_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();

    rt_hw_spi_device_attach("spi1", "spi10", GPIOA, GPIO_PIN_4);

    return RT_EOK;
}
/* 导出到自动初始化 */
INIT_COMPONENT_EXPORT(rt_hw_spi_CH395_init);

/*******************************************************************************
* Function Name  : Query395Interrupt
* Description    : 查询CH395中断(INT#低电平)
* Input          : None
* Output         : None
* Return         : 返回中断状态
*******************************************************************************/
uint8_t Query395Interrupt(void)
{
    return( CH395_INT_PIN_INPUT() ? 0 : 1 );
}

/*配置 Socket 发送和接收缓存区*/
/*芯片有0-47个块,每个块512字节大小*/
void socket_buffer_config(void )
{
    /*Socket 0*/
    ch395_set_socket_recv_buf(0,0,4);//第0,1,2,3块缓存区作为Socket的接收缓存区(512*4=2KB)
    ch395_set_socket_send_buf(0,4,2);//第4,5块缓存区作为Socket的发送缓存区(512*2=1KB)
    /*Socket 1*/
    ch395_set_socket_recv_buf(1,6,4);//第6,7,8,9块缓存区作为Socket的接收缓存区(512*4=2KB)
    ch395_set_socket_send_buf(1,10,2);//第10,11块缓存区作为Socket的发送缓存区(512*2=1KB)
    /*Socket 2*/
    ch395_set_socket_recv_buf(2,12,4);//第12,13,14,15块缓存区作为Socket的接收缓存区(512*4=2KB)
    ch395_set_socket_send_buf(2,16,2);//第16,17块缓存区作为Socket的发送缓存区(512*2=1KB)
    /*Socket 3*/
    ch395_set_socket_recv_buf(3,18,4);//第18,19,20,21块缓存区作为Socket的接收缓存区(512*4=2KB)
    ch395_set_socket_send_buf(3,22,2);//第22,23块缓存区作为Socket的发送缓存区(512*2=1KB)

    /*硬件版本大于4的才有Socket4-7*/
  if(ch395_version >= 0x44)
    {
        /*Socket 4*/
        ch395_set_socket_recv_buf(4,24,4);//第24,25,26,27块缓存区作为Socket的接收缓存区(512*4=2KB)
        ch395_set_socket_send_buf(4,28,2);//第28,29块缓存区作为Socket的发送缓存区(512*2=1KB)
        /*Socket 5*/
        ch395_set_socket_recv_buf(5,30,4);//第30,31,32,33块缓存区作为Socket的接收缓存区(512*4=2KB)
        ch395_set_socket_send_buf(5,34,2);//第34,35块缓存区作为Socket的发送缓存区(512*2=1KB)

        /*Socket 6*/
        ch395_set_socket_recv_buf(6,36,4);//第36,37,38,39块缓存区作为Socket的接收缓存区(512*4=2KB)
        ch395_set_socket_send_buf(6,40,2);//第40,41块缓存区作为Socket的发送缓存区(512*2=1KB)

        /*Socket 7*/
        ch395_set_socket_recv_buf(7,42,4);//第42,43,44,45块缓存区作为Socket的接收缓存区(512*4=2KB)
        ch395_set_socket_send_buf(7,46,2);//第46,47块缓存区作为Socket的发送缓存区(512*2=1KB)
    }
}

/*******************************************************************************
* Function Name  : ch395_socket_tcp_server_init
* Description    : 初始化socket
* Input          : None
* Output         : None
* Return         : 0:初始化成功; others:初始化失败
*******************************************************************************/
char ch395_socket_tcp_server_init(void)
{
    /*让Socket作为监听连接*/
    ch395_set_socket_prot_type(0,PROTO_TYPE_TCP);               /* 协议类型 */
    ch395_set_socket_sour_port(0,SocketServerPort);             /* 本地端口号 */
    if(ch395_open_socket(0) !=0)                                /* 打开Socket */
    {
        return 1;
    }

    /*其它Socket作为数据通信*/
    /*想要几路Socket客户端连接通信,就需要配置几个Socket,所以模块最多支持7个Socket TCP客户端通信*/
    ch395_set_socket_prot_type(1,PROTO_TYPE_TCP); /* 协议类型 */
    ch395_set_socket_sour_port(1,SocketServerPort);/* 本地端口号 */

    ch395_set_socket_prot_type(2,PROTO_TYPE_TCP); /* 协议类型 */
    ch395_set_socket_sour_port(2,SocketServerPort);/* 本地端口号 */

    ch395_set_socket_prot_type(3,PROTO_TYPE_TCP); /* 协议类型 */
    ch395_set_socket_sour_port(3,SocketServerPort);/* 本地端口号 */

    ch395_set_socket_prot_type(4,PROTO_TYPE_TCP); /* 协议类型 */
    ch395_set_socket_sour_port(4,SocketServerPort);/* 本地端口号 */

    ch395_set_socket_prot_type(5,PROTO_TYPE_TCP); /* 协议类型 */
    ch395_set_socket_sour_port(5,SocketServerPort);/* 本地端口号 */

    ch395_set_socket_prot_type(6,PROTO_TYPE_TCP); /* 协议类型 */
    ch395_set_socket_sour_port(6,SocketServerPort);/* 本地端口号 */

    ch395_set_socket_prot_type(7,PROTO_TYPE_TCP); /* 协议类型 */
    ch395_set_socket_sour_port(7,SocketServerPort);/* 本地端口号 */

    return 0;
}

/*******************************************************************************
* Function Name  : ch395_socket_tcp_client_interrupt
* Description    : socket中断处理函数
* Input          : sockindex  Socket索引(0,1,2,3,4,5,6,7)
* Output         : None
* Return         : None
*******************************************************************************/
void ch395_socket_tcp_client_interrupt(uint8_t sockindex)
{
    uint8_t  sock_int_socket;
    uint16_t len;

    /* 获取socket 的中断状态 */
    sock_int_socket = ch395_get_socket_int(sockindex);

    /* 发送缓冲区空闲，可以继续写入要发送的数据 */
    if(sock_int_socket & SINT_STAT_SENBUF_FREE)
    {
    }

    /* 发送完成中断 */
    if(sock_int_socket & SINT_STAT_SEND_OK)
    {
    }

    /* 接收数据中断 */
    if(sock_int_socket & SINT_STAT_RECV)
    {
        len = ch395_get_recv_length(sockindex);/* 获取当前缓冲区内数据长度 */
        if(len == 0)return;
        if(len > recv_buff_len)
        {
            len = recv_buff_len;
        }
        ch395_get_recv_data(sockindex,len,recv_buff);/* 读取数据 */

        /* 把接收的数据发送给客户端 */
        ch395_send_data(sockindex,recv_buff,len);

		rt_kprintf("socket%d recv data len:%d   ",sockindex, len);
        
		/* 使用串口打印接收的数据 */
		rt_kprintf("data:");
        for(int i = 0; i < len ;++i)
        {
            rt_kprintf("%c",recv_buff[i]);
        }
        rt_kprintf("\r\n");
    }

    /* 连接中断，仅在TCP模式下有效*/
    if(sock_int_socket & SINT_STAT_CONNECT)
    {
        rt_kprintf("socket%d SINT_STAT_CONNECT\r\n",sockindex);

        ch395_get_remote_ipp(sockindex,buf,sizeof(buf));//获取客户端信息

        rt_kprintf("IP address = %d.%d.%d.%d\t",(uint16_t)buf[0],(uint16_t)buf[1],(uint16_t)buf[2],(uint16_t)buf[3]);
        rt_kprintf("Port = %d\r\n",((buf[5]<<8) + buf[4]));
    }

    /* 断开中断，仅在TCP模式下有效 */
    if(sock_int_socket & SINT_STAT_DISCONNECT)
    {
        rt_kprintf("socket%d SINT_STAT_DISCONNECT \r\n",sockindex);
    }

    /* 超时中断，仅在TCP模式下有效*/
    if(sock_int_socket & SINT_STAT_TIM_OUT)
    {
        rt_kprintf("socket%d SINT_STAT_TIM_OUT\n",sockindex);
    }
}

/* CH395线程执行函数 */
void ch395_static_ip_test_entry(void *parameter)
{
	uint16_t ch395_status = 0;  		/* 获取中断事件状态 */
	uint8_t sockect_status = 0; 		/* 获取socket状态 */
	uint8_t  buf[20];					/* 缓存数据 */
	uint8_t RunStatue = DEF_PYH_CON;
	
	while(1)
	{
		if(RunStatue ==   DEF_PYH_CON)
		{
			 /* 等待以太网连接成功*/
			if(ch395_get_phy_status() != PHY_DISCONN)            /* 查询CH395是否连接 */
			{
				rt_kprintf("CH395 Connect Ethernet\r\n");
				RunStatue = DEF_SOCK_CON;

				//设置socket0为TCP模式、打开并监听，并设置1-7socket
				if(ch395_socket_tcp_server_init() == 0)
				{
					if(ch395_tcp_listen(0) == 0) //Socke 0 启动TCP监听
					{
						rt_kprintf("CH395TCPListen\r\n");
					}
				}
			}
		}

		if(RunStatue == DEF_SOCK_CON)
		{
			if(Query395Interrupt())
			{
				/*获取中断事件*/
				ch395_status = ch395_get_glob_int_status_all();

				/* 处理PHY改变中断*/
				if(ch395_status & GINT_STAT_PHY_CHANGE)
				{
					if(ch395_get_phy_status() == PHY_DISCONN)//网线断开
					{
						//获取socket0的状态ch395_get_socket_status
						ch395_get_socket_status(0, &sockect_status);

						//如果不等于关闭状态
						if(sockect_status != SOCKET_CLOSE)
						{
							//将所有socket进行关闭
							for(rt_size_t i = 0; i<8; i++)
							{
								ch395_close_socket(i);
							}
							rt_kprintf("CH395 PHY status : STAT_PHY_DISCONN\r\n");
						}
						RunStatue = DEF_PYH_CON;
					}
				}

				/* 处理DHCP/PPPOE中断 */
				if(ch395_status & GINT_STAT_DHCP)
				{

				}

				/* 处理不可达中断，读取不可达信息 */
				if(ch395_status & GINT_STAT_UNREACH)
				{
					ch395_get_unreach_ippt(buf,sizeof(buf));
				}

				/* 处理IP冲突中断，重新修改CH395的IP，并初始化CH395*/
				if(ch395_status & GINT_STAT_IP_CONFLI)
				{
					ch395_set_ip_addr(CH395IPAddr, sizeof(CH395IPAddr));            /* 设置CH395的IP地址 */
					ch395_set_gw_ip_addr(CH395GWIPAddr, sizeof(CH395GWIPAddr));     /* 设置网关地址 */
					ch395_set_mask_addr(CH395IPMask, sizeof(CH395IPMask));          /* 设置子网掩码，默认为255.255.255.0*/
					rt_thread_mdelay(100);

					/* 初始化CH395芯片模块 */
					while(!ch395_init())
					{
						rt_kprintf("ch395_init OK\r\n");
						break;
					}
				}

				/* 处理 socket中断 */
				for(rt_size_t i = 0; i<8; i++)
				{
					if(ch395_status & (1<<(i+4)))
					{
						//处理socket中断
						ch395_socket_tcp_client_interrupt(i);
					}
				}
			}
		}
	}
}

int spi_ch395_test(void)
{
	rt_thread_t th_ptr;
	struct rt_spi_device* CH395_spi_dev;
	uint8_t  buf[20];					/* 缓存id数据 */

    /* 查找 spi 设备获取设备句柄 */
    CH395_spi_dev = (struct rt_spi_device* )rt_device_find(SPI_DEVICE_NAME);
    if (!CH395_spi_dev)
    {
        rt_kprintf("spi sample run failed! can't find %s device!\n", SPI_DEVICE_NAME);
    }
    else
    {
         /* 配置SPI */
        struct rt_spi_configuration cfg;
        cfg.data_width = 8;
        cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_0  | RT_SPI_MSB;
        cfg.max_hz = 62000;     /* 主控是stm32F103,最大频率不超过35MHz */
        rt_spi_configure(CH395_spi_dev, &cfg);

        /* 初始化中断引脚 */
        rt_pin_mode(GET_PIN(C,13), PIN_MODE_INPUT_PULLUP);

        /* 初始化CH395设备句柄 */
        ch395_init_dev(CH395_spi_dev);
		
		/* 复位 */
        ch395_reset();
		rt_thread_mdelay(100);

        /* 获取版本号 */
        ch395_version = ch395_get_ver();
        rt_kprintf("CH395_VER: %x\r\n",ch395_version);

        /* 测试命令 */
        while(1)
        {
            if(ch395_check_exist(0x00) == 0xff)
            {
                rt_kprintf("ch395_check_exist OK\r\n");
                break;
            }
        }

        /* TCP SERVER支持多连接时，需初始化此启动参数 */
        ch395_set_start_para(FUN_PARA_FLAG_TCP_SERVER);
        rt_thread_mdelay(100);
 
        /* 配置 Socket 发送和接收缓存区大小 */
        socket_buffer_config();
        rt_thread_mdelay(100);

        ch395_set_ip_addr(CH395IPAddr, sizeof(CH395IPAddr));                              /* 设置CH395的IP地址 */
        ch395_set_gw_ip_addr(CH395GWIPAddr, sizeof(CH395GWIPAddr));                          /* 设置网关地址 */
        ch395_set_mask_addr(CH395IPMask, sizeof(CH395IPMask));                          /* 设置子网掩码，默认为255.255.255.0*/
        rt_thread_mdelay(100);

        /* 初始化CH395芯片模块 */
        while(!ch395_init())
        {
            rt_kprintf("ch395_init OK\r\n");
            break;
        }

        /* 查询一下设置的IP地址 */
        ch395_get_ip_inf(buf,sizeof(buf));      //获取IP，子网掩码和网关地址

        rt_kprintf("IP:%d.%d.%d.%d\r\n",buf[0],buf[1],buf[2],buf[3]);
        rt_kprintf("GWIP:%d.%d.%d.%d\r\n",buf[4],buf[5],buf[6],buf[7]);
        rt_kprintf("Mask:%d.%d.%d.%d\r\n",buf[8],buf[9],buf[10],buf[11]);
        rt_kprintf("DNS1:%d.%d.%d.%d\r\n",buf[12],buf[13],buf[14],buf[15]);
        rt_kprintf("DNS2:%d.%d.%d.%d\r\n",buf[16],buf[17],buf[18],buf[19]);
	}
	
	/* 创建线程 */
	th_ptr = rt_thread_create("ch395_static_ip_test_thread", ch395_static_ip_test_entry, CH395_spi_dev, 1024, 25, 5);
	if(th_ptr == NULL)
	{
		LOG_E("rt_thread_create failed....\r\n");
	}
	else {	
		/* 启动线程 */
		rt_thread_startup(th_ptr);
	}
	
	return 0;
}

int main(int argc,char* argv[])
{
	int count = 1;
	
	spi_ch395_test();
	
    while (count++)
    {
        rt_thread_mdelay(1000);
    }

    return RT_EOK;
}
