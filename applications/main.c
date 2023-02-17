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

/* CH395��ض��� */
uint8_t CH395IPAddr[4] = {192,168,1,200};                         /* CH395IP��ַ */
uint8_t CH395GWIPAddr[4] = {192,168,1,1};                        /* CH395���� */
uint8_t CH395IPMask[4] = {255,255,255,0};                        /* CH395�������� */

/* socket ��ض���*/
uint16_t SocketServerPort   = 8080;           /* Socket Ŀ�Ķ˿� */

/*�洢������յ�����*/
#define recv_buff_len 1500
uint8_t recv_buff[recv_buff_len];

/* �汾�� */
uint8_t ch395_version = 0;
uint8_t buf[6] = {0};

//INT  -- ����ģ��INT���� (��⵽�ж��ź�֮���ٻ�ȡ����)
#define CH395_INT_PORT  GPIOC
#define CH395_INT_PIN   GPIO_PIN_13

#define CH395_INT_PIN_INPUT()    (CH395_INT_PORT->IDR & CH395_INT_PIN)   /* ��ȡINT��ƽ */


/* ����   CH395 �� SPI ���� */
static int rt_hw_spi_CH395_init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();

    rt_hw_spi_device_attach("spi1", "spi10", GPIOA, GPIO_PIN_4);

    return RT_EOK;
}
/* �������Զ���ʼ�� */
INIT_COMPONENT_EXPORT(rt_hw_spi_CH395_init);

/*******************************************************************************
* Function Name  : Query395Interrupt
* Description    : ��ѯCH395�ж�(INT#�͵�ƽ)
* Input          : None
* Output         : None
* Return         : �����ж�״̬
*******************************************************************************/
uint8_t Query395Interrupt(void)
{
    return( CH395_INT_PIN_INPUT() ? 0 : 1 );
}

/*���� Socket ���ͺͽ��ջ�����*/
/*оƬ��0-47����,ÿ����512�ֽڴ�С*/
void socket_buffer_config(void )
{
    /*Socket 0*/
    ch395_set_socket_recv_buf(0,0,4);//��0,1,2,3�黺������ΪSocket�Ľ��ջ�����(512*4=2KB)
    ch395_set_socket_send_buf(0,4,2);//��4,5�黺������ΪSocket�ķ��ͻ�����(512*2=1KB)
    /*Socket 1*/
    ch395_set_socket_recv_buf(1,6,4);//��6,7,8,9�黺������ΪSocket�Ľ��ջ�����(512*4=2KB)
    ch395_set_socket_send_buf(1,10,2);//��10,11�黺������ΪSocket�ķ��ͻ�����(512*2=1KB)
    /*Socket 2*/
    ch395_set_socket_recv_buf(2,12,4);//��12,13,14,15�黺������ΪSocket�Ľ��ջ�����(512*4=2KB)
    ch395_set_socket_send_buf(2,16,2);//��16,17�黺������ΪSocket�ķ��ͻ�����(512*2=1KB)
    /*Socket 3*/
    ch395_set_socket_recv_buf(3,18,4);//��18,19,20,21�黺������ΪSocket�Ľ��ջ�����(512*4=2KB)
    ch395_set_socket_send_buf(3,22,2);//��22,23�黺������ΪSocket�ķ��ͻ�����(512*2=1KB)

    /*Ӳ���汾����4�Ĳ���Socket4-7*/
  if(ch395_version >= 0x44)
    {
        /*Socket 4*/
        ch395_set_socket_recv_buf(4,24,4);//��24,25,26,27�黺������ΪSocket�Ľ��ջ�����(512*4=2KB)
        ch395_set_socket_send_buf(4,28,2);//��28,29�黺������ΪSocket�ķ��ͻ�����(512*2=1KB)
        /*Socket 5*/
        ch395_set_socket_recv_buf(5,30,4);//��30,31,32,33�黺������ΪSocket�Ľ��ջ�����(512*4=2KB)
        ch395_set_socket_send_buf(5,34,2);//��34,35�黺������ΪSocket�ķ��ͻ�����(512*2=1KB)

        /*Socket 6*/
        ch395_set_socket_recv_buf(6,36,4);//��36,37,38,39�黺������ΪSocket�Ľ��ջ�����(512*4=2KB)
        ch395_set_socket_send_buf(6,40,2);//��40,41�黺������ΪSocket�ķ��ͻ�����(512*2=1KB)

        /*Socket 7*/
        ch395_set_socket_recv_buf(7,42,4);//��42,43,44,45�黺������ΪSocket�Ľ��ջ�����(512*4=2KB)
        ch395_set_socket_send_buf(7,46,2);//��46,47�黺������ΪSocket�ķ��ͻ�����(512*2=1KB)
    }
}

/*******************************************************************************
* Function Name  : ch395_socket_tcp_server_init
* Description    : ��ʼ��socket
* Input          : None
* Output         : None
* Return         : 0:��ʼ���ɹ�; others:��ʼ��ʧ��
*******************************************************************************/
char ch395_socket_tcp_server_init(void)
{
    /*��Socket��Ϊ��������*/
    ch395_set_socket_prot_type(0,PROTO_TYPE_TCP);               /* Э������ */
    ch395_set_socket_sour_port(0,SocketServerPort);             /* ���ض˿ں� */
    if(ch395_open_socket(0) !=0)                                /* ��Socket */
    {
        return 1;
    }

    /*����Socket��Ϊ����ͨ��*/
    /*��Ҫ��·Socket�ͻ�������ͨ��,����Ҫ���ü���Socket,����ģ�����֧��7��Socket TCP�ͻ���ͨ��*/
    ch395_set_socket_prot_type(1,PROTO_TYPE_TCP); /* Э������ */
    ch395_set_socket_sour_port(1,SocketServerPort);/* ���ض˿ں� */

    ch395_set_socket_prot_type(2,PROTO_TYPE_TCP); /* Э������ */
    ch395_set_socket_sour_port(2,SocketServerPort);/* ���ض˿ں� */

    ch395_set_socket_prot_type(3,PROTO_TYPE_TCP); /* Э������ */
    ch395_set_socket_sour_port(3,SocketServerPort);/* ���ض˿ں� */

    ch395_set_socket_prot_type(4,PROTO_TYPE_TCP); /* Э������ */
    ch395_set_socket_sour_port(4,SocketServerPort);/* ���ض˿ں� */

    ch395_set_socket_prot_type(5,PROTO_TYPE_TCP); /* Э������ */
    ch395_set_socket_sour_port(5,SocketServerPort);/* ���ض˿ں� */

    ch395_set_socket_prot_type(6,PROTO_TYPE_TCP); /* Э������ */
    ch395_set_socket_sour_port(6,SocketServerPort);/* ���ض˿ں� */

    ch395_set_socket_prot_type(7,PROTO_TYPE_TCP); /* Э������ */
    ch395_set_socket_sour_port(7,SocketServerPort);/* ���ض˿ں� */

    return 0;
}

/*******************************************************************************
* Function Name  : ch395_socket_tcp_client_interrupt
* Description    : socket�жϴ�����
* Input          : sockindex  Socket����(0,1,2,3,4,5,6,7)
* Output         : None
* Return         : None
*******************************************************************************/
void ch395_socket_tcp_client_interrupt(uint8_t sockindex)
{
    uint8_t  sock_int_socket;
    uint16_t len;

    /* ��ȡsocket ���ж�״̬ */
    sock_int_socket = ch395_get_socket_int(sockindex);

    /* ���ͻ��������У����Լ���д��Ҫ���͵����� */
    if(sock_int_socket & SINT_STAT_SENBUF_FREE)
    {
    }

    /* ��������ж� */
    if(sock_int_socket & SINT_STAT_SEND_OK)
    {
    }

    /* ���������ж� */
    if(sock_int_socket & SINT_STAT_RECV)
    {
        len = ch395_get_recv_length(sockindex);/* ��ȡ��ǰ�����������ݳ��� */
        if(len == 0)return;
        if(len > recv_buff_len)
        {
            len = recv_buff_len;
        }
        ch395_get_recv_data(sockindex,len,recv_buff);/* ��ȡ���� */

        /* �ѽ��յ����ݷ��͸��ͻ��� */
        ch395_send_data(sockindex,recv_buff,len);

		rt_kprintf("socket%d recv data len:%d   ",sockindex, len);
        
		/* ʹ�ô��ڴ�ӡ���յ����� */
		rt_kprintf("data:");
        for(int i = 0; i < len ;++i)
        {
            rt_kprintf("%c",recv_buff[i]);
        }
        rt_kprintf("\r\n");
    }

    /* �����жϣ�����TCPģʽ����Ч*/
    if(sock_int_socket & SINT_STAT_CONNECT)
    {
        rt_kprintf("socket%d SINT_STAT_CONNECT\r\n",sockindex);

        ch395_get_remote_ipp(sockindex,buf,sizeof(buf));//��ȡ�ͻ�����Ϣ

        rt_kprintf("IP address = %d.%d.%d.%d\t",(uint16_t)buf[0],(uint16_t)buf[1],(uint16_t)buf[2],(uint16_t)buf[3]);
        rt_kprintf("Port = %d\r\n",((buf[5]<<8) + buf[4]));
    }

    /* �Ͽ��жϣ�����TCPģʽ����Ч */
    if(sock_int_socket & SINT_STAT_DISCONNECT)
    {
        rt_kprintf("socket%d SINT_STAT_DISCONNECT \r\n",sockindex);
    }

    /* ��ʱ�жϣ�����TCPģʽ����Ч*/
    if(sock_int_socket & SINT_STAT_TIM_OUT)
    {
        rt_kprintf("socket%d SINT_STAT_TIM_OUT\n",sockindex);
    }
}

/* CH395�߳�ִ�к��� */
void ch395_static_ip_test_entry(void *parameter)
{
	uint16_t ch395_status = 0;  		/* ��ȡ�ж��¼�״̬ */
	uint8_t sockect_status = 0; 		/* ��ȡsocket״̬ */
	uint8_t  buf[20];					/* �������� */
	uint8_t RunStatue = DEF_PYH_CON;
	
	while(1)
	{
		if(RunStatue ==   DEF_PYH_CON)
		{
			 /* �ȴ���̫�����ӳɹ�*/
			if(ch395_get_phy_status() != PHY_DISCONN)            /* ��ѯCH395�Ƿ����� */
			{
				rt_kprintf("CH395 Connect Ethernet\r\n");
				RunStatue = DEF_SOCK_CON;

				//����socket0ΪTCPģʽ���򿪲�������������1-7socket
				if(ch395_socket_tcp_server_init() == 0)
				{
					if(ch395_tcp_listen(0) == 0) //Socke 0 ����TCP����
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
				/*��ȡ�ж��¼�*/
				ch395_status = ch395_get_glob_int_status_all();

				/* ����PHY�ı��ж�*/
				if(ch395_status & GINT_STAT_PHY_CHANGE)
				{
					if(ch395_get_phy_status() == PHY_DISCONN)//���߶Ͽ�
					{
						//��ȡsocket0��״̬ch395_get_socket_status
						ch395_get_socket_status(0, &sockect_status);

						//��������ڹر�״̬
						if(sockect_status != SOCKET_CLOSE)
						{
							//������socket���йر�
							for(rt_size_t i = 0; i<8; i++)
							{
								ch395_close_socket(i);
							}
							rt_kprintf("CH395 PHY status : STAT_PHY_DISCONN\r\n");
						}
						RunStatue = DEF_PYH_CON;
					}
				}

				/* ����DHCP/PPPOE�ж� */
				if(ch395_status & GINT_STAT_DHCP)
				{

				}

				/* �����ɴ��жϣ���ȡ���ɴ���Ϣ */
				if(ch395_status & GINT_STAT_UNREACH)
				{
					ch395_get_unreach_ippt(buf,sizeof(buf));
				}

				/* ����IP��ͻ�жϣ������޸�CH395��IP������ʼ��CH395*/
				if(ch395_status & GINT_STAT_IP_CONFLI)
				{
					ch395_set_ip_addr(CH395IPAddr, sizeof(CH395IPAddr));            /* ����CH395��IP��ַ */
					ch395_set_gw_ip_addr(CH395GWIPAddr, sizeof(CH395GWIPAddr));     /* �������ص�ַ */
					ch395_set_mask_addr(CH395IPMask, sizeof(CH395IPMask));          /* �����������룬Ĭ��Ϊ255.255.255.0*/
					rt_thread_mdelay(100);

					/* ��ʼ��CH395оƬģ�� */
					while(!ch395_init())
					{
						rt_kprintf("ch395_init OK\r\n");
						break;
					}
				}

				/* ���� socket�ж� */
				for(rt_size_t i = 0; i<8; i++)
				{
					if(ch395_status & (1<<(i+4)))
					{
						//����socket�ж�
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
	uint8_t  buf[20];					/* ����id���� */

    /* ���� spi �豸��ȡ�豸��� */
    CH395_spi_dev = (struct rt_spi_device* )rt_device_find(SPI_DEVICE_NAME);
    if (!CH395_spi_dev)
    {
        rt_kprintf("spi sample run failed! can't find %s device!\n", SPI_DEVICE_NAME);
    }
    else
    {
         /* ����SPI */
        struct rt_spi_configuration cfg;
        cfg.data_width = 8;
        cfg.mode = RT_SPI_MASTER | RT_SPI_MODE_0  | RT_SPI_MSB;
        cfg.max_hz = 62000;     /* ������stm32F103,���Ƶ�ʲ�����35MHz */
        rt_spi_configure(CH395_spi_dev, &cfg);

        /* ��ʼ���ж����� */
        rt_pin_mode(GET_PIN(C,13), PIN_MODE_INPUT_PULLUP);

        /* ��ʼ��CH395�豸��� */
        ch395_init_dev(CH395_spi_dev);
		
		/* ��λ */
        ch395_reset();
		rt_thread_mdelay(100);

        /* ��ȡ�汾�� */
        ch395_version = ch395_get_ver();
        rt_kprintf("CH395_VER: %x\r\n",ch395_version);

        /* �������� */
        while(1)
        {
            if(ch395_check_exist(0x00) == 0xff)
            {
                rt_kprintf("ch395_check_exist OK\r\n");
                break;
            }
        }

        /* TCP SERVER֧�ֶ�����ʱ�����ʼ������������ */
        ch395_set_start_para(FUN_PARA_FLAG_TCP_SERVER);
        rt_thread_mdelay(100);
 
        /* ���� Socket ���ͺͽ��ջ�������С */
        socket_buffer_config();
        rt_thread_mdelay(100);

        ch395_set_ip_addr(CH395IPAddr, sizeof(CH395IPAddr));                              /* ����CH395��IP��ַ */
        ch395_set_gw_ip_addr(CH395GWIPAddr, sizeof(CH395GWIPAddr));                          /* �������ص�ַ */
        ch395_set_mask_addr(CH395IPMask, sizeof(CH395IPMask));                          /* �����������룬Ĭ��Ϊ255.255.255.0*/
        rt_thread_mdelay(100);

        /* ��ʼ��CH395оƬģ�� */
        while(!ch395_init())
        {
            rt_kprintf("ch395_init OK\r\n");
            break;
        }

        /* ��ѯһ�����õ�IP��ַ */
        ch395_get_ip_inf(buf,sizeof(buf));      //��ȡIP��������������ص�ַ

        rt_kprintf("IP:%d.%d.%d.%d\r\n",buf[0],buf[1],buf[2],buf[3]);
        rt_kprintf("GWIP:%d.%d.%d.%d\r\n",buf[4],buf[5],buf[6],buf[7]);
        rt_kprintf("Mask:%d.%d.%d.%d\r\n",buf[8],buf[9],buf[10],buf[11]);
        rt_kprintf("DNS1:%d.%d.%d.%d\r\n",buf[12],buf[13],buf[14],buf[15]);
        rt_kprintf("DNS2:%d.%d.%d.%d\r\n",buf[16],buf[17],buf[18],buf[19]);
	}
	
	/* �����߳� */
	th_ptr = rt_thread_create("ch395_static_ip_test_thread", ch395_static_ip_test_entry, CH395_spi_dev, 1024, 25, 5);
	if(th_ptr == NULL)
	{
		LOG_E("rt_thread_create failed....\r\n");
	}
	else {	
		/* �����߳� */
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
