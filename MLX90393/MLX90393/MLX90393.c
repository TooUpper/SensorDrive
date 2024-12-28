#include "mlx90393.h"
        //MLX90393相关参数
          enum { STATUS_OK = 0, STATUS_ERROR = 0xff } return_status_t;
          enum { Z_FLAG = 0x8, Y_FLAG = 0x4, X_FLAG = 0x2, T_FLAG = 0x1 } axis_flag_t;
          enum { I2C_BASE_ADDR = 0x0c };
          enum { GAIN_SEL_REG = 0x0, GAIN_SEL_MASK = 0x0070, GAIN_SEL_SHIFT = 4 };
          enum { HALLCONF_REG = 0x0, HALLCONF_MASK = 0x000f, HALLCONF_SHIFT = 0 };
          enum { BURST_SEL_REG = 0x1, BURST_SEL_MASK = 0x03c0, BURST_SEL_SHIFT = 6};
          enum { TRIG_INT_SEL_REG = 0x1, TRIG_INT_SEL_MASK = 0x8000, TRIG_INT_SEL_SHIFT = 15 };
          enum { EXT_TRIG_REG = 0x1, EXT_TRIG_MASK = 0x0800, EXT_TRIG_SHIFT = 11 };
          enum { OSR_REG = 0x2, OSR_MASK = 0x0003, OSR_SHIFT = 0 };
          enum { OSR2_REG = 0x2, OSR2_MASK = 0x1800, OSR2_SHIFT = 11 };
          enum { DIG_FLT_REG = 0x2, DIG_FLT_MASK = 0x001c, DIG_FLT_SHIFT = 2 };
          enum { RES_XYZ_REG = 0x2, RES_XYZ_MASK = 0x07e0, RES_XYZ_SHIFT = 5 };
          enum { TCMP_EN_REG = 0x1, TCMP_EN_MASK = 0x0400, TCMP_EN_SHIFT = 10 };
          enum { X_OFFSET_REG = 4, Y_OFFSET_REG = 5, Z_OFFSET_REG = 6 };
          enum { WOXY_THRESHOLD_REG = 7, WOZ_THRESHOLD_REG = 8, WOT_THRESHOLD_REG = 9 };
          enum { BURST_MODE_BIT = 0x80, WAKE_ON_CHANGE_BIT = 0x40,\
                   POLLING_MODE_BIT = 0x20, ERROR_BIT = 0x10, EEC_BIT = 0x08,\
                   RESET_BIT = 0x04, D1_BIT = 0x02, D0_BIT = 0x01 };

        struct txyz
        {
                float t;
                float x;
                float y;
                float z;
        };
        struct txyzRaw
        {
                uint16_t t;
                uint16_t x;
                uint16_t y;
                uint16_t z;
        };

        uint8_t SPI_RX[16]={0};
        //uint8_t MLX_MODE=0;
        //uint8_t MLX_REG_R[3][2]={0x00,0x7c,0x03,0xff,0x1f,0xff};
        uint8_t MLX_REG_R[3][2]={0x00,0x70,\
                                         0x83,0xc8,\
                                         0x18,0x1F};

        float gain_multipliers[8]={5.0,4.0,3.0,2.5,2.0,1.66666667,1.33333333,1.0};
        // from datasheet for hallconf = 0
        float base_xy_sens_hc0 = 0.196;
        float base_z_sens_hc0 = 0.316;
        // for hallconf = 0xc
        float base_xy_sens_hc0xc = 0.150;
        float base_z_sens_hc0xc = 0.242;
        const uint8_t IIC_ADD_W=0x18;
        const uint8_t IIC_ADD_R=0x19;
        // SPI commands begin
        uint8_t MLX_EX=0x80;
        uint8_t MLX_SB=0x1F;
        uint8_t MLX_SWOC=0x2F;
        uint8_t MLX_SM=0x3F;
        uint8_t MLX_RM=0x4F;
        uint8_t MLX_RR[2]={0x50,0x00};
        //        RR:0101 0abc,  {A5…A0,0,0}
        uint8_t MLX_WR[4]={0x60,0x55,0x33,0x04};
        //         WR:0110 0abc,D15...D0,{A5…A0,0,0}
        uint8_t MLX_RT=0xF0;
        uint8_t MLX_HR=0xD0;
        uint8_t MLX_HS=0xE0;
        uint8_t MLX_NOP=0x00;

/******************************************************************
 * 函 数 名 称：convertRaw
 * 函 数 说 明：对原始数据进行转换
 * 函 数 形 参：raw，原始数据，整形
 * 函 数 返 回：data，转换后的浮点数据
 * 作       者：KWIN
 * 备       注：
******************************************************************/
struct txyz convertRaw(struct txyzRaw raw)
{
  const uint8_t gain_sel = (MLX_REG_R[GAIN_SEL_REG][1] & GAIN_SEL_MASK) >> GAIN_SEL_SHIFT;
  const uint8_t hallconf = (MLX_REG_R[HALLCONF_REG][1] & HALLCONF_MASK) >> HALLCONF_SHIFT;
  const uint8_t res_xyz = ((MLX_REG_R[RES_XYZ_REG][0] * 256 + MLX_REG_R[RES_XYZ_REG][1]) & RES_XYZ_MASK) >> RES_XYZ_SHIFT;
  const uint8_t res_x = (res_xyz >> 0) & 0x3;
  const uint8_t res_y = (res_xyz >> 2) & 0x3;
  const uint8_t res_z = (res_xyz >> 4) & 0x3;
  uint8_t tcmp_en = MLX_REG_R[TCMP_EN_REG][0] & 0x04;
  float gain_factor = gain_multipliers[gain_sel & 0x7];
  struct txyz data;
  float xy_sens;
  float z_sens;

  switch(hallconf){
  default:
  case 0:
    xy_sens = base_xy_sens_hc0;
    z_sens = base_z_sens_hc0;
    break;
  case 0xc:
    xy_sens = base_xy_sens_hc0xc;
    z_sens = base_z_sens_hc0xc;
    break;
  }



  if (tcmp_en){
    data.x = ( (raw.x - 32768.f) * xy_sens *
               gain_factor * (1 << res_x) );
  } else {
    switch(res_x){
    case 0:
    case 1:
      data.x = (int16_t)(raw.x) * xy_sens * gain_factor * (1 << res_x);
      break;
    case 2:
      data.x = ( (raw.x - 32768.f) * xy_sens *
                 gain_factor * (1 << res_x) );
      break;
    case 3:
      data.x = ( (raw.x - 16384.f) * xy_sens *
                 gain_factor * (1 << res_x) );
      break;
    }
  }

  if (tcmp_en){
    data.y = ( (raw.y - 32768.f) * xy_sens *
               gain_factor * (1 << res_y) );
  } else {
    switch(res_y){
    case 0:
    case 1:
      data.y = (int16_t)(raw.y) * xy_sens * gain_factor * (1 << res_y);
      break;
    case 2:
      data.y = ( (raw.y - 32768.f) * xy_sens *
                 gain_factor * (1 << res_y) );
      break;
    case 3:
      data.y = ( (raw.y - 16384.f) * xy_sens *
                 gain_factor * (1 << res_y) );
      break;
    }
  }

  if (tcmp_en){
    data.z = ( (raw.z - 32768.f) * z_sens *
               gain_factor * (1 << res_z) );
  } else {
    switch(res_z){
    case 0:
    case 1:
      data.z = (int16_t)(raw.z) * z_sens * gain_factor * (1 << res_z);
      break;
    case 2:
      data.z = ( (raw.z - 32768.f) * z_sens *
                 gain_factor * (1 << res_z) );
      break;
    case 3:
      data.z = ( (raw.z - 16384.f) * z_sens *
                 gain_factor * (1 << res_z) );
      break;
    }
  }

  data.t = 25 + (raw.t - 46244.f)/45.2f;
  return data;
}

/******************************************************************
 * 函 数 名 称：MLX90393_Init
 * 函 数 说 明：MLX90393进行初始化
 * 函 数 形 参：
 * 函 数 返 回：无
 * 作       者：kwin
 * 备       注：无
******************************************************************/
void MLX90393_Init(void)
{
        uint8_t tmp;
        ///////////////////////////////////////////EX
        EX:
        {
                IIC_Start();
                IIC_Send_Byte(IIC_ADD_W);
                if(IIC_Wait_Ack())
                {
                        printf("ADDW-Wrong\r\n");
                        IIC_Stop();
                        return;
                }
                IIC_Send_Byte(MLX_EX);
                if(IIC_Wait_Ack())
                {
                        printf("EX-Wrong\r\n");
                        IIC_Stop();
                        return;
                }
                IIC_Start();
                IIC_Send_Byte(IIC_ADD_R);
                if(IIC_Wait_Ack())
                {
                        printf("ADDR-Wrong\r\n");
                        IIC_Stop();
                        return;
                }
                tmp=IIC_Read_Byte();
                IIC_NAck();
                IIC_Stop();
                printf("EX-Status:0x%02X\r\n",tmp);
                delay_1ms(5);

        }
                ///////////////////////////////////////////RT
        RT:
        {
                IIC_Start();
                IIC_Send_Byte(IIC_ADD_W);
                if(IIC_Wait_Ack())
                {
                        printf("ADDW-Wrong\r\n");
                        IIC_Stop();
                        return;
                }
                IIC_Send_Byte(MLX_RT);
                if(IIC_Wait_Ack())
                {
                        printf("RT-Wrong\r\n");
                        IIC_Stop();
                        return;
                }
                IIC_Start();
                IIC_Send_Byte(IIC_ADD_R);
                if(IIC_Wait_Ack())
                {
                        printf("ADDR-Wrong\r\n");
                        IIC_Stop();
                        return;
                }
                tmp=IIC_Read_Byte();
                IIC_NAck();
                IIC_Stop();
                printf("RT-Status:0x%02X\r\n",tmp);
                delay_1ms(10);
        }

        ///////////////////////////////////////////WR 0
        WR0:
        {
                MLX_WR[0]=0x60;MLX_WR[1]=MLX_REG_R[0][0];MLX_WR[2]=MLX_REG_R[0][1];MLX_WR[3]=0x00;
                IIC_Start();
                IIC_Send_Byte(IIC_ADD_W);
                if(IIC_Wait_Ack())
                {
                        printf("ADDW-Wrong\r\n");
                        IIC_Stop();
                        return;
                }
                IIC_Send_Byte(MLX_WR[0]);
                if(IIC_Wait_Ack())
                {
                        printf("WR0-Wrong\r\n");
                        IIC_Stop();
                        return;
                }
                IIC_Send_Byte(MLX_WR[1]);
                if(IIC_Wait_Ack())
                {
                        printf("WR1-Wrong\r\n");
                        IIC_Stop();
                        return;
                }
                IIC_Send_Byte(MLX_WR[2]);
                if(IIC_Wait_Ack())
                {
                        printf("WR2-Wrong\r\n");
                        IIC_Stop();
                        return;
                }
                IIC_Send_Byte(MLX_WR[3]);
                if(IIC_Wait_Ack())
                {
                        printf("WR3-Wrong\r\n");
                        IIC_Stop();
                        return;
                }
                IIC_Start();
                IIC_Send_Byte(IIC_ADD_R);
                if(IIC_Wait_Ack())
                {
                        printf("ADDR-Wrong\r\n");
                        IIC_Stop();
                        return;
                }
                tmp=IIC_Read_Byte();
                IIC_NAck();
                IIC_Stop();
                printf("0WR-Status:0x%02X\r\n",tmp);
                delay_1ms(1);


        }
        ///////////////////////////////////////////WR 1
        WR1:
        {
                MLX_WR[0]=0x60;MLX_WR[1]=MLX_REG_R[1][0];MLX_WR[2]=MLX_REG_R[1][1];MLX_WR[3]=0x04;
                IIC_Start();
                IIC_Send_Byte(IIC_ADD_W);
                if(IIC_Wait_Ack())
                {
                        printf("ADDW-Wrong\r\n");
                        IIC_Stop();
                        return;
                }
                IIC_Send_Byte(MLX_WR[0]);
                if(IIC_Wait_Ack())
                {
                        printf("WR0-Wrong\r\n");
                        IIC_Stop();
                        return;
                }
                IIC_Send_Byte(MLX_WR[1]);
                if(IIC_Wait_Ack())
                {
                        printf("WR1-Wrong\r\n");
                        IIC_Stop();
                        return;
                }
                IIC_Send_Byte(MLX_WR[2]);
                if(IIC_Wait_Ack())
                {
                        printf("WR2-Wrong\r\n");
                        IIC_Stop();
                        return;
                }
                IIC_Send_Byte(MLX_WR[3]);
                if(IIC_Wait_Ack())
                {
                        printf("WR3-Wrong\r\n");
                        IIC_Stop();
                        return;
                }
                IIC_Start();
                IIC_Send_Byte(IIC_ADD_R);
                if(IIC_Wait_Ack())
                {
                        printf("ADDR-Wrong\r\n");
                        IIC_Stop();
                        return;
                }
                tmp=IIC_Read_Byte();
                IIC_NAck();
                IIC_Stop();
                printf("1WR-Status:0x%02X\r\n",tmp);
                delay_1ms(1);

        }
        ///////////////////////////////////////////WR 2
        WR2:
        {
                MLX_WR[0]=0x60;MLX_WR[1]=MLX_REG_R[2][0];MLX_WR[2]=MLX_REG_R[2][1];MLX_WR[3]=0x08;
                IIC_Start();
                IIC_Send_Byte(IIC_ADD_W);
                if(IIC_Wait_Ack())
                {
                        printf("ADDW-Wrong\r\n");
                        IIC_Stop();
                        return;
                }
                IIC_Send_Byte(MLX_WR[0]);
                if(IIC_Wait_Ack())
                {
                        printf("WR0-Wrong\r\n");
                        IIC_Stop();
                        return;
                }
                IIC_Send_Byte(MLX_WR[1]);
                if(IIC_Wait_Ack())
                {
                        printf("WR1-Wrong\r\n");
                        IIC_Stop();
                        return;
                }
                IIC_Send_Byte(MLX_WR[2]);
                if(IIC_Wait_Ack())
                {
                        printf("WR2-Wrong\r\n");
                        IIC_Stop();
                        return;
                }
                IIC_Send_Byte(MLX_WR[3]);
                if(IIC_Wait_Ack())
                {
                        printf("WR3-Wrong\r\n");
                        IIC_Stop();
                        return;
                }
                IIC_Start();
                IIC_Send_Byte(IIC_ADD_R);
                if(IIC_Wait_Ack())
                {
                        printf("ADDR-Wrong\r\n");
                        IIC_Stop();
                        return;
                }
                tmp=IIC_Read_Byte();
                IIC_NAck();
                IIC_Stop();
                printf("2WR-Status:0x%02X\r\n",tmp);
                delay_1ms(10);
        }
        ///////////////////////////////////////////SB
        MLX_SB=0x1F;// output TXYZ
        IIC_Start();
        IIC_Send_Byte(IIC_ADD_W);
        if(IIC_Wait_Ack())
        {
                printf("ADDW-Wrong\r\n");
                IIC_Stop();
                return;
        }
        IIC_Send_Byte(MLX_SB);
        if(IIC_Wait_Ack())
        {
                printf("SB-Wrong\r\n");
                IIC_Stop();
                return;
        }
        IIC_Start();
        IIC_Send_Byte(IIC_ADD_R);
        if(IIC_Wait_Ack())
        {
                printf("ADDR-Wrong\r\n");
                IIC_Stop();
                return;
        }
        tmp=IIC_Read_Byte();
        IIC_NAck();
        IIC_Stop();
        printf("SB-Status:0x%02X\r\n",tmp);
        delay_1ms(1);

}

/******************************************************************
 * 函 数 名 称：EXTI10_15_IRQHandler
 * 函 数 说 明：轮询查找IIC地址
 * 函 数 形 参：
 * 函 数 返 回：无
 * 作       者：kwin
 * 备       注：无
******************************************************************/
void EXTI10_15_IRQHandler(void)
{
        uint8_t tmp;
        struct txyzRaw myraw;
        struct txyz mytxyz;
        if(RESET != exti_interrupt_flag_get(EXTI_13))
        {
                IIC_Start();
                IIC_Send_Byte(IIC_ADD_W);
                if(IIC_Wait_Ack())
                {
                        printf("ADDW-Wrong\r\n");
                        IIC_Stop();
                        return;
                }
                IIC_Send_Byte(MLX_RM);
                if(IIC_Wait_Ack())
                {
                        printf("RM-Wrong\r\n");
                        IIC_Stop();
                        return;
                }

                IIC_Start();
                IIC_Send_Byte(IIC_ADD_R);
                if(IIC_Wait_Ack())
                {
                        printf("ADDR-Wrong\r\n");
                        IIC_Stop();
                        return;
                }
                tmp=IIC_Read_Byte();
                IIC_Ack();
                myraw.t=IIC_Read_Byte();
                IIC_Ack();
                myraw.t<<=8;
                myraw.t+=IIC_Read_Byte();
                IIC_Ack();

                myraw.x=IIC_Read_Byte();
                IIC_Ack();
                myraw.x<<=8;
                myraw.x+=IIC_Read_Byte();
                IIC_Ack();

                myraw.y=IIC_Read_Byte();
                IIC_Ack();
                myraw.y<<=8;
                myraw.y+=IIC_Read_Byte();
                IIC_Ack();

                myraw.z=IIC_Read_Byte();
                IIC_Ack();
                myraw.z<<=8;
                myraw.z+=IIC_Read_Byte();
                IIC_NAck();
                IIC_Stop();
                mytxyz=convertRaw(myraw);
                printf("RM-Status:0x%02X\r\n",tmp);
                printf("T;%4.1f,X:%4.1f,Y:%4.1f,Z:%4.1f\r\n",mytxyz.t,mytxyz.x,mytxyz.y,mytxyz.z);
                gpio_bit_toggle(PORT_LED2,PIN_LED2);
                exti_interrupt_flag_clear(EXTI_13);

    }
}

/******************************************************************
 * 函 数 名 称：get_IIC_ADD_W
 * 函 数 说 明：中断查询数据并打印
 * 函 数 形 参：
 * 函 数 返 回：无
 * 作       者：kwin
 * 备       注：无
******************************************************************/
void get_iic_addr(void)
{
        uint8_t cnt;
        for(cnt=0x0c;cnt<0x1c;cnt++)
        {
                IIC_Start();
                IIC_Send_Byte(cnt);
                if(!IIC_Wait_Ack())
                {
                        printf("ADDR:0X%02X",cnt);
                        IIC_Stop();
                        break;
                }
                IIC_Stop();
        }

}

/******************************************************************
 * 函 数 名 称：get_data
 * 函 数 说 明：主动查询数据并打印
 * 函 数 形 参：
 * 函 数 返 回：无
 * 作       者：kwin
 * 备       注：无
******************************************************************/
void get_data(void)
{
        uint8_t tmp;
        struct txyzRaw myraw;
        struct txyz mytxyz;
        IIC_Start();
        IIC_Send_Byte(IIC_ADD_W);
        if(IIC_Wait_Ack())
        {
                printf("ADDW-Wrong\r\n");
                IIC_Stop();
                return;
        }
        IIC_Send_Byte(MLX_RM);
        if(IIC_Wait_Ack())
        {
                printf("RM-Wrong\r\n");
                IIC_Stop();
                return;
        }
        IIC_Start();
        IIC_Send_Byte(IIC_ADD_R);
        if(IIC_Wait_Ack())
        {
                printf("ADDR-Wrong\r\n");
                IIC_Stop();
                return;
        }
        tmp=IIC_Read_Byte();
        IIC_Ack();
        myraw.t=IIC_Read_Byte();
        IIC_Ack();
        myraw.t<<=8;
        myraw.t+=IIC_Read_Byte();
        IIC_Ack();

        myraw.x=IIC_Read_Byte();
        IIC_Ack();
        myraw.x<<=8;
        myraw.x+=IIC_Read_Byte();
        IIC_Ack();

        myraw.y=IIC_Read_Byte();
        IIC_Ack();
        myraw.y<<=8;
        myraw.y+=IIC_Read_Byte();
        IIC_Ack();

        myraw.z=IIC_Read_Byte();
        IIC_Ack();
        myraw.z<<=8;
        myraw.z+=IIC_Read_Byte();
        IIC_NAck();

        IIC_Stop();

        mytxyz=convertRaw(myraw);
        printf("RM-Status:0x%02X\r\n",tmp);
        printf("T;%4.1f,X:%4.1f,Y:%4.1f,Z:%4.1f\r\n",mytxyz.t,mytxyz.x,mytxyz.y,mytxyz.z);

}