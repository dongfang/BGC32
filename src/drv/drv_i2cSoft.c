///////////////////////////////////////////////////////////////////////////////

#include "board.h"

///////////////////////////////////////////////////////////////////////////////

int I2Cerror = 0;
int I2Cerrorcount = 0;

#define I2C_SDA_PIN     GPIO_Pin_11
#define I2C_SDA_PORT    GPIOB

#define I2C_SCL_PIN     GPIO_Pin_10
#define I2C_SCL_PORT    GPIOB

#define SDAH()  GPIO_WriteBit(I2C_SDA_PORT, I2C_SDA_PIN,   Bit_SET)
#define SDAL()  GPIO_WriteBit(I2C_SDA_PORT, I2C_SDA_PIN,   Bit_RESET)

#define SCLH()  GPIO_WriteBit(I2C_SCL_PORT, I2C_SCL_PIN,   Bit_SET)
#define SCLL()  GPIO_WriteBit(I2C_SCL_PORT, I2C_SCL_PIN,   Bit_RESET)

///////////////////////////////////////

void I2C_delay(void)
{
    delayMicroseconds(1);
}

///////////////////////////////////////

void I2C_Start(void)
{
    SDAH();
    SCLH();
    I2C_delay();
    SDAL();
    I2C_delay();
    SCLL();
    I2C_delay();

}

///////////////////////////////////////

void I2C_Stop(void)
{
    SCLL();
    I2C_delay();
    SDAL();
    I2C_delay();
    SCLH();
    I2C_delay();
    SDAH();
    I2C_delay();
}

///////////////////////////////////////

void I2C_Ack(void)
{
    SCLL();
    I2C_delay();
    SDAL();
    I2C_delay();
    SCLH();
    I2C_delay();
    SCLL();
    I2C_delay();
}

///////////////////////////////////////

void I2C_NoAck(void)
{
    SCLL();
    I2C_delay();
    SDAH();
    I2C_delay();
    SCLH();
    I2C_delay();
    SCLL();
    I2C_delay();
}

///////////////////////////////////////

void I2C_SendByte(unsigned char SendByte)
{
    //    int8_t i = 8;
    unsigned char i = 8;

    while (i--)
        //  for(; i > 0; i--)
    {
        SCLL();
        I2C_delay();

        if (SendByte & 0x80)
        {
            SDAH();
        }

        if (!(SendByte & 0x80))
        {
            SDAL();
        }

        SendByte <<= 1;
        I2C_delay();
        SCLH();
        I2C_delay();
    }

    SCLL();
}

///////////////////////////////////////

uint8_t I2C_ReceiveByte(void)
{
    unsigned char i = 8;
    unsigned char ReceiveByte = 0;
    uint8_t t;
    uint8_t data;

    SDAH();

    while (i--)
    {
        ReceiveByte <<= 1;
        SCLL();
        I2C_delay();
        SCLH();
        data = 0;

        for (t = 0; t < 8; t++)
        {
            data += GPIO_ReadInputDataBit(I2C_SDA_PORT, I2C_SDA_PIN);
        }

        if (data >= 4)
        {
            ReceiveByte |= 0x01;
        }

    }

    SCLL();
    return ReceiveByte;
}

///////////////////////////////////////

void I2C_WaitAck(void)
{
    SCLL();
    I2C_delay();
    SDAH();
    I2C_delay();
    SCLH();
    I2C_delay();

    if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11) == 1)
    {
        I2Cerror = 1;
        I2Cerrorcount++;
    }

    SCLL();
}

///////////////////////////////////////////////////////////////////////////////

void i2cInit(I2C_TypeDef * I2Cx)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Pin   = I2C_SCL_PIN | I2C_SDA_PIN;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    GPIO_Init(GPIOB, &GPIO_InitStructure);
}

///////////////////////////////////////////////////////////////////////////////

bool i2cWrite(uint8_t addr_, uint8_t reg, uint8_t data)
{
	addr_ = addr_ * 2 + 1;

    I2C_Start();
    I2C_SendByte((addr_ & 0xFE));  // FE-0(Write)
    I2C_WaitAck();
    I2C_SendByte(reg);
    I2C_WaitAck();
    I2C_SendByte(data);
    I2C_WaitAck();
    I2C_Stop();

    return 1;
}

///////////////////////////////////////////////////////////////////////////////

bool i2cRead(uint8_t addr_, uint8_t reg, uint8_t len, uint8_t * buf)
{
    uint8_t i;

    addr_ = addr_ * 2 + 1;

    I2Cerror = 0;

    I2C_Start();
    I2C_SendByte((addr_ & 0xFE));  // FE-0(Write)
    I2C_WaitAck();

    if (I2Cerror == 0)
    {
        I2C_SendByte(reg);
        I2C_WaitAck();

        if (I2Cerror == 0)
        {
            I2C_Stop();
            I2C_Start();
            I2C_SendByte((addr_ & 0xFF));  // FF-1(Read)
            I2C_WaitAck();

            if (I2Cerror == 0)
            {
				for (i = 0; i < len - 1; i++)
				{
                    buf[i] = I2C_ReceiveByte(); //receive
                    I2C_Ack();
				}

			    buf[len - 1] = I2C_ReceiveByte(); //receive
                I2C_NoAck();
                I2C_Stop();
            }
        }
    }

    return 1;
}

///////////////////////////////////////////////////////////////////////////////

uint16_t i2cGetErrorCounter(void)
{
    return I2Cerrorcount;
}

///////////////////////////////////////////////////////////////////////////////

