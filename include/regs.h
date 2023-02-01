#define BIT_MASK(a, b) (((unsigned) -1 >> (31 - (b))) & ~((1U << (a)) - 1))
#define I2C_ADDR 0x52

#define REG_MAIN_CTRL 0x00
#define REG_PS_LED 0x01
#define REG_PS_PULSES 0x02
#define REG_PS_MEAS_RATE 0x03
#define REG_LS_MEAS_RATE 0x04
#define REG_LS_GAIN 0x05
#define REG_PART_ID 0x06
#define REG_MAIN_STATUS 0x07
#define REG_PS_DATA0 0x08
#define REG_PS_DATA1 0x09
#define REG_LS_DATA_IR0 0x0A
#define REG_LS_DATA_IR1 0x0B
#define REG_LS_DATA_IR2 0x0C
#define REG_LS_DATA_GREEN0 0x0D
#define REG_LS_DATA_GREEN1 0x0E
#define REG_LS_DATA_GREEN2 0x0F
#define REG_LS_DATA_BLUE0 0x10
#define REG_LS_DATA_BLUE1 0x11
#define REG_LS_DATA_BLUE2 0x12
#define REG_LS_DATA_RED0 0x13
#define REG_LS_DATA_RED1 0x14
#define REG_LS_DATA_RED2 0x15
#define REG_INT_CFG 0x19
#define REG_INT_PST 0x1A
#define REG_PS_THRES_UP0 0x1B
#define REG_PS_THRES_UP1 0x1C
#define REG_PS_THRES_LOW 0x1D
#define REG_PS_THRES_LOW 0x1E
#define REG_PS_CAN0 0x1F
#define REG_PS_CAN1 0x20
#define REG_LS_THRES_UP0 0x21
#define REG_LS_THRES_UP1 0x22
#define REG_LS_THRES_UP2 0x23
#define REG_LS_THRES_LOW0 0x24
#define REG_LS_THRES_LOW1 0x25
#define REG_LS_THRES_LOW2 0x26
#define REG_LS_THRES_VAR 0x27

#define BIT_MAIN_CTRL_PS_EN 1 << 0
#define BIT_MAIN_CTRL_LS_EN 1 << 1
#define BIT_MAIN_CTRL_RGB_MODE 1 << 2
#define BIT_MAIN_CTRL_SW_RESET 1 << 4
#define BIT_MAIN_CTRL_SAI_LS 1 << 5
#define BIT_MAIN_CTRL_SAI_PS 1 << 6
