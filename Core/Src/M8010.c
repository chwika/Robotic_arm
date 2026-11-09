//============================================================
//
//  go-m8010??
//  RS485 ??? DMA ??????
//  !!! ???? stm32h7xx_it.c
//  !!! ?? HAL_UART_TxCpltCallback ? USART3_IRQHandler(IDLE ??)
//
//============================================================


#include "m8010.h"
#include <string.h>


Gom485Bus_t m8010_bus_1 = {0};   // m8010????1??
Gom485Bus_t m8010_bus_2 = {0};   // m8010????2??
Gom485Bus_t m8010_bus_3 = {0};   // m8010????3??
float kp[3]={0.45f,0.1f,0.1f};
float kd[3]={0.02f,0.f,0.f};
int16_t Pos[3];

/**
 * @brief ??????
 */
#define SATURATE(_IN, _MIN, _MAX) \
	{                             \
		if ((_IN) <= (_MIN))      \
			(_IN) = (_MIN);       \
		else if ((_IN) >= (_MAX)) \
			(_IN) = (_MAX);       \
	}


/* ============================================== crc???? ============================================== */

/*
 * This mysterious table is just the CRC of each possible byte. It can be
 * computed using the standard bit-at-a-time methods. The polynomial can
 * be seen in entry 128, 0x8408. This corresponds to x^0 + x^5 + x^12.
 * Add the implicit x^16, and you have the standard CRC-CCITT.
 * https://github.com/torvalds/linux/blob/5bfc75d92efd494db37f5c4c173d3639d4772966/lib/crc-ccitt.c
 */
const uint16_t crc_ccitt_table[256] = {
	0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
	0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
	0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
	0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
	0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

uint16_t crc_ccitt_byte(uint16_t crc, const uint8_t c)
{
	return (crc >> 8) ^ crc_ccitt_table[(crc ^ c) & 0xff];
}

/**
 *	crc_ccitt - recompute the CRC (CRC-CCITT variant) for the data
 *	buffer
 *	@crc: previous CRC value
 *	@buffer: data pointer
 *	@len: number of bytes in the buffer
 */
uint16_t crc_ccitt(uint16_t crc, uint8_t const *buffer, size_t len)
{
	while (len--)
		crc = crc_ccitt_byte(crc, *buffer++);
	return crc;
}//??????????CRC

/* ============================================== crc???? ============================================== */


/* ============================================== ???485?????? ============================================== */
/**
 * @brief ????????????????????,?? GPIO ???????
 * @note ??????cubemx??485???????(????????),?????????????????,????
*/

void m8010_bus1_rx_mode(void) {HAL_GPIO_WritePin(GPIOD,  GPIO_PIN_10, GPIO_PIN_RESET);}
void m8010_bus1_tx_mode(void) {HAL_GPIO_WritePin(GPIOD,  GPIO_PIN_10, GPIO_PIN_SET);}
void m8010_bus2_rx_mode(void) {HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);}
void m8010_bus2_tx_mode(void) {HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET);}
void m8010_bus3_rx_mode(void) {HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_RESET);}
void m8010_bus3_tx_mode(void) {HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, GPIO_PIN_SET);}

/* ============================================== ???485?????? ============================================== */


/**
 * @brief ?m8010?????????????????,??? CRC16
 * @note ????????????????????,??????
 */
void m8010_modify_data(MotorCmd_t *motor_s)//数据限幅之后放入发送数组
{
	motor_s->motor_send_data.head[0] = 0xFE;
	motor_s->motor_send_data.head[1] = 0xEE;

	SATURATE(motor_s->id, 0, 15);
	SATURATE(motor_s->mode, 0, 7);
	SATURATE(motor_s->K_P, 0.0f, 25.599f);
	SATURATE(motor_s->K_W, 0.0f, 25.599f);
	SATURATE(motor_s->T, -127.99f, 127.99f);
	SATURATE(motor_s->W, -804.00f, 804.00f);
	SATURATE(motor_s->Pos, -411774.0f, 411774.0f);

	motor_s->motor_send_data.mode.id = motor_s->id;
	motor_s->motor_send_data.mode.status = motor_s->mode;
	motor_s->motor_send_data.comd.k_pos = motor_s->K_P / 25.6f * 32768.0f;
	motor_s->motor_send_data.comd.k_spd = motor_s->K_W / 25.6f * 32768.0f;
	motor_s->motor_send_data.comd.pos_des = motor_s->Pos / 6.28318f * 32768.0f * REDUCTION_RATIO;
	motor_s->motor_send_data.comd.spd_des = motor_s->W / 6.28318f * 256.0f * REDUCTION_RATIO;
	motor_s->motor_send_data.comd.tor_des = motor_s->T * 256.0f;
	motor_s->motor_send_data.CRC16 = crc_ccitt(0, (uint8_t *)&motor_s->motor_send_data, sizeof(RIS_ControlData_t) - sizeof(motor_s->motor_send_data.CRC16));
}

/**
 * @brief ?m8010???????????????????,???? CRC16
 * @brief ????????????????????,??????
*/
void m8010_extract_data(MotorData_t *motor_r)
{
	if (motor_r->motor_recv_data.head[0] != 0xFD || motor_r->motor_recv_data.head[1] != 0xEE)
	{
		motor_r->correct = 0;
		return;
	}
	motor_r->calc_crc = crc_ccitt(0, (uint8_t *)&motor_r->motor_recv_data, sizeof(RIS_MotorData_t) - sizeof(motor_r->motor_recv_data.CRC16));
	if (motor_r->motor_recv_data.CRC16 != motor_r->calc_crc)
	{
		memset(&motor_r->motor_recv_data, 0, sizeof(RIS_MotorData_t));
		motor_r->correct = 0;
		motor_r->bad_msg++;
		return;
	}
	else
	{
		motor_r->motor_id = motor_r->motor_recv_data.mode.id;
		motor_r->mode = motor_r->motor_recv_data.mode.status;
		motor_r->Temp = motor_r->motor_recv_data.fbk.temp;
		motor_r->MError = motor_r->motor_recv_data.fbk.MError;
		motor_r->W = ((float)motor_r->motor_recv_data.fbk.speed / 256.0f) * 6.28318f / REDUCTION_RATIO;
		motor_r->T = ((float)motor_r->motor_recv_data.fbk.torque) / 256.0f;
		motor_r->Pos = 6.28318f * ((float)motor_r->motor_recv_data.fbk.pos) / 32768.0f / REDUCTION_RATIO;
		motor_r->footForce = motor_r->motor_recv_data.fbk.force;
		motor_r->correct = 1;
		return;
	}
}//???????????

/* ======================================================================== */
/*  GOM-485 ??? DMA ?????? � ????                              */
/* ======================================================================== */

#include "main.h"

#define GOM_RX_TIMEOUT_TICK  5

/* === ??????? ====================================================== */
static Gom485Bus_t *g_bus_table[GOM_MAX_BUSES] = {NULL};
static uint8_t      g_bus_count = 0;

Gom485Bus_t *m8010_find_by_huart(UART_HandleTypeDef *huart)
{
    for (uint8_t i = 0; i < g_bus_count; i++)
        if (g_bus_table[i] && g_bus_table[i]->p_huart == huart)
            return g_bus_table[i];
    return NULL;
}

Gom485Bus_t *m8010_find_by_htim(TIM_HandleTypeDef *htim)
{
    for (uint8_t i = 0; i < g_bus_count; i++)
        if (g_bus_table[i] && g_bus_table[i]->p_htim == htim)
            return g_bus_table[i];
    return NULL;
}


void m8010_485_idle_callback(UART_HandleTypeDef *huart)
{
    Gom485Bus_t *b = m8010_find_by_huart(huart);
    if (!b) return;

    uint32_t ndtr = __HAL_DMA_GET_COUNTER(b->p_huart->hdmarx);
    uint16_t size = GOM_DMA_RX_BUF_SIZE - ndtr;

    if (size > 0)
    {
        b->internal.rx_data_len = size;
        b->internal.rx_frame_ready = 1;
        b->internal.g_state = GOM_STATE_RX_READY;
    }
    else
    {
        HAL_UART_AbortReceive(b->p_huart);
        HAL_UART_Receive_DMA(b->p_huart, b->internal.rx_dma_buf, GOM_DMA_RX_BUF_SIZE);
    }
}

/**
 * @brief ????????????????,???m8010??????
*/
void m8010_485_timer_trigger(TIM_HandleTypeDef *htim)
{
    // Gom485Bus_t *b = m8010_find_by_htim(htim);
    // if (b) b->internal.tx_done_flag = 1;
    for (uint8_t i = 0; i < g_bus_count; i++)
    {
        if (g_bus_table[i] && g_bus_table[i]->p_htim == htim)
        {
            g_bus_table[i]->internal.tx_done_flag = 1;
        }
    }
}

/* ======================================================================== */
/*  ?? API                                                                */
/* ======================================================================== */

/**
 * @brief ??? m8010 ??????,?? UART ????,?????????
*/
void m8010_bus_init(Gom485Bus_t *bus, UART_HandleTypeDef *huart, TIM_HandleTypeDef *htim)
{
    bus->p_huart = huart;
    bus->p_htim  = htim;
    bus->current_idx = 0;

    __HAL_UART_ENABLE_IT(huart, UART_IT_IDLE);
    HAL_UART_Receive_DMA(huart, bus->internal.rx_dma_buf, GOM_DMA_RX_BUF_SIZE);

    bus->internal.rx_frame_ready = 0;
    bus->internal.rx_data_len    = 0;
    bus->internal.tx_done_flag   = 1;
    bus->internal.g_state        = GOM_STATE_IDLE;
    bus->internal.debug_cnt      = 0;
    bus->internal.last_tick      = 0;
    bus->internal.tx_start_tick  = 0;
    bus->internal.rx_timeout_cnt = 0;
    bus->internal.rx_noise_cnt   = 0;

    if (g_bus_count < GOM_MAX_BUSES)
        g_bus_table[g_bus_count++] = bus;
}

/**
 * @brief go-m8010??????,?????????
 * @note ??????????????,????????
*/
Gom485State_t m8010_bus_poll(Gom485Bus_t *bus)
{
    uint32_t now = HAL_GetTick();

    /* ?????? systick */
    if (bus->p_htim == NULL && (now - bus->internal.last_tick >= 1))
    {
        bus->internal.last_tick = now;
        bus->internal.tx_done_flag = 1;
    }

    /* -- ?????? -- */
    if (bus->internal.g_state == GOM_STATE_WAIT_RX && (now - bus->internal.tx_start_tick) >= GOM_RX_TIMEOUT_TICK)
    {
        HAL_UART_AbortReceive(bus->p_huart);
        HAL_UART_Receive_DMA(bus->p_huart, bus->internal.rx_dma_buf, GOM_DMA_RX_BUF_SIZE);
        bus->internal.rx_frame_ready = 0;
        bus->internal.g_state = GOM_STATE_IDLE;
        bus->internal.rx_timeout_cnt++;
        bus->internal.tx_done_flag = 0;
        bus->internal.last_tick = now;

        /* ??:??????,????? */
        bus->current_idx = (bus->current_idx + 1) % GOM_MOTORS_PER_BUS;
    }

    /* -- ????? -- */
    if (bus->internal.rx_frame_ready)
    {
        bus->internal.rx_frame_ready = 0;

        GOM_Motor_t *motor = &bus->motors[bus->current_idx];
        if (motor && bus->internal.rx_data_len == GOM_RX_FRAME_SIZE)
        {
            memcpy(&motor->measure.motor_recv_data, bus->internal.rx_dma_buf, bus->internal.rx_data_len);
            m8010_extract_data(&motor->measure);

            if (motor->measure.Pos_init_flag < 200 && motor->measure.correct == 1)
            {
                motor->measure.init_Pos = motor->measure.Pos;
                motor->measure.Pos_init_flag += 1;
            }
            motor->measure.Pos = motor->measure.Pos - motor->measure.init_Pos;
            //去除初始位置的注释cancel annotation
            
                    if (motor->measure.Pos_init_flag == 0 && motor->measure.correct == 1)
                    motor->measure.init_Pos = motor->measure.Pos;
                    if (motor->measure.Pos_init_flag < 200) motor->measure.Pos_init_flag++;
        }
        else if (motor)
        {
            motor->measure.correct = 0;
            bus->internal.rx_noise_cnt++;
        }

        bus->internal.debug_cnt++;

        HAL_UART_AbortReceive(bus->p_huart);
        HAL_UART_Receive_DMA(bus->p_huart, bus->internal.rx_dma_buf, GOM_DMA_RX_BUF_SIZE);
        bus->internal.g_state = GOM_STATE_IDLE;

        /* ?????(??????):??????? */
        bus->current_idx = (bus->current_idx + 1) % GOM_MOTORS_PER_BUS;
    }

    /* -- ???? -- */
    if (bus->internal.tx_done_flag && bus->internal.g_state == GOM_STATE_IDLE)
    {
        bus->internal.tx_done_flag = 0;

        /* ?????(id==0 ?????) */
        uint8_t tries = GOM_MOTORS_PER_BUS;
        while (tries--)
        {
            GOM_Motor_t *motor = &bus->motors[bus->current_idx];
            if (motor->target.id != 0)   /* ???????? */
                break;
            bus->current_idx = (bus->current_idx + 1) % GOM_MOTORS_PER_BUS;
        }

        GOM_Motor_t *motor = &bus->motors[bus->current_idx];
        if (motor->target.id != 0)   /* ????????,??? */
        {
            m8010_modify_data(&motor->target);
            if(bus == &m8010_bus_1){m8010_bus1_tx_mode();}
            if(bus == &m8010_bus_2){m8010_bus2_tx_mode();}
            if(bus == &m8010_bus_3){m8010_bus3_tx_mode();}

            bus->internal.g_state = GOM_STATE_TX_BUSY;
            bus->internal.tx_start_tick = now;
            HAL_UART_Transmit_DMA(bus->p_huart, (uint8_t *)&motor->target.motor_send_data,
                                  sizeof(motor->target.motor_send_data));
        }
    }

    return bus->internal.g_state;
}

/* ========================================== ?????? ========================================== */

/**
 * @brief ??? M8010 ?????
 * @note ??????? id, mode, kp, kw
 * @param id ?? ID,?? 0-15
 * @param mode ??????,0???????,1???????,2?????(??!!!)
 * @param K_P ??????
 * @param K_W ??????
*/
void m8010_parameter_init(MotorCmd_t *motor, uint16_t id, uint16_t mode, float K_P, float K_W)
{
    motor->id = id;
    motor->mode = mode;
    motor->K_P = K_P;
    motor->K_W = K_W;
}

/**
 * @brief M8010 ???????
 * @param T ????????,?? Nm
*/
void m8010_torque_control(GOM_Motor_t *motor, float T)
{
    motor->target.mode = 1;
    motor->target.T = T;

    motor->target.W = 0.0f;
    motor->target.K_W = 0.0f;
    motor->target.Pos = 0.0f;
    motor->target.K_P = 0.0f;
}

/**
 * @brief M8010 ???????
 * @param W ???????,?? rad/s
 * @param K_W ??????
*/
void m8010_velocity_control(GOM_Motor_t *motor, float W, float K_W)
{
    motor->target.mode = 1;
    motor->target.W = W;
    motor->target.K_W = K_W;

    motor->target.T = 0.0f;
    motor->target.Pos = 0.0f;
    motor->target.K_P = 0.0f;
}

/**
 * @brief M8010 ???????,????????
 * @param Pos ???????,?? rad
 * @param K_P ?????kp
 * @param K_W ?????kd
 * @warning target_position?:???????????????????,???????????,???????????????????
*/
void m8010_position_control(GOM_Motor_t *motor, float Pos, float K_P, float K_W)
{
    motor->target.mode = 1;
    motor->target.Pos = Pos + motor->measure.init_Pos;
    motor->target.K_W = K_W;
    motor->target.K_P = K_P;

    motor->target.T = 0.0f;
    motor->target.W = 0.0f;
}
