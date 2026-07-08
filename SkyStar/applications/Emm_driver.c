#include "Emm_driver.h"

/* Private function prototypes -----------------------------------------------*/
static int8_t Emm_ValidateParams(Emm_Motor_t *emm);
static int8_t Emm_SendCommand(Emm_Motor_t *emm, const uint8_t *cmd_buffer, uint8_t length);
static uint8_t Emm_ParseResponseInternal(uint8_t *buffer, uint8_t len, Emm_V5_Response_t *resp);

/* Exported functions --------------------------------------------------------*/

int8_t Emm_Create(Emm_Motor_t *emm, uart_port_id_t port, uint8_t addr, uint32_t timeout_ms)
{
    if (emm == NULL || addr == 0 || port >= UART_PORT_MAX)
    {
        return -1;
    }

    emm->hw.port = port;
    emm->hw.timeout_ms = (timeout_ms == 0) ? EMM_UART_TIMEOUT : timeout_ms;

    emm->address = addr;
    emm->enable = 1;

    memset(&emm->data, 0, sizeof(Emm_Data_t));
    emm->data.addr = addr;

    emm->state = EMM_STATE_IDLE;
    emm->rx_index = 0;
    memset(emm->rx_buffer, 0, sizeof(emm->rx_buffer));

    return 0;
}

int8_t Emm_ProcessBuffer(Emm_Motor_t *emm, uint8_t *buffer, uint16_t length)
{
    if (Emm_ValidateParams(emm) != 0 || buffer == NULL || length == 0)
    {
        return -1;
    }

    if (buffer[0] != emm->address)
    {
        return -1;
    }

    if (length > 0 && buffer[length - 1] != EMM_FIXED_END_BYTE)
    {
        emm->state = EMM_STATE_ERROR;
        return -1;
    }

    Emm_V5_Response_t temp_resp;
    if (Emm_ParseResponseInternal(buffer, length, &temp_resp) == 0)
    {
        emm->state = EMM_STATE_ERROR;
        return -1;
    }

    emm->data.addr = temp_resp.addr;
    emm->data.timestamp = rt_tick_get();
    emm->data.data_valid = 1;

    switch (temp_resp.func)
    {
    case 0x1F:
        strncpy(emm->data.version, temp_resp.version, sizeof(emm->data.version) - 1);
        emm->data.version[sizeof(emm->data.version) - 1] = '\0';
        break;
    case 0x20:
        emm->data.current = temp_resp.current;
        emm->data.voltage = temp_resp.voltage;
        break;
    case 0x21:
        break;
    case 0x24:
        emm->data.voltage = temp_resp.voltage;
        break;
    case 0x27:
        emm->data.current = temp_resp.current;
        break;
    case 0x31:
        emm->data.encoder = temp_resp.encoder;
        break;
    case 0x33:
        emm->data.status = temp_resp.status;
        break;
    case 0x35:
        emm->data.dir = temp_resp.dir;
        emm->data.speed = temp_resp.speed;
        break;
    case 0x36:
        emm->data.dir = temp_resp.dir;
        emm->data.position = temp_resp.position;
        break;
    case 0x37:
        emm->data.position = temp_resp.position;
        break;
    case 0x39:
        emm->data.error = temp_resp.error;
        break;
    case 0x3A:
        emm->data.status = temp_resp.status;
        break;
    case 0x3B:
        emm->data.origin_state = temp_resp.origin_state;
        break;
    case 0x3D:
        emm->data.target_pos = temp_resp.target_pos;
        break;
    case 0x3E:
        emm->data.target_speed = temp_resp.target_speed;
        break;
    case 0x3F:
        emm->data.acceleration = temp_resp.acceleration;
        break;
    case 0x40:
        emm->data.subdivision = temp_resp.subdivision;
        break;
    case 0x41:
        emm->data.ctrl_mode = temp_resp.ctrl_mode;
        break;
    case 0x42:
        emm->data.protection = temp_resp.protection;
        break;
    case 0x43:
        emm->data.pwm_duty = temp_resp.pwm_duty;
        break;
    case 0x44:
        emm->data.closed_loop_state = temp_resp.closed_loop_state;
        break;
    case 0x45:
        emm->data.encoder_state = temp_resp.encoder_state;
        break;
    case 0x46:
        emm->data.sync_state = temp_resp.sync_state;
        break;
    case 0x47:
        emm->data.origin_state = temp_resp.origin_state;
        break;
    default:
        break;
    }

    emm->state = EMM_STATE_DATA_READY;
    return 0;
}

int8_t Emm_ResetCurrentPositionToZero(Emm_Motor_t *emm)
{
    if (Emm_ValidateParams(emm) != 0)
        return -1;
    uint8_t cmd[4] = {0};
    cmd[0] = emm->address;
    cmd[1] = 0x0A;
    cmd[2] = 0x6D;
    cmd[3] = EMM_FIXED_END_BYTE;
    return Emm_SendCommand(emm, cmd, sizeof(cmd));
}

int8_t Emm_ResetClogProtection(Emm_Motor_t *emm)
{
    if (Emm_ValidateParams(emm) != 0)
        return -1;
    uint8_t cmd[4] = {0};
    cmd[0] = emm->address;
    cmd[1] = 0x0E;
    cmd[2] = 0x52;
    cmd[3] = EMM_FIXED_END_BYTE;
    return Emm_SendCommand(emm, cmd, sizeof(cmd));
}

int8_t Emm_ReadSysParams(Emm_Motor_t *emm, SysParams_t s)
{
    if (Emm_ValidateParams(emm) != 0)
        return -1;
    uint8_t i = 0;
    uint8_t cmd[16] = {0};
    cmd[i] = emm->address;
    ++i;
    switch (s)
    {
    case S_VER:  cmd[i] = 0x1F; ++i; break;
    case S_RL:   cmd[i] = 0x20; ++i; break;
    case S_PID:  cmd[i] = 0x21; ++i; break;
    case S_VBUS: cmd[i] = 0x24; ++i; break;
    case S_CPHA: cmd[i] = 0x27; ++i; break;
    case S_ENCL: cmd[i] = 0x31; ++i; break;
    case S_TPOS: cmd[i] = 0x33; ++i; break;
    case S_VEL:  cmd[i] = 0x35; ++i; break;
    case S_CPOS: cmd[i] = 0x36; ++i; break;
    case S_PERR: cmd[i] = 0x37; ++i; break;
    case S_FLAG: cmd[i] = 0x3A; ++i; break;
    case S_ORG:  cmd[i] = 0x3B; ++i; break;
    case S_Conf: cmd[i] = 0x42; ++i; cmd[i] = 0x6C; ++i; break;
    case S_State: cmd[i] = 0x43; ++i; cmd[i] = 0x7A; ++i; break;
    default: return -1;
    }
    cmd[i] = EMM_FIXED_END_BYTE;
    ++i;

    emm->last_read_param_req = s;
    return Emm_SendCommand(emm, cmd, i);
}

int8_t Emm_ModifyControlMode(Emm_Motor_t *emm, bool svF, uint8_t ctrl_mode)
{
    if (Emm_ValidateParams(emm) != 0)
        return -1;
    uint8_t cmd[6] = {0};
    cmd[0] = emm->address;
    cmd[1] = 0x46;
    cmd[2] = 0x69;
    cmd[3] = svF;
    cmd[4] = ctrl_mode;
    cmd[5] = EMM_FIXED_END_BYTE;
    return Emm_SendCommand(emm, cmd, sizeof(cmd));
}

int8_t Emm_SetEnableControl(Emm_Motor_t *emm, bool state, bool snF)
{
    if (Emm_ValidateParams(emm) != 0)
        return -1;
    uint8_t cmd[6] = {0};
    cmd[0] = emm->address;
    cmd[1] = 0xF3;
    cmd[2] = 0xAB;
    cmd[3] = (uint8_t)state;
    cmd[4] = snF;
    cmd[5] = EMM_FIXED_END_BYTE;
    return Emm_SendCommand(emm, cmd, sizeof(cmd));
}

int8_t Emm_VelocityControl(Emm_Motor_t *emm, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
    if (Emm_ValidateParams(emm) != 0)
        return -1;
    uint8_t cmd[8] = {0};
    cmd[0] = emm->address;
    cmd[1] = 0xF6;
    cmd[2] = dir;
    cmd[3] = (uint8_t)(vel >> 8);
    cmd[4] = (uint8_t)(vel >> 0);
    cmd[5] = acc;
    cmd[6] = snF;
    cmd[7] = EMM_FIXED_END_BYTE;
    return Emm_SendCommand(emm, cmd, sizeof(cmd));
}

int8_t Emm_PositionControl(Emm_Motor_t *emm, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
    if (Emm_ValidateParams(emm) != 0)
        return -1;
    uint8_t cmd[13] = {0};
    cmd[0] = emm->address;
    cmd[1] = 0xFD;
    cmd[2] = dir;
    cmd[3] = (uint8_t)(vel >> 8);
    cmd[4] = (uint8_t)(vel >> 0);
    cmd[5] = acc;
    cmd[6] = (uint8_t)(clk >> 24);
    cmd[7] = (uint8_t)(clk >> 16);
    cmd[8] = (uint8_t)(clk >> 8);
    cmd[9] = (uint8_t)(clk >> 0);
    cmd[10] = raF;
    cmd[11] = snF;
    cmd[12] = EMM_FIXED_END_BYTE;
    return Emm_SendCommand(emm, cmd, sizeof(cmd));
}

int8_t Emm_StopNow(Emm_Motor_t *emm, bool snF)
{
    if (Emm_ValidateParams(emm) != 0)
        return -1;
    uint8_t cmd[5] = {0};
    cmd[0] = emm->address;
    cmd[1] = 0xFE;
    cmd[2] = 0x98;
    cmd[3] = snF;
    cmd[4] = EMM_FIXED_END_BYTE;
    return Emm_SendCommand(emm, cmd, sizeof(cmd));
}

int8_t Emm_SynchronousMotion(Emm_Motor_t *emm)
{
    if (Emm_ValidateParams(emm) != 0)
        return -1;
    uint8_t cmd[4] = {0};
    cmd[0] = emm->address;
    cmd[1] = 0xFF;
    cmd[2] = 0x66;
    cmd[3] = EMM_FIXED_END_BYTE;
    return Emm_SendCommand(emm, cmd, sizeof(cmd));
}

int8_t Emm_SetOrigin(Emm_Motor_t *emm, bool svF)
{
    if (Emm_ValidateParams(emm) != 0)
        return -1;
    uint8_t cmd[5] = {0};
    cmd[0] = emm->address;
    cmd[1] = 0x93;
    cmd[2] = 0x88;
    cmd[3] = svF;
    cmd[4] = EMM_FIXED_END_BYTE;
    return Emm_SendCommand(emm, cmd, sizeof(cmd));
}

int8_t Emm_ModifyOriginParams(Emm_Motor_t *emm, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
    if (Emm_ValidateParams(emm) != 0)
        return -1;
    uint8_t cmd[20] = {0};
    cmd[0] = emm->address;
    cmd[1] = 0x4C;
    cmd[2] = 0xAE;
    cmd[3] = svF;
    cmd[4] = o_mode;
    cmd[5] = o_dir;
    cmd[6] = (uint8_t)(o_vel >> 8);
    cmd[7] = (uint8_t)(o_vel >> 0);
    cmd[8] = (uint8_t)(o_tm >> 24);
    cmd[9] = (uint8_t)(o_tm >> 16);
    cmd[10] = (uint8_t)(o_tm >> 8);
    cmd[11] = (uint8_t)(o_tm >> 0);
    cmd[12] = (uint8_t)(sl_vel >> 8);
    cmd[13] = (uint8_t)(sl_vel >> 0);
    cmd[14] = (uint8_t)(sl_ma >> 8);
    cmd[15] = (uint8_t)(sl_ma >> 0);
    cmd[16] = (uint8_t)(sl_ms >> 8);
    cmd[17] = (uint8_t)(sl_ms >> 0);
    cmd[18] = potF;
    cmd[19] = EMM_FIXED_END_BYTE;
    return Emm_SendCommand(emm, cmd, sizeof(cmd));
}

int8_t Emm_TriggerOriginReturn(Emm_Motor_t *emm, uint8_t o_mode, bool snF)
{
    if (Emm_ValidateParams(emm) != 0)
        return -1;
    uint8_t cmd[5] = {0};
    cmd[0] = emm->address;
    cmd[1] = 0x9A;
    cmd[2] = o_mode;
    cmd[3] = snF;
    cmd[4] = EMM_FIXED_END_BYTE;
    return Emm_SendCommand(emm, cmd, sizeof(cmd));
}

int8_t Emm_InterruptOrigin(Emm_Motor_t *emm)
{
    if (Emm_ValidateParams(emm) != 0)
        return -1;
    uint8_t cmd[4] = {0};
    cmd[0] = emm->address;
    cmd[1] = 0x9C;
    cmd[2] = 0x48;
    cmd[3] = EMM_FIXED_END_BYTE;
    return Emm_SendCommand(emm, cmd, sizeof(cmd));
}

/* Getter functions ----------------------------------------------------------*/

int8_t Emm_GetVersion(Emm_Motor_t *emm, char *version_buf, uint8_t buf_len)
{
    if (Emm_ValidateParams(emm) != 0 || version_buf == NULL || buf_len == 0 || !emm->data.data_valid)
        return -1;
    strncpy(version_buf, emm->data.version, buf_len - 1);
    version_buf[buf_len - 1] = '\0';
    return 0;
}

int16_t Emm_GetSpeed(Emm_Motor_t *emm)
{
    if (Emm_ValidateParams(emm) != 0 || !emm->data.data_valid)
        return 0;
    return emm->data.speed;
}

int32_t Emm_GetPosition(Emm_Motor_t *emm)
{
    if (Emm_ValidateParams(emm) != 0 || !emm->data.data_valid)
        return 0;
    return emm->data.position;
}

bool Emm_IsEnabled(Emm_Motor_t *emm)
{
    if (Emm_ValidateParams(emm) != 0)
        return false;
    return (bool)emm->enable;
}

Emm_Data_t *Emm_GetData(Emm_Motor_t *emm)
{
    if (Emm_ValidateParams(emm) != 0 || !emm->data.data_valid)
        return NULL;
    return &(emm->data);
}

/* Private functions ---------------------------------------------------------*/

static int8_t Emm_ValidateParams(Emm_Motor_t *emm)
{
    if (emm == NULL || emm->hw.port >= UART_PORT_MAX || emm->address == 0)
    {
        return -1;
    }
    return 0;
}

static int8_t Emm_SendCommand(Emm_Motor_t *emm, const uint8_t *cmd_buffer, uint8_t length)
{
    if (Emm_ValidateParams(emm) != 0 || cmd_buffer == NULL || length == 0)
    {
        return -1;
    }

    rt_size_t sent = uart_send(emm->hw.port, cmd_buffer, length);

    rt_thread_mdelay(50);

    return (sent == length) ? 0 : -1;
}

static uint8_t Emm_ParseResponseInternal(uint8_t *buffer, uint8_t len, Emm_V5_Response_t *resp)
{
    uint8_t i;

    if (buffer == NULL || resp == NULL || len < 3)
        return 0;

    memset(resp, 0, sizeof(Emm_V5_Response_t));

    resp->addr = buffer[0];
    resp->func = buffer[1];
    resp->data_len = len;
    for (i = 0; i < len && i < sizeof(resp->raw_data); i++)
        resp->raw_data[i] = buffer[i];

    switch (resp->func)
    {
    case 0x1F:
        if (len >= 4)
        {
            for (i = 0; i < len - 3 && i < (sizeof(resp->version) - 1); i++)
                resp->version[i] = buffer[2 + i];
            resp->version[i] = '\0';
            resp->valid = 1;
        }
        break;
    case 0x20:
        if (len >= 6)
        {
            resp->current = (buffer[2] << 8) | buffer[3];
            resp->voltage = (buffer[4] << 8) | buffer[5];
            resp->valid = 1;
        }
        break;
    case 0x21:
        if (len >= 8)
        {
            resp->valid = 1;
        }
        break;
    case 0x24:
        if (len >= 4)
        {
            resp->voltage = (buffer[2] << 8) | buffer[3];
            resp->valid = 1;
        }
        break;
    case 0x27:
        if (len >= 4)
        {
            resp->current = (buffer[2] << 8) | buffer[3];
            resp->valid = 1;
        }
        break;
    case 0x31:
        if (len >= 6)
        {
            resp->encoder = (buffer[2] << 24) | (buffer[3] << 16) | (buffer[4] << 8) | buffer[5];
            resp->valid = 1;
        }
        break;
    case 0x33:
        if (len >= 3)
        {
            resp->status = buffer[2];
            resp->valid = 1;
        }
        break;
    case 0x35:
        if (len >= 5)
        {
            resp->dir = buffer[2];
            resp->speed = (buffer[3] << 8) | buffer[4];
            if (resp->dir)
                resp->speed = -resp->speed;
            resp->valid = 1;
        }
        break;
    case 0x36:
        if (len >= 7)
        {
            resp->dir = buffer[2];
            uint32_t full_position = (buffer[3] << 24) | (buffer[4] << 16) | (buffer[5] << 8) | buffer[6];
            resp->position = full_position % 65536;
            if (resp->dir)
                resp->position = -resp->position;
            resp->valid = 1;
        }
        break;
    case 0x37:
        if (len >= 7)
        {
            int32_t perr = (buffer[2] << 24) | (buffer[3] << 16) | (buffer[4] << 8) | buffer[5];
            resp->position = perr;
            resp->valid = 1;
        }
        break;
    case 0x39:
        if (len >= 3)
        {
            resp->error = buffer[2];
            resp->valid = 1;
        }
        break;
    case 0x3A:
        if (len >= 3)
        {
            resp->status = buffer[2];
            resp->valid = 1;
        }
        break;
    case 0x3B:
        if (len >= 3)
        {
            resp->origin_state = buffer[2];
            resp->valid = 1;
        }
        break;
    case 0x3D:
        if (len >= 6)
        {
            resp->target_pos = (buffer[2] << 24) | (buffer[3] << 16) | (buffer[4] << 8) | buffer[5];
            resp->valid = 1;
        }
        break;
    case 0x3E:
        if (len >= 4)
        {
            resp->target_speed = (buffer[2] << 8) | buffer[3];
            resp->valid = 1;
        }
        break;
    case 0x3F:
        if (len >= 3)
        {
            resp->acceleration = buffer[2];
            resp->valid = 1;
        }
        break;
    case 0x40:
        if (len >= 3)
        {
            resp->subdivision = buffer[2];
            resp->valid = 1;
        }
        break;
    case 0x41:
        if (len >= 3)
        {
            resp->ctrl_mode = buffer[2];
            resp->valid = 1;
        }
        break;
    case 0x42:
        if (len >= 3)
        {
            resp->protection = buffer[2];
            resp->valid = 1;
        }
        break;
    case 0x43:
        if (len >= 4)
        {
            resp->pwm_duty = (buffer[2] << 8) | buffer[3];
            resp->valid = 1;
        }
        break;
    case 0x44:
        if (len >= 3)
        {
            resp->closed_loop_state = buffer[2];
            resp->valid = 1;
        }
        break;
    case 0x45:
        if (len >= 3)
        {
            resp->encoder_state = buffer[2];
            resp->valid = 1;
        }
        break;
    case 0x46:
        if (len >= 3)
        {
            resp->sync_state = buffer[2];
            resp->valid = 1;
        }
        break;
    case 0x47:
        if (len >= 3)
        {
            resp->origin_state = buffer[2];
            resp->valid = 1;
        }
        break;
    default:
        resp->valid = 1;
        break;
    }
    return resp->valid;
}
