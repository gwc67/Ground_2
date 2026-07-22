#include "ano_ture.h"
#include "uarts.h"
#include "ano.h"
#include "main.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include "SysConfig.h"
/* container_of：从基类指针 stAnoBase* 还原出子类指针 stAnoDevice*。
 * 前提：stAnoDevice 第一个成员必须是 stBase（见 ano_ture.h）。
 * 理解成 C++ 的 this 还原即可：vtable 函数收到的是基类指针，
 * 要访问子类私有字段(如 pucRxBuffer)就得靠这个宏转回去。 */
#define container_of(ptr, type, member) ((type*)((char*)(ptr) - offsetof(type, member)))


/* ============================================================================
 * ANO 协议帧格式（背下来，下面的状态机/拼帧都围绕它）：
 *
 *   位置   内容
 *   [0]    0xAA          帧头1（固定）
 *   [1]    0xFF          帧头2 / 目的地址（0xFF=广播给所有人）
 *   [2]    CMD           命令字（0xE0命令 / 0x00确认 / 0x16航点 / 0xE2参数…）
 *   [3]    LEN           载荷字节数（从 [4] 开始算这么多字节）
 *   [4..]  载荷数据      LEN 个字节
 *   后2    SUM1, SUM2    两级累加校验和
 *
 * 校验和：SUM1 = 所有字节之和 & 0xFF；
 *         SUM2 = 每加一字节就把当前 SUM1 累加进去 & 0xFF（见 vFrame_Send_Ano）。
 *         比"普通累加和"更能检出字节乱序。
 *
 * 本文件的两大职责：
 *   (a) 收：Data_receive_prepare_Ano_s（7级状态机逐字节收帧）
 *          Data_Check_Ano_s（while 循环从 UART RingBuffer 读字节喂状态机）
 *   (b) 发：vFrame_Send_Ano（拼帧+算校验+发）
 *          Check_to_send_Ano_s（按频率/标志决定发不发）
 *          CK_Back_Check_Ano_s（ACK 没回就重传，最多5次）
 * ========================================================================== */



/* ANO 协议接收状态机 —— 逐字节喂进来，收满一帧回调 Ano_Receive_Anl。
 *
 * 7 级状态（ucRxstate）：
 *   state0 收到 0xAA → state1
 *   state1 收到 0xFF → state2（否则复位回0，抗噪）
 *   state2 收到 CMD  → state3
 *   state3 收到 LEN  → state4（记下载荷要收几字节）
 *   state4 收载荷    → 收满 LEN 个 → state5
 *   state5 收到 SUM1  → state6
 *   state6 收到 SUM2  → 复位，整帧已在 pucRxBuffer，回调处理
 *
 * 任何一步对不上 → 复位回 state0（丢掉半截帧，从下一帧头重新对齐）。
 * 这样能扛住线路上的随机噪声。
 * 返回 1 表示收到完整一帧（供上层统计），0 表示还没收满。 */
static int Data_receive_prepare_Ano_s(stAnoBase* stBase,uint8_t ucByte)
{
    stAnoDevice *me = container_of(stBase,stAnoDevice,stBase);

    if (me->ucRxstate == 0 && ucByte == 0xAA)
    {
        me->ucRxstate = 1;
        me->ucData_Cnt = 0;
        me->ucData_Len = 0;
        me->pucRxBuffer[me->ucData_Cnt++] = ucByte;
    }
    else if (me->ucRxstate == 1 && ucByte == 0xFF)
    {
        me->ucRxstate = 2;
        me->pucRxBuffer[me->ucData_Cnt++] = ucByte;
    }
    else if (me->ucRxstate == 2)
    {
        me->ucRxstate = 3;
        me->pucRxBuffer[me->ucData_Cnt++] = ucByte;
    }
    else if (me->ucRxstate == 3)
    {
        me->ucRxstate = 4;
        me->pucRxBuffer[me->ucData_Cnt++] = ucByte;
        me->ucData_Len = ucByte;
    }
    else if (me->ucRxstate == 4 && me->ucData_Len > 0)
    {
        me->ucData_Len--;
        me->pucRxBuffer[me->ucData_Cnt++] = ucByte;
        if (me->ucData_Len == 0)
        {
            me->ucRxstate = 5;
        }
    }
    else if (me->ucRxstate == 5)
    {
        me->ucRxstate = 6;
        me->pucRxBuffer[me->ucData_Cnt++] = ucByte;
    }
    else if (me->ucRxstate == 6)
    {
        me->ucRxstate = 0;
        me->pucRxBuffer[me->ucData_Cnt++] = ucByte;
        me->stUser.Ano_Receive_Anl(me->pucRxBuffer,me->ucData_Cnt);
        return 1;  
    }
    else
    {
        me->ucRxstate = 0;
    }
    return 0;
}

static void Pending_cmd_Dispatch_Ano_v(stAnoDevice *me)
{
    if (!me->pstDeviceDt->has_cmd_queue_b)
        return;
    me->pstDeviceDt->has_cmd_queue_b = 0;
    me->pstDeviceDt->cmd_send = me->pstDeviceDt->cmd_queue;
    me->pstDeviceDt->fun[0xe0].WTS = 1;
    me->pstDeviceDt->wait_ck = 1;
}

/* 发送命令帧(0xE0)。
 * 关键机制：同一时刻只能有一个命令帧在"等 ACK"(wait_ck=1)。
 *   - 如果当前没在等 ACK：直接填 cmd_send，置 WTS=1 触发发送，置 wait_ck=1。
 *   - 如果正在等 ACK：把这条命令暂存进 cmd_queue，等 ACK 回来后由
 *     Pending_cmd_Dispatch_Ano_v 自动续发。这样命令不会丢，也不会并发。
 * CMD[10] 固定 10 字节槽位，多余截断。 */
static int Cmd_send_Ano_s(stAnoBase *stBase, u8 dest_addr, u8 cid, const u8 *cmd_bytes, u8 cmd_len)
{
    stAnoDevice *me = container_of(stBase,stAnoDevice,stBase);

    if (me->pstDeviceDt->wait_ck)
    {
        me->pstDeviceDt->cmd_queue.CID = cid;
        memset(me->pstDeviceDt->cmd_queue.CMD, 0, 10);
        for (int i = 0; i < cmd_len && i < 10; i++)
            me->pstDeviceDt->cmd_queue.CMD[i] = cmd_bytes[i];
        me->pstDeviceDt->fun[0xe0].D_Addr = dest_addr;
        me->pstDeviceDt->has_cmd_queue_b = 1;
        return 0;
    }

    me->pstDeviceDt->cmd_send.CID = cid;
    memset(me->pstDeviceDt->cmd_send.CMD, 0, 10);
    for (int i = 0; i < cmd_len && i < 10; i++)
    {
        me->pstDeviceDt->cmd_send.CMD[i] = cmd_bytes[i];
    }

    me->pstDeviceDt->fun[0xe0].D_Addr = dest_addr;
    me->pstDeviceDt->fun[0xe0].WTS = 1;
    me->pstDeviceDt->wait_ck       = 1;
    return 0;
}

static int Ck_back_Ano_s(stAnoBase *stBase, u8 dest_addr, u8 id, u8 sc, u8 ac)
{
    stAnoDevice *me = container_of(stBase,stAnoDevice,stBase);

    me->pstDeviceDt->ck_send.ID = id;
    me->pstDeviceDt->ck_send.SC = sc;
    me->pstDeviceDt->ck_send.AC = ac;

    me->pstDeviceDt->fun[0x00].D_Addr = dest_addr;
    me->pstDeviceDt->fun[0x00].WTS = 1;
    return 0;
}
static int Par_back_Ano_s(stAnoBase *stBase, u8 dest_addr, u16 par_id, s32 par_val)
{
    stAnoDevice *me = container_of(stBase,stAnoDevice,stBase);
    me->pstDeviceDt->par_data.par_id = par_id;
    me->pstDeviceDt->par_data.par_val = par_val;
    me->pstDeviceDt->fun[0xe2].D_Addr = dest_addr;
    me->pstDeviceDt->fun[0xe2].WTS = 1;
    return 0;
}


static uint8_t Ck_id_get_Ano(stAnoBase *stBase)
{
    stAnoDevice *me = container_of(stBase, stAnoDevice, stBase);
    return me->pstDeviceDt->ck_send.ID;
}

static uint8_t Ck_sc_get_Ano(stAnoBase *stBase)
{
    stAnoDevice *me = container_of(stBase, stAnoDevice, stBase);
    return me->pstDeviceDt->ck_send.SC;
}

static uint8_t Ck_ac_get_Ano(stAnoBase *stBase)
{
    stAnoDevice *me = container_of(stBase, stAnoDevice, stBase);
    return me->pstDeviceDt->ck_send.AC;
}

static uint8_t Cmd_cid_get_Ano(stAnoBase *stBase)
{
    stAnoDevice *me = container_of(stBase, stAnoDevice, stBase);
    return me->pstDeviceDt->cmd_send.CID;
}

static int Cmd_copy_bytes_Ano(stAnoBase *stBase, uint8_t *buf, uint8_t max_len)
{
    stAnoDevice *me = container_of(stBase, stAnoDevice, stBase);
    uint8_t copy_cnt = (max_len < 10) ? max_len : 10;
    memcpy(buf, me->pstDeviceDt->cmd_send.CMD, copy_cnt);
    return (int)copy_cnt;
}

static uint16_t Par_id_get_Ano(stAnoBase *stBase)
{
    stAnoDevice *me = container_of(stBase, stAnoDevice, stBase);
    return me->pstDeviceDt->par_data.par_id;
}

static int32_t Par_val_get_Ano(stAnoBase *stBase)
{
    stAnoDevice *me = container_of(stBase, stAnoDevice, stBase);
    return me->pstDeviceDt->par_data.par_val;
}

/* 拼装并发送一帧（ANO 帧格式的"反过程"）。
 * 流程：
 *   1. 填帧头 0xAA、目的地址、CMD、LEN(先填0占位)
 *   2. 调 unique.Ano_Add_Send_Data 让通路填载荷（各串口填法不同）
 *   3. 回填真正的 LEN = _cnt - 4
 *   4. 算两级累加校验和，追加 SUM1/SUM2
 *   5. 如果是命令帧(0xE0)且在等 ACK：记下本校验和到 ck_send，
 *      这样等对方回 0x00 确认时能比对是不是"我发的那帧"的回执
 *   6. 调 unique.Send_Buff 物理发出（→ uart_transmit → TX 队列非阻塞） */
static void vFrame_Send_Ano(stAnoBase *stBase, uint8_t ucFrame_num)
{
    stAnoDevice *me = container_of(stBase, stAnoDevice, stBase);

    uint8_t _cnt = 0;

    me->pucTxBuffer[_cnt++] = 0xAA;
    me->pucTxBuffer[_cnt++] = me->pstDeviceDt->fun[ucFrame_num].D_Addr;
    me->pucTxBuffer[_cnt++] = ucFrame_num;
    me->pucTxBuffer[_cnt++] = 0;

    me->stUser.Ano_Add_Send_Data(ucFrame_num,&_cnt,me->pucTxBuffer);

    me->pucTxBuffer[3] = _cnt - 4;

    u8 check_sum1 = 0, check_sum2 = 0;

    for (u8 i = 0; i < _cnt; i++)
    {
        check_sum1 += me->pucTxBuffer[i];
        check_sum2 += check_sum1;
    }
    me->pucTxBuffer[_cnt++] = check_sum1;
    me->pucTxBuffer[_cnt++] = check_sum2;

    if (me->pstDeviceDt->wait_ck != 0 && ucFrame_num == 0xe0)
    {
        //对于发送是发送校验位,ck_send 
        me->pstDeviceDt->ck_send.ID = ucFrame_num;
        me->pstDeviceDt->ck_send.SC = check_sum1;
        me->pstDeviceDt->ck_send.AC = check_sum2;       
    }

    me->stUser.Send_Buff(me->pucTxBuffer,_cnt);
    
    
}

/* 周期调度：决定某帧"现在要不要发"。
 * 两种触发：
 *   1) 定时发送：fun[帧号].fre_ms 不为 0 时，按毫秒计数到点 → 置 WTS=1。
 *      (用于周期上报数据，如电池电压每 20ms 一次)
 *   2) 事件触发：上层直接调 vano_WTS_set 置 WTS=1。
 *      (用于一次性命令，如发航点)
 * 只要 WTS=1 就调 vFrame_Send_Ano 真正发出去，发完清 WTS。
 * 注：fre_ms=0 表示"不自动定时，只靠外部触发"。 */
static int Check_to_send_Ano_s(stAnoBase* stBase,uint8_t ucFrame_num)
{
    stAnoDevice *me = container_of(stBase,stAnoDevice,stBase);
    if (me->pstDeviceDt->fun[ucFrame_num].fre_ms)
    {
        if(me->pstDeviceDt->fun[ucFrame_num].time_cnt_ms < me->pstDeviceDt->fun[ucFrame_num].fre_ms)
        {
            me->pstDeviceDt->fun[ucFrame_num].time_cnt_ms++;
        }
        else
        {
            me->pstDeviceDt->fun[ucFrame_num].time_cnt_ms = 1;
            me->pstDeviceDt->fun[ucFrame_num].WTS = 1;
        }
    }

    if (me->pstDeviceDt->fun[ucFrame_num].WTS)
    {
        me->pstDeviceDt->fun[ucFrame_num].WTS = 0;
        vFrame_Send_Ano(&me->stBase,ucFrame_num);
    }
    return 0;
}

static int Frame_num_set_Ano_s(stAnoBase* stBase,uint8_t ucFrame_num,uint16_t usFreq)
{
    stAnoDevice *me = container_of(stBase,stAnoDevice,stBase);
    me->pstDeviceDt->fun[ucFrame_num].D_Addr = 0xff;
    me->pstDeviceDt->fun[ucFrame_num].fre_ms = usFreq;
    me->pstDeviceDt->fun[ucFrame_num].time_cnt_ms = 0;
    return 0;
}

static int WTS_Set_Ano_s(stAnoBase* stBase,uint8_t ucFrame_num,uint8_t ucState)
{
    stAnoDevice *me = container_of(stBase,stAnoDevice,stBase);
    me->pstDeviceDt->fun[ucFrame_num].WTS = ucState;
    return 0;
}

static int Wait_ck_clear_Ano_s(stAnoBase* stBase)
{
    stAnoDevice *me = container_of(stBase,stAnoDevice,stBase);
    me->pstDeviceDt->wait_ck = 0;
    /* wait_ck 清零（ACK 已收到）→ 发送队列中的待命 CMD */
    Pending_cmd_Dispatch_Ano_v(me);
    return 0;
}

static int8_t Wait_ck_get_c(stAnoBase* stBase)
{
    stAnoDevice *me = container_of(stBase,stAnoDevice,stBase);
    return (int8_t)me->pstDeviceDt->wait_ck;
}

/* ACK 重传机制 —— 协议层的"可靠传输"，类似 TCP 的超时重传。
 * 前提：发命令帧(0xE0)时置了 wait_ck=1，等对方回 0x00 确认。
 * 逻辑：
 *   - wait_ck=1 时，每调用一次这里累加 ucTime_dly（由任务周期驱动）
 *   - 累满 50 个 tick 还没等到 ACK → 重发一次 0xE0，计数 ucRepeat_cnt++
 *   - 重发不超过 5 次：第 5 次还没 ACK → 放弃，清 wait_ck，
 *     并把暂存的 cmd_queue 续发出去（避免命令永远卡死）
 * 调用方：任务循环里周期调 vano_ck_back_check（见 ano_device_*.c 的
 *         v*_Data_Exchange_Task_Ano）。 */
static int CK_Back_Check_Ano_s(stAnoBase*stBase)
{
    stAnoDevice *me = container_of(stBase,stAnoDevice,stBase);
    if (me->pstDeviceDt->wait_ck == 1)
    {
        if (me->pstDeviceDt->ucTime_dly < 50)
        {
            me->pstDeviceDt->ucTime_dly++;
        }
        else
        {
            me->pstDeviceDt->ucTime_dly = 0;
            me->pstDeviceDt->ucRepeat_cnt++;
            if (me->pstDeviceDt->ucRepeat_cnt < 5)
            {
                me->pstDeviceDt->fun[0xe0].WTS = 1;
            }
            else
            {
                me->pstDeviceDt->ucRepeat_cnt = 0;
                me->pstDeviceDt->wait_ck = 0;
                /* 重传超时 → 发送队列中的待命 CMD */
                Pending_cmd_Dispatch_Ano_v(me);
            }
        }
    }
    return 0;
}

static int Send_String_Ano_s(stAnoBase *stBase, s32 lValue, char *pcstr)
{
    stAnoDevice *me = container_of(stBase, stAnoDevice, stBase);
    uint8_t _cnt = 0;
    me->pucTxBuffer[_cnt++] = 0xAA;
    me->pucTxBuffer[_cnt++] = 0xFF;
    me->pucTxBuffer[_cnt++] = 0xA1;
    me->pucTxBuffer[_cnt++] = 0;
    me->pucTxBuffer[_cnt++] = BYTE0(lValue);
    me->pucTxBuffer[_cnt++] = BYTE1(lValue);
    me->pucTxBuffer[_cnt++] = BYTE2(lValue);
    me->pucTxBuffer[_cnt++] = BYTE3(lValue);

    for (int i = 0; pcstr[i] != '\0'; i++)
    {
        me->pucTxBuffer[_cnt++] = pcstr[i];
        if (_cnt > 50) break;
    }
    me->pucTxBuffer[3] = _cnt - 4;

    u8 check_sum1 = 0, check_sum2 = 0;
    for (int i = 0; i < _cnt; i++)
    {
        check_sum1 += me->pucTxBuffer[i];
        check_sum2 += check_sum1;
    }
    me->pucTxBuffer[_cnt++] = check_sum1;
    me->pucTxBuffer[_cnt++] = check_sum2;
    me->stUser.Send_Buff(me->pucTxBuffer,_cnt);
    return 0;
}

/* ★ 数据消费的核心 —— ANO 层主动从 UART 的 RingBuffer 读字节喂状态机。
 *
 * 这是"两层 vtable 的咬合点 1"：
 *   ANO 层通过 uc_uart_read_ucbyte() 向 UART 层要字节，UART 层不知道
 *   要这些字节干嘛。这样 UART 层完全不依赖任何协议，只管搬字节。
 *
 * while 循环把 RingBuffer 里的字节逐个读出来，喂给状态机。读到空为止
 * （返回值!=0 表示空）。一次调用可能处理多个完整帧。
 *
 * 对比旧架构：旧版是 UART 层内部硬编码调协议解析，耦合死。现在反过来了：
 * 协议层主动来"要"字节，UART 层被动"给"。 */
static int Data_Check_Ano_s(stAnoBase *stBase,stUartBase* pstbase_uart)
{
    stAnoDevice *me = container_of(stBase, stAnoDevice, stBase);
    uint8_t data_temp;
    int frame_count = 0;
    while (uc_uart_read_ucbyte(pstbase_uart,&data_temp) == 0)
    {
        frame_count += Data_receive_prepare_Ano_s(&me->stBase,data_temp);
    }
    return frame_count;
}

static int Printf_Ano_s(stAnoBase *stBase, ano_printf_color_em color_em, const char *fmt, va_list ap)
{
    stAnoDevice *me = container_of(stBase, stAnoDevice, stBase);
    uint8_t _cnt = 0;
    char buffer[128];

    int len = vsnprintf(buffer, sizeof(buffer), fmt, ap);
    if (len < 0) return -1;
    if (len >= (int)sizeof(buffer)) len = sizeof(buffer) - 1;

    me->pucTxBuffer[_cnt++] = 0xAA;
    me->pucTxBuffer[_cnt++] = 0xFF;
    me->pucTxBuffer[_cnt++] = 0xA0;
    me->pucTxBuffer[_cnt++] = 0;
    me->pucTxBuffer[_cnt++] = (uint8_t)color_em;

    for (int i = 0; i < len; i++)
    {
        me->pucTxBuffer[_cnt++] = buffer[i];
        if (_cnt >= 58) break;
    }

    me->pucTxBuffer[3] = _cnt - 4;

    uint8_t check_sum1 = 0, check_sum2 = 0;
    for (int i = 0; i < _cnt; i++)
    {
        check_sum1 += me->pucTxBuffer[i];
        check_sum2 += check_sum1;
    }
    me->pucTxBuffer[_cnt++] = check_sum1;
    me->pucTxBuffer[_cnt++] = check_sum2;

    me->stUser.Send_Buff(me->pucTxBuffer, _cnt);
    return 0;
}

stAnoOps ano_operate = {
    .check_to_send = Check_to_send_Ano_s,
    .ck_back  = Ck_back_Ano_s,
    .cmd_send = Cmd_send_Ano_s,
    .par_back = Par_back_Ano_s,
    .sendID_set = Frame_num_set_Ano_s,
    .data_receive_prepare = Data_receive_prepare_Ano_s,
    .WTS_set =  WTS_Set_Ano_s,
    .wait_ck_clear = Wait_ck_clear_Ano_s,
    .ck_id_get = Ck_id_get_Ano,
    .ck_sc_get = Ck_sc_get_Ano,
    .ck_ac_get = Ck_ac_get_Ano,
    .cmd_cid_get = Cmd_cid_get_Ano,
    .cmd_copy_bytes = Cmd_copy_bytes_Ano,
    .par_id_get = Par_id_get_Ano,
    .par_val_get = Par_val_get_Ano,
    .ano_ck_back_check = CK_Back_Check_Ano_s,
    .send_string = Send_String_Ano_s,
    .ano_check_data = Data_Check_Ano_s,
    .ano_printf = Printf_Ano_s,
    .wait_ck_get_c = Wait_ck_get_c,
};

void Ano_Device_init(stAnoDevice* me,
                    _dt_st* ppstDeviceDt,
                    uint8_t* pucTxBuffer,
                    uint8_t* pucRxBuffer,
                    unique* pstUser)
{
    me->pstDeviceDt = ppstDeviceDt;
    me->pucRxBuffer = pucRxBuffer;
    me->pucTxBuffer = pucTxBuffer;
    me->stUser      = *pstUser;
    me->stBase.ops  = &ano_operate;
}
