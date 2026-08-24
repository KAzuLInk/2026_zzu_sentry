#include "daemon.h"
#include "bsp_dwt.h" // 后续通过定时器来计时?
#include "stdlib.h"
#include "memory.h"
#include "buzzer.h"

// 用于保存所有的daemon instance
static DaemonInstance *daemon_instances[DAEMON_MX_CNT] = {NULL};
static uint8_t idx; // 用于记录当前的daemon instance数量,配合回调使用

/* 诊断: 每个daemon实例的实时状态, Ozone直接看这个数组(按注册idx排列) */
Daemon_Status_s daemon_status[DAEMON_MX_CNT];

/* ==================== 掉线报警音序 ==================== */
/* 任一模块离线: 循环"滴-滴"两声; 全部恢复在线: 一次性 do-re-mi 三声升阶。 */
static BuzzzerInstance *daemon_alarm_buzzer;
static uint8_t daemon_alarm_inited;

typedef enum
{
    DAEMON_ALARM_IDLE = 0, // 静默
    DAEMON_ALARM_OFFLINE,  // 循环滴-滴
    DAEMON_ALARM_RECOVER,  // 一次性升阶
} daemon_alarm_mode_e;

typedef struct
{
    uint8_t on;      // 1=响, 0=停
    uint16_t ms;     // 该阶段时长
    octave_e octave; // 响时的音高
} alarm_step_t;

// 掉线: 循环"滴-滴"
static const alarm_step_t offline_seq[] = {
    {1, 150, OCTAVE_1}, // 滴
    {0, 100, OCTAVE_1}, // 停
    {1, 150, OCTAVE_1}, // 滴
    {0, 400, OCTAVE_1}, // 长停
};

// 恢复: do-re-mi 三声升阶
static const alarm_step_t recover_seq[] = {
    {1, 150, OCTAVE_1}, // do
    {0, 80, OCTAVE_1},
    {1, 150, OCTAVE_2}, // re
    {0, 80, OCTAVE_2},
    {1, 150, OCTAVE_3}, // mi
    {0, 200, OCTAVE_3},
};

static daemon_alarm_mode_e daemon_alarm_mode = DAEMON_ALARM_IDLE;
static uint8_t daemon_alarm_phase;
static float daemon_alarm_stamp;

static const alarm_step_t *DaemonAlarmSeq(daemon_alarm_mode_e mode, uint8_t *len, uint8_t *loop)
{
    if (mode == DAEMON_ALARM_OFFLINE)
    {
        *len = (uint8_t)(sizeof(offline_seq) / sizeof(offline_seq[0]));
        *loop = 1;
        return offline_seq;
    }
    if (mode == DAEMON_ALARM_RECOVER)
    {
        *len = (uint8_t)(sizeof(recover_seq) / sizeof(recover_seq[0]));
        *loop = 0;
        return recover_seq;
    }
    *len = 0;
    *loop = 0;
    return NULL;
}

static void DaemonAlarmApply(const alarm_step_t *seq, uint8_t phase)
{
    if (seq[phase].on)
    {
        daemon_alarm_buzzer->octave = seq[phase].octave;
        AlarmSetStatus(daemon_alarm_buzzer, ALARM_ON);
    }
    else
    {
        AlarmSetStatus(daemon_alarm_buzzer, ALARM_OFF);
    }
}

static void DaemonAlarmEnsureInit(void)
{
    if (daemon_alarm_inited)
        return;
    Buzzer_config_s cfg = {
        .alarm_level = ALARM_LEVEL_HIGH,
        .loudness = 0.5f,
        .octave = OCTAVE_1,
    };
    daemon_alarm_buzzer = BuzzerRegister(&cfg);
    daemon_alarm_inited = 1;
}

static void DaemonAlarmSetMode(daemon_alarm_mode_e mode)
{
    if (daemon_alarm_mode == mode)
        return;
    daemon_alarm_mode = mode;
    daemon_alarm_phase = 0;
    daemon_alarm_stamp = DWT_GetTimeline_ms();

    if (mode == DAEMON_ALARM_IDLE)
    {
        AlarmSetStatus(daemon_alarm_buzzer, ALARM_OFF);
        return;
    }
    const alarm_step_t *seq = (mode == DAEMON_ALARM_OFFLINE) ? offline_seq : recover_seq;
    DaemonAlarmApply(seq, 0);
}

static void DaemonAlarmProcess(void)
{
    if (daemon_alarm_mode == DAEMON_ALARM_IDLE)
        return;

    uint8_t len, loop;
    const alarm_step_t *seq = DaemonAlarmSeq(daemon_alarm_mode, &len, &loop);
    if (seq == NULL)
        return;

    if (DWT_GetTimeline_ms() - daemon_alarm_stamp >= seq[daemon_alarm_phase].ms)
    {
        daemon_alarm_phase++;
        if (daemon_alarm_phase >= len)
        {
            if (loop)
            {
                daemon_alarm_phase = 0;
            }
            else
            {
                daemon_alarm_mode = DAEMON_ALARM_IDLE;
                AlarmSetStatus(daemon_alarm_buzzer, ALARM_OFF);
                return;
            }
        }
        daemon_alarm_stamp = DWT_GetTimeline_ms();
        DaemonAlarmApply(seq, daemon_alarm_phase);
    }
}

DaemonInstance *DaemonRegister(Daemon_Init_Config_s *config)
{
    DaemonInstance *instance = (DaemonInstance *)malloc(sizeof(DaemonInstance));
    memset(instance, 0, sizeof(DaemonInstance));

    instance->owner_id = config->owner_id;
    instance->reload_count = config->reload_count == 0 ? 100 : config->reload_count; // 默认值为100
    instance->callback = config->callback;
    instance->alarm_silent = config->alarm_silent;
    instance->name = config->name;
    // init_count 为 0 时用 reload_count 作为初始计数; 否则用 init_count(上电"上线宽限时间")
    instance->temp_count = config->init_count == 0 ? instance->reload_count : config->init_count;

    daemon_instances[idx++] = instance;
    return instance;
}

/* "喂狗"函数 */
void DaemonReload(DaemonInstance *instance)
{
    instance->temp_count = instance->reload_count;
    instance->reload_cnt++; // 诊断: 喂狗计数
}

uint8_t DaemonIsOnline(DaemonInstance *instance)
{
    return instance->temp_count > 0;
}

void DaemonTask()
{
    DaemonInstance *dins;   // 提高可读性同时降低访存开销
    // uint8_t offline_cnt = 0; // 当前离线的实例数

    // DaemonAlarmEnsureInit(); // 掉线提示音暂时关闭

    for (size_t i = 0; i < idx; ++i)
    {

        dins = daemon_instances[i];
        if (dins->temp_count > 0) // 如果计数器还有值,说明上一次喂狗后还没有超时,则计数器减一
        {
            dins->temp_count--;
            // 刚减到 0 = 本次超时边沿, 只触发一次回调; 之后停在 0 不再重复触发, 直到喂狗重新上线
            if (dins->temp_count == 0 && dins->callback)
                dins->callback(dins->owner_id); // module内可以将owner_id强制类型转换成自身类型从而调用特定module的offline callback
        }
        else
        {
            // temp_count == 0 表示模块离线；提示音暂时关闭。
            // if (!dins->alarm_silent)
            //     offline_cnt++;
        }

        // 诊断: 刷新统一状态数组, Ozone看 daemon_status[] 即可
        daemon_status[i].name = dins->name;
        daemon_status[i].online = (dins->temp_count > 0) ? 1 : 0;
        daemon_status[i].reload_cnt = dins->reload_cnt;
    }

    // 掉线报警暂时关闭；需要恢复时取消下面整段注释。
    // if (offline_cnt > 0)
    // {
    //     if (daemon_alarm_mode != DAEMON_ALARM_OFFLINE)
    //         DaemonAlarmSetMode(DAEMON_ALARM_OFFLINE);
    // }
    // else if (daemon_alarm_mode == DAEMON_ALARM_OFFLINE)
    // {
    //     DaemonAlarmSetMode(DAEMON_ALARM_RECOVER);
    // }
    // DaemonAlarmProcess();
}
// (需要id的原因是什么?) 下面是copilot的回答!
// 需要id的原因是因为有些module可能有多个实例,而我们需要知道具体是哪个实例offline
// 如果只有一个实例,则可以不用id,直接调用callback即可
// 比如: 有一个module叫做"电机",它有两个实例,分别是"电机1"和"电机2",那么我们调用电机的离线处理函数时就需要知道是哪个电机offline
