// device_manager.h
#ifndef DEVICE_MANAGER_H_
#define DEVICE_MANAGER_H_

#include "main.h"

#define MAX_DEVICES 2  // 最大设备数量

typedef enum {
    DEVICE_STATE_UNINIT = 0,
    DEVICE_STATE_READY,
    DEVICE_STATE_BUSY,
    DEVICE_STATE_ERROR
} DeviceState_t;

typedef enum {
    DEVICE_TYPE_NONE = 0,
    DEVICE_TYPE_PWM,
    DEVICE_TYPE_POWER,
    DEVICE_TYPE_CURRENT_SENSOR,
    DEVICE_TYPE_VOLTAGE_SENSOR,
    DEVICE_TYPE_MAX
} DeviceType_t;

// 统一的命令类型
typedef enum {
    CMD_NONE = 0,
    CMD_PWM_SET_DUTY_SLOW,
    CMD_PWM_SET_DUTY_INSTANT,
    CMD_PWM_SET_FREQUENCY,
    CMD_PWM_SLOWSTART,
    CMD_PWM_STOP,
    CMD_SENSOR_ENABLE,
    CMD_SENSOR_DISABLE,
    CMD_POWER_ON,
    CMD_POWER_OFF,
    CMD_MAX
} DeviceCommand_t;

// 统一的命令参数结构 - 提前定义
typedef struct {
    DeviceCommand_t cmd;
    uint8_t device_id;
    void* params;           // 指向具体参数
    uint32_t param_size;    // 参数大小
} DeviceCommandData_t;

// 简化的设备操作接口
typedef struct {
    en_result_t (*init)(void* handle);
    en_result_t (*read)(void* handle, void* data, uint32_t size);
    en_result_t (*write)(void* handle, const void* data, uint32_t size);
    en_result_t (*control)(void* handle, DeviceCommandData_t* cmd);
    en_result_t (*update)(void* handle);  // 非阻塞更新
} DeviceOps_t;

// 设备节点
typedef struct {
    uint8_t id;
    char name[16];
    DeviceType_t type;
    DeviceState_t state;
    void* private_data;     // 指向具体设备的句柄
    DeviceOps_t ops;
    uint8_t used;  // 标记该位置是否已使用
} DeviceNode_t;

extern DeviceNode_t s_device_registry[MAX_DEVICES];

// 设备管理器接口
en_result_t DeviceManager_Init(void);
en_result_t DeviceManager_RegisterDevice(uint8_t id, const char* name, DeviceType_t type, 
                                       void* private_data, DeviceOps_t ops);
en_result_t DeviceManager_UnregisterDevice(uint8_t id);
DeviceNode_t* DeviceManager_GetDevice(uint8_t id);

// 统一的应用层接口
en_result_t Device_Init(uint8_t dev_id);
en_result_t Device_Read(uint8_t dev_id, void* data, uint32_t size);
en_result_t Device_Write(uint8_t dev_id, const void* data, uint32_t size);
en_result_t Device_Control(uint8_t dev_id, DeviceCommandData_t* cmd);
en_result_t Device_Update(uint8_t dev_id);  // 非阻塞更新

DeviceState_t Device_GetState(uint8_t dev_id);

// 调试接口
DeviceNode_t* DeviceManager_GetRegistry(void);
uint32_t DeviceManager_GetRegistrySize(void);

#endif
