#include "ranging_vl53l0x.h"
#include "vl53l0x_api.h"

#define IR_N                            1       // число датчиков в диапазоне от 1 до 32
#define FAULT_LEVEL                     IR_N    //определяет число датчиков, которые должны выдать ошибку для того, чтобы система обнаружила критическую ошибку,
                                                //и предприняла попытку полностью перезагрузить модуль
#define SINGLE_ERRORS_UNTIL_REINIT      3       //число циклов опроса, в течение которых данные с датчика могут быть не получены.
                                                //при превышении - происходит переинициализация этого, а также всех остальных датчиков,
                                                //у которых к моменту конца цикла зарегистрирована хотя бы одна ошибка

GPIO_TypeDef*   IR_GPIO_PORT[IR_N]=
{
    GPIOB
};

uint16_t        IR_GPIO_PIN[IR_N]=
{
    GPIO_PIN_0  
};

VL53L0X_RangingMeasurementData_t    IR_range_data[IR_N];                //под хранение результата измерений
VL53L0X_Dev_t                       IR_device[IR_N];                    //хранение настроек каждого датчика
int8_t                              IR_device_errors[IR_N];             //хранение кодов ошибок.  как только общее число ошибок превышает пороговое значение, происходит переинициализация всего.
uint8_t                             IR_device_errors_total[IR_N];       //число подряд идущих обращений у датчику, в результате которых по любой причине не были получены данные. 
                                                                        //является маркером работоспособности отдельно взятого датчика. 
                                                                        //когда превышает пороговое значение, происходит переинициализация соответствующего датчика датчика.
                                                                        //также используется в методе IR_IfDataCorrect

uint8_t                             IR_device_reinit_running[IR_N];     //флаг того, тчо какой то датчик переинициализируется.

/**************************************************************************************************
Описание:  Возвращает факт наличия ошибки хотя бы на одном из датчиков
Аргументы: нет
Возврат:   число датчиков с ошибкой
Замечания: 0, если ошибки нету
**************************************************************************************************/
uint8_t IR_IfErrorsOccured(void)
{
    uint8_t errors_count=0;
    uint8_t count;
    for(count=0;count<IR_N;count++)
    {
        if(IR_device_errors[count]!=VL53L0X_ERROR_NONE)
        {
            errors_count++;
        }
    }
    return errors_count;    
}

/**************************************************************************************************
Описание:  Возвращает код ошибки определенного датчика
Аргументы: номер датчика
Возврат:   код ошибки
Замечания: 0, если ошибки нету
**************************************************************************************************/
int8_t IR_GetErrorID(uint8_t N)
{
    return IR_device_errors[N];
}

/**************************************************************************************************
Описание:  Возвращает показания измерений определенного датчика
Аргументы: номер датчика
Возврат:   расстояние в мм
Замечания: если расстояние первышает допстимый предел, выдает 8192
**************************************************************************************************/
uint16_t IR_GetRange(uint8_t N)
{
    return IR_range_data[N].RangeMilliMeter;    
}

/**************************************************************************************************
Описание:  реинициализация отдельно взятого датчика
Аргументы: номер датчика для реинициализации
Возврат:   нет
Замечания: нет
**************************************************************************************************/
uint8_t IR_ReInit(uint8_t N)
{
    static enum
    {
        STAGE1,
        WAIT,
        STAGE2,
        STAGE3,
        STAGE4,
        STAGE5,
        STAGE6,
        STAGE7,
        STAGE8,
        STAGE9,
        STAGE10,
        STAGE11,
        STAGE12,
        STAGE13        
    } state=STAGE1;
    static uint32_t start;
    
    static VL53L0X_Error Status;                           //под хранение кода ошибки
    static uint32_t refSpadCount;                      //для процесса конфигурации датчиков
    static uint8_t  isApertureSpads;                    //для процесса конфигурации датчиков
    static uint8_t  VhvSettings;                          //для процесса конфигурации датчиков
    static uint8_t  PhaseCal;                             //для процесса конфигурации датчиков

    switch(state)
    {
        case STAGE1:
        {
            HAL_GPIO_WritePin(IR_GPIO_PORT[N],IR_GPIO_PIN[N],GPIO_PIN_RESET);           //отправляем датчики спать. ноги сконфигурированы как open drain без подтяжки
            IR_device[N].I2cDevAddr=0x52;                                               //заполняем массив дефолтными адресами датчиков
            start=HAL_GetTick();            
            refSpadCount =    0;   
            isApertureSpads = 0;
            VhvSettings =     0;      
            PhaseCal =        0;        
            
            state=WAIT;
            return 0;
        }
        case WAIT:
        {
            if(HAL_GetTick()>=start+2)
            {
                state=STAGE2;                
            }
            return 0;            
        }
        case STAGE2:
        {
            HAL_GPIO_WritePin(IR_GPIO_PORT[N],IR_GPIO_PIN[N],GPIO_PIN_SET);             //будим датчик
            Status=VL53L0X_ERROR_NONE;                                                  //сбрасываем код ошибки        
            Status=VL53L0X_SetDeviceAddress(&IR_device[N],0x52+2*(N+1));                //меняем адрес датчика         
           
            state=STAGE3;
            return 0;
        }
        case STAGE3:
        {    
            if (Status == VL53L0X_ERROR_NONE) 
            {
                IR_device[N].I2cDevAddr=0x52+2*(N+1);                                                           
                Status=VL53L0X_DataInit(&IR_device[N]);
            }
            
            state=STAGE4;
            return 0;            
        }
        case STAGE4:
        {            
            if (Status == VL53L0X_ERROR_NONE) 
            {
                Status=VL53L0X_StaticInit(&IR_device[N]);	
            }
            
            state=STAGE5;
            return 0;
        }
        case STAGE5:
        {            
            if (Status == VL53L0X_ERROR_NONE) 
            {
                Status = VL53L0X_PerformRefSpadManagement(&IR_device[N], &refSpadCount, &isApertureSpads);
            }
            
            state=STAGE6;
            return 0;  
        }
        case STAGE6:
        {            
            if (Status == VL53L0X_ERROR_NONE) 
            {
                Status = VL53L0X_PerformRefCalibration(&IR_device[N], &VhvSettings, &PhaseCal);
            }
            
            state=STAGE7;
            return 0;            
        }
        case STAGE7:
        {
            if (Status == VL53L0X_ERROR_NONE) 
            {
                Status=VL53L0X_SetReferenceSpads(&IR_device[N], refSpadCount, isApertureSpads);
            }      
            
            state=STAGE8;
            return 0;            
        }
        case STAGE8:
        {            
            if (Status == VL53L0X_ERROR_NONE) 
            {
                Status=VL53L0X_SetRefCalibration(&IR_device[N], VhvSettings, PhaseCal);
            }
            
            state=STAGE9;
            return 0;             
        }
        case STAGE9:
        {
            if (Status == VL53L0X_ERROR_NONE) 
            {
                Status=VL53L0X_SetDeviceMode(&IR_device[N],VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);	
            }
            
            state=STAGE10;
            return 0;
        }
        case STAGE10:
        {
            if (Status == VL53L0X_ERROR_NONE) 
            {
                Status = VL53L0X_SetLimitCheckValue(&IR_device[N],VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE,(FixPoint1616_t)(0.25*65536));
            }
            
            state=STAGE11;
            return 0;            
        }
        case STAGE11:
        {            
            if (Status == VL53L0X_ERROR_NONE) 
            {
                Status = VL53L0X_SetLimitCheckValue(&IR_device[N],VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE,(FixPoint1616_t)(32*65536));
            }
            
            state=STAGE12;
            return 0; 
        }
        case STAGE12:
        {            
            if (Status == VL53L0X_ERROR_NONE) 
            {
                Status =VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&IR_device[N],	20000);
            }
            
            state=STAGE13;
            return 0; 
        }            
        case STAGE13:
        {
            if (Status == VL53L0X_ERROR_NONE) 
            {
                Status=VL53L0X_StartMeasurement(&IR_device[N]);
            }
            
            IR_device_errors[N]=Status;
            
            state=STAGE1;
            return 1;            
        }        
    }    
    return 0;
}

/**************************************************************************************************
Описание:  реинициализация всех подключенных датчиков
Аргументы: номер датчика для реинициализации
Возврат:   закончена ли переинициализация
Замечания: нет
**************************************************************************************************/
uint8_t IR_ReInitAll(void)
{
    uint8_t count=0;  
    
    for(count=0;count<IR_N;count++)
    {
        HAL_GPIO_WritePin(IR_GPIO_PORT[count],IR_GPIO_PIN[count],GPIO_PIN_RESET);           //отправляем датчики спать. ноги сконфигурированы как open drain без подтяжки
        IR_device[count].I2cDevAddr=0x52;                                                   //заполняем массив дефолтными адресами датчиков
        IR_device[count].Present=0;
        IR_device[count].Id=count;    
    }

    for(count=0;count<IR_N;count++)
    {
        while(!IR_ReInit(count));
    } 
    
    return 0;   
}


/**************************************************************************************************
Описание:  Инициализация модуля и заодно датчиков, должна быть вызвана извне
Аргументы: -
Возврат:   число датчиков с ошибкой в процессе инициализации
Замечания: 0, если ошибки нету
**************************************************************************************************/
uint8_t IR_Init(I2C_HandleTypeDef* i2c_handle)
{
    uint8_t count=0;  
    for(count=0;count<IR_N;count++)
    {
        IR_device[count].I2cHandle=i2c_handle;                                              //передаем указатель на I2C_HandleTypeDef
    }
    IR_ReInitAll();
    return IR_IfErrorsOccured();  
}

/**************************************************************************************************
Описание:  метод последовательнго опроса массива датчиков
Аргументы: -
Возврат:   код возникшей ошибки
Замечания: 0, если ошибки нету
**************************************************************************************************/
uint8_t IR_Process(void)
{
    static uint8_t count;           // счетчик для периодического опроса датчиков
    uint8_t i;                      //для перебора числа накопленных ошибок
    uint8_t reinit_required;        //флаг того, что надо переинициализировать некоторые датчики.
    static VL53L0X_Error Status;
    static uint8_t data_ready;      //флаг готовности результата измерений
    static uint8_t fix_count;
    
    static enum
    {
        LAUNCH,
        SET_NEXT,
        ASK_READY,
        ASK_RESULT,
    } work_state=LAUNCH;
    
    static enum
    {
        CHECK,
        FIX_SELECTED,     
    }   fix_state=CHECK;
    
    switch (fix_state)
    {
        case CHECK:
        {
            reinit_required=0;
            if(count==0)                    //раз в прогон - проверяем накопившиеся ошибки
            {
                for(i=0;i<IR_N;i++)         //не накопилось ли на каком либо датчике подряд идущие ошибки/неответы.
                {
                    if(IR_device_errors_total[i]>=SINGLE_ERRORS_UNTIL_REINIT)
                    {
                        IR_device_reinit_running[i]=1;
                        reinit_required++;
                        continue;
                    }            
                }
                
                if(reinit_required)         //если обнаружено говно хотя бы на одном датчике
                {
                    if(reinit_required<FAULT_LEVEL)     //чиним отдельно взятые
                    {          
                        for(i=0;i<IR_N;i++)             //то сперва не даем опросу трогать обосравшиеся датчики
                        {
                            if(IR_device_errors_total[i]!=0)
                            {
                                IR_device_reinit_running[i]=1;                           
                            }  
                        }                                                     
                    }
                    else                                //чиним все
                    {                        
                        for(i=0;i<IR_N;i++)             //то сперва не даем опросу трогать обосравшиеся датчики
                        {
                            IR_device_reinit_running[i]=1;                           
                        }
                        
                    }
                    fix_count=0;
                    fix_state=FIX_SELECTED; 
                } 
            }
            break;           
        }
        case FIX_SELECTED:
        {
            if(fix_count<IR_N)
            { 
                if(IR_device_reinit_running[fix_count])
                {                    
                    if(IR_ReInit(fix_count))
                    {                        
                        IR_device_reinit_running[fix_count]=0;
                        IR_device_errors_total[fix_count]=0;
                        fix_count++;
                    } 
                }
                else    
                {
                    fix_count++;
                }                    
            }
            else
            {
                fix_count=0;
                fix_state=CHECK;                
            }   
            break;
        }        
    } 
    
    switch(work_state)
    {
        case LAUNCH:
        {
            count=0;
            Status=VL53L0X_ERROR_NONE;
            data_ready=0;
            work_state=SET_NEXT;
            break;
        }
        case SET_NEXT:
        {
            count++;
            count=count%IR_N;
            work_state=ASK_READY;
            break;
        }
        case ASK_READY:
        {
            if(IR_device_reinit_running[count])     //если текущий датчик реинитится, то переключаемся к следующему
            {
                work_state=SET_NEXT;
                break;
            }                
            
            Status=VL53L0X_GetMeasurementDataReady(&IR_device[count],&data_ready);
            if (Status!=VL53L0X_ERROR_NONE)
            {
                IR_range_data[count].RangeMilliMeter=0xFFFF;          //в любой непонятной ситуации - считаем, что препятствия перед датчиком нет   
                IR_device_errors_total[count]++;
                work_state=SET_NEXT;
            }
            else
            {
                if(data_ready)
                {                    
                    work_state=ASK_RESULT;
                }
                else
                {
                    IR_range_data[count].RangeMilliMeter=0xFFFF;      //в любой непонятной ситуации - считаем, что препятствия перед датчиком нет   
                    IR_device_errors_total[count]++;                
                    work_state=SET_NEXT;
                }                
            }           
            break;
        }
        case ASK_RESULT:
        { 
            if(IR_device_reinit_running[count])     //если текущий датчик реинитится, то переключаемся к следующему
            {
                work_state=SET_NEXT;
                break;
            }
            
            Status=VL53L0X_GetRangingMeasurementData(&IR_device[count],&IR_range_data[count]);            
            if (Status == VL53L0X_ERROR_NONE) 
            {
                IR_device_errors_total[count]=0;
                Status=VL53L0X_ClearInterruptMask(&IR_device[count],0);
            }
            else
            {
                IR_range_data[count].RangeMilliMeter=0xFFFF;          //в любой непонятной ситуации - считаем, что препятствия перед датчиком нет          
                IR_device_errors_total[count]++;
            }
            work_state=SET_NEXT;
            break;            
        } 
    }
    IR_device_errors[count]=Status; 
    return IR_IfErrorsOccured();    
}
