#include <stdint.h>
#include <stdio.h>


#define NB_TAB_ECHO  4
#define MAX_ECHO 30

typedef uint16_t status_t;


/**
 * \struct _HAL_gyro_SI_t
 * \brief Gyroscope data with SI unit (rad/s)
 */
typedef struct _HAL_gyro_SI_t
{
    union{
        float v[3];
        struct
        {
            float x;
            float y;
            float z;
        };
    };
    float temperature; //celsius
    int temperature_lsb;
} HAL_gyro_SI_t;


/**
 * \struct _HAL_fifo_gyro_SI_t
 * \brief Gyroscope data with SI unit (rad/s)
 */
typedef struct _HAL_fifo_gyro_SI_t
{
    union{
        float v[3];
        struct
        {
            float x;
            float y;
            float z;
        };
    };
} HAL_fifo_gyro_SI_t;

/**
 * \struct _HAL_acc_SI_t
 * \brief Accelerometer data with SI unit (m/s2)
 */
typedef struct _HAL_acc_SI_t
{
    union{
        float v[3];
        struct
        {
            float x;
            float y;
            float z;
        };
    };
    float temperature; ///< Unit is Celsius
} HAL_acc_SI_t;

/**
 * \struct _HAL_magn_mg_t
 * \brief Magnetic data with mG unit
 */
typedef struct _HAL_magn_mG_t
{
    union{
        float v[3];
        struct
        {
            float x;
            float y;
            float z;
        };
    };
} HAL_magn_mG_t;


/**
 * \struct _HAL_pressurePS_SI_t
 * \brief Pressure data of pressure sensor with SI unit
 */
typedef struct _HAL_pressure_SI_t
{
    double __attribute__((aligned(8))) temperature; ///< Unit is Celsius
    float pressure;  ///< Unit is Pascal
} HAL_pressure_SI_t;





/**
 * \struct _HAL_vbat_SI_t
 * \brief  Voltage of battery powering device
 */
typedef struct _HAL_vbat_SI_t
{
    float vbat_V; ///< Battery voltage (unit : Volt)
    uint32_t vbat_percentage; ///< percentage of battery according of min and max of tension permitted
} HAL_vbat_SI_t;


/**
 * \struct _HAL_etron_gpio_t
 * \brief Etron Chip GPIOs
 */
typedef struct _HAL_etron_gpio_t
{
    int jump_half_load; ///< Input active high (inverted from HW)
    int onoff_button; ///< Input active high (inverted from HW)
    int charging; ///< Input active high
    int charge_done;
} HAL_etron_gpio_t;


/**
 * Echo information
 * */
typedef struct _HAL_echo_t
{

    uint16_t begin_echo_index; ///<index of raw data corresponding to the beggining of the echo (begin_echo_index/ultrasound_frequency_acquisition = time)
    uint16_t end_echo_index; ///<index of raw data corresponding to the end of the echo (end_echo_index/ultrasound_frequency_acquisition = time)
    int16_t max_value_index; ///<index of max value of the echo
    int32_t max_value; ///<Max value of the echo
    uint16_t precedent; ///<Number "id" of the echo matched in precedent ADC_acquistion serie
    int16_t d_echo; ///<Index difference between this echo and the echo matched in precedent ADC_acquisition serie
    uint16_t pre_max_index; ///<Index of raw data corresponding to maw_value minus noise
} HAL_echo_t;


/**
 * Array with echo found and number of echo found
 * */
typedef struct _HAL_list_echo_t
{
    HAL_echo_t tab_echo[MAX_ECHO]; ///<array with echo found
    uint8_t number_of_echoes; ///<number of echoes found
}HAL_list_echo_t;

enum
{
    HAL_US_status_No_Measure = 0,   //  1
    HAL_US_status_Inv_Data_number,  //  2
    HAL_US_status_No_Echo_Init,     //  4
    HAL_US_status_No_Echo,          //  8
    HAL_US_status_No_Matching,      // 16
};


typedef struct _HAL_ultrasound_SI_t
{
    float altitude; ///<Unit is meter
    float raw_altitude;
    uint16_t nb_echo;
    int measure_ref;
    int measure_status;
    uint8_t new_data;
    HAL_list_echo_t HAL_list_echo;
        HAL_list_echo_t HAL_list_echo_p;
} HAL_ultrasound_SI_t;


/**
 * Information used for processing altitude
 * */
typedef struct _HAL_ultrasound_result_t
{

    HAL_list_echo_t list_echo[NB_TAB_ECHO]; ///<array of last NB_TAB_ECHO succession acquisition process
    HAL_echo_t echo_altitude; ///<Echo chosen as "real" echo
    uint8_t index_list_echo_used; ///<Index of element of list echo used
    uint8_t index_last_list_echo_used; ///<Index of element of list used in last succession acquisition process
    uint8_t nb_echoes; //number of elements of actual list echo
    uint32_t sum_echo; ///<Value corresponding to area before ultrasound envelope
    int32_t gradient; ///<Gradient when envelope reaches threshold curve
}HAL_ultrasound_result_t;

/**
 * USB plugged or not
 * */
typedef enum HAL_VBUS_state_t
{
    HAL_VBUS_ON = 0,
    HAL_VBUS_OFF,
} HAL_VBUS_state_t;

/**
 * \struct _HAL_acquisition_t
 * \brief data result of data_format function
 */
typedef struct __attribute__((aligned(8))) _HAL_acquisition_t
{
    /*HAL_acquisition general information*/
    int number_HAL_read_call; ///< number of HAL_read() call
    int64_t timestamp; ///< timestamp corresponding to HAL_acquisition
    status_t status; ///<bitfield indicating if a sensors have been updated>
    uint8_t used; ///< indicating if HAL_acquisition is used or not
    uint8_t count_user; ///< number of threads using this element

    /* IMU data = acc/gyro on NONE-FIFO mode*/
    HAL_acc_SI_t HAL_acc_SI; ///< accelerometer data    TODO REMOVE this
    HAL_gyro_SI_t HAL_gyro_SI; ///< gyroscope data      TODO REMOVE this

    /*IMU data = acc + gyro on FIFO mode*/
    /*FIXME: please make a structure*/
    uint16_t HAL_fifo_count;        ///< Raw fifo size in the MPU6050 (debug purpose only)
    int64_t fifo_timestamp; ///< timestamp corresponding to the very last set of data read from fifo


    HAL_fifo_gyro_SI_t HAL_fifo_gyro_SI_TempCorr[5];  ///< gyroscope temperature corrected data in FIFO
    HAL_fifo_gyro_SI_t HAL_fifo_acce_SI_TempCorr[5];  ///< accelerometer temperature corrected data in FIFO
    union
    {
        HAL_fifo_gyro_SI_t HAL_fifo_gyro_SI_Raw[5];  ///< gyroscope data in FIFO
        HAL_fifo_gyro_SI_t HAL_fifo_gyro_SI[5]; ///< Fifo abstracter: pointer to choose between SI_Raw and SI_TempCorr
    };
    union
    {
        HAL_fifo_gyro_SI_t HAL_fifo_acce_SI_Raw[5];  ///< accelerometer data in FIFO
        HAL_fifo_gyro_SI_t HAL_fifo_acce_SI[5]; ///< Fifo abstracter: pointer to choose between SI_Raw and SI_TempCorr
    };

    float HAL_ref_IMU_temp; ///< Desired IMU temperature
    uint8_t HAL_fifo_fsync[5]; ///< imu fsync signal
    int HAL_fifo_size;                     ///< Size of each array HAL_fifo_gyro_SI and HAL_fifo_acce_SI

    /*Magnetometer data*/
    HAL_magn_mG_t HAL_magn_mG; ///< magnetic data

    /*Pressure data*/
    HAL_pressure_SI_t HAL_pressure_SI; ///< pressure data

    /*Ultrasound data*/
    HAL_ultrasound_SI_t HAL_ultrasound_SI; ///< ultrasound data

    uint8_t padding[52+144];

    /*Battery data*/
    HAL_vbat_SI_t HAL_vbat_SI; //Battery voltage value (unit : V)

} HAL_acquisition_t;




typedef enum _HAL_ultrasound_command_t
{
    ULTRASOUND_CMD_NO_CMD=0,
    ULTRASOUND_CMD_SET_MODE = 1,
    ULTRASOUND_CMD_SET_NB_PULSES = 2,
    ULTRASOUND_CMD_SET_VOLTAGE_MODE = 4,
    ULTRASOUND_CMD_SET_NB_PULSES_AND_TENSION_MODE = 6,
    ULTRASOUND_CMD_SET_FREQUENCY = 8,
    ULTRASOUND_CMD_START = 16,
    ULTRASOUND_CMD_STOP = 32,
} HAL_ultrasound_command_t;


typedef enum _HAL_BLDC_motor_command_t
{
    BLDC_CMD_NO_CMD=0,
    BLDC_CMD_START=1,
    BLDC_CMD_STOP=2,
    BLDC_CMD_RUN=3,
    BLDC_CMD_CLEAR=4,
    BLDC_CMD_REBOOT_BLDC=5,
    BLDC_CMD_LED=6,
} HAL_BLDC_motor_command_t;




typedef struct _HAL_command_t
{
    HAL_BLDC_motor_command_t command;

    int16_t* motors_speed;
    uint8_t  enable_security;

    int HAL_ultrasound_command;
    uint8_t ultrasound_mode;
    uint8_t nb_pulses;
    uint8_t voltage_mode;
    uint32_t acquisition_frequency_wanted;

    // LEDs commands :
    void* LEDs_cmds;

    int32_t HAL_leds_command;

} HAL_command_t;
