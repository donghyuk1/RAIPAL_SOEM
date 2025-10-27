#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "ethercat.h"

#define EC_TIMEOUTMON 500

// ---- Configuration you may tweak ----
#define SLAVE_IDX                   1          // motor driver is slave #1
#define TARGET_POSITION_BYTE_OFFSET            0x00  // int32
#define TARGET_VELOCITY_BYTE_OFFSET            0x04  // int32
#define TARGET_TORQUE_BYTE_OFFSET              0x08  // int16
#define CONTROLWORD_BYTE_OFFSET                0x0A  // uint8  (your map uses 8-bit CW)
#define MODE_OF_OPERATION_BYTE_OFFSET          0x0B  // uint8
#define IMPEDANCE_PROPORTIONAL_GAIN_BYTE_OFFSET 0x0C // uint16
#define IMPEDANCE_DERIVATIVE_GAIN_BYTE_OFFSET   0x0E // uint16


#define POSITION_ACTUAL_VALUE_BYTE_OFFSET    0x00  // int32
#define TORQUE_ACTUAL_VALUE_BYTE_OFFSET      0x04  // int16
#define VELOCITY_ACTUAL_VALUE_BYTE_OFFSET    0x06  // int32
#define STATUSWORD_BYTE_OFFSET               0x0A  // uint8
#define TEMPERATURE_BYTE_OFFSET              0x0B  // uint8
#define ERROR_CODE_BYTE_OFFSET               0x0C  // uint8


// ---- Modes of Operation (examples) ----
#define NO_MODE                                0x00
#define MODE_PROFILE_POSITION                  0x01
#define MODE_PROFILE_VELOCITY                  0x03
#define MODE_PROFILE_TORQUE                    0x04
#define MODE_HOMING                            0x06
#define MODE_INTERPOLATION_POSITION            0x07 /**< \brief Interpolation Position mode*/
#define MODE_CYCLIC_SYNC_POSITION              0x08 /**< \brief Cyclic Synchronous Position mode*/
#define MODE_CYCLIC_SYNC_VELOCITY              0x09 /**< \brief Cyclic Synchronous Velocity mode*/
#define MODE_CYCLIC_SYNC_TORQUE_MODE           0x0A/**< \brief Cyclic Synchronous Torque mode*/
#define MODE_PROFILE_COMMUTATION               0x0F

// ---- Controlword commands ----
#define CONTROLWORD_SHUTDOWN                   0x06
#define CONTROLWORD_SWITCH_ON                  0x07  // (0x0007 low byte)
#define CONTROLWORD_ENABLE_OPERATION           0x0F  // (0x000F low byte)
#define CONTROLWORD_DISABLE_OPERATION          0x07
#define CONTROLWORD_FAULTRESET                 0x80


#define STATUSWORD_STATE_NOTREADYTOSWITCHON                  0x0000 /**< \brief Not ready to switch on*/
/* ECATCHANGE_START(V5.12) CIA402 1*/
#define STATUSWORD_STATE_SWITCHEDONDISABLED                  0x0040 /**< \brief Switched on but disabled*/
/* ECATCHANGE_END(V5.12) CIA402 1*/
#define STATUSWORD_STATE_READYTOSWITCHON                     0x0021 /**< \brief Ready to switch on*/
#define STATUSWORD_STATE_SWITCHEDON                          0x0023 /**< \brief Switched on*/
#define STATUSWORD_STATE_OPERATIONENABLED                    0x0027 /**< \brief Operation enabled*/
#define STATUSWORD_STATE_QUICKSTOPACTIVE                     0x0007 /**< \brief Quickstop active*/
#define STATUSWORD_STATE_FAULTREACTIONACTIVE                 0x000F /**< \brief Fault reaction active*/
#define STATUSWORD_STATE_FAULT                               0x0008 /**< \brief Fault state*/


#define PRINT_PERIOD_US     5000       // 5 ms loop


// Optional: force aligned mapping (same toggle you had)
static boolean forceByteAlignment = FALSE;

// Globals (same as SOEM examples)
static char IOmap[4096];
static OSAL_THREAD_HANDLE thread1;
static int expectedWKC;
static volatile int wkc;
static boolean inOP = FALSE;
static boolean needlf = FALSE;
static uint8 currentgroup = 0;

typedef enum {
    PHASE_SWITCH_OFF = 0,
    PHASE_SWITCH_ON,     // send 0x07
    PHASE_ENABLE_OPERATION   // then 0x0F
} ControlPhase;


typedef struct {
    uint8_t  controlword;                 // 0x6040 low byte (your PDO map)
    uint8_t  mode_of_operation;           // 0x6060
    int32_t  target_position;             // 0x607A
    int32_t  target_velocity;             // 0x60FF
    int16_t  target_torque;               // 0x6071
    uint16_t impedance_proportional_gain; // 0x6081
    uint16_t impedance_derivative_gain;   // 0x6082
} DriveOutputs;


typedef struct {
    int32_t position_actual_value;    // 0x6064:00 (int32)
    int16_t torque_actual_value;      // 0x6077:00 (int16)
    int32_t velocity_actual_value;    // 0x606C:00 (int32)
    uint8_t statusword;      // 0x6041:00 (low byte only in your map)
    uint8_t temperature;              // vendor map
    uint8_t error_code;               // vendor map
} SlaveInputData;

static inline void write_uint8 (uint8_t *base, int byte_offset, uint8_t value) {
    base[byte_offset] = value;
}
static inline void write_int16 (uint8_t *base, int byte_offset, int16_t value) {
    int16_t le = htoes(value);
    memcpy(base + byte_offset, &le, 2);
}
static inline void write_uint16(uint8_t *base, int byte_offset, uint16_t value) {
    uint16_t le = htoes(value);
    memcpy(base + byte_offset, &le, 2);
}
static inline void write_int32 (uint8_t *base, int byte_offset, int32_t value) {
    int32_t le = htoel(value);
    memcpy(base + byte_offset, &le, 4);
}

static inline int32_t read_int32 (const uint8_t *base, int byte_offset) {
    int32_t v; memcpy(&v, base + byte_offset, 4); return etohl(v);
}
static inline int16_t read_int16 (const uint8_t *base, int byte_offset) {
    int16_t v; memcpy(&v, base + byte_offset, 2); return etohs(v);
}
static inline uint8_t read_uint8 (const uint8_t *base, int byte_offset) {
    return base[byte_offset];
}

static void apply_drive_outputs(uint8_t *outputs_base, const DriveOutputs *out)
{
    write_uint8 (outputs_base, CONTROLWORD_BYTE_OFFSET,                  out->controlword);
    write_uint8 (outputs_base, MODE_OF_OPERATION_BYTE_OFFSET,            out->mode_of_operation);
    write_int32 (outputs_base, TARGET_POSITION_BYTE_OFFSET,              out->target_position);
    write_int32 (outputs_base, TARGET_VELOCITY_BYTE_OFFSET,              out->target_velocity);
    write_int16 (outputs_base, TARGET_TORQUE_BYTE_OFFSET,                out->target_torque);
    write_uint16(outputs_base, IMPEDANCE_PROPORTIONAL_GAIN_BYTE_OFFSET,  out->impedance_proportional_gain);
    write_uint16(outputs_base, IMPEDANCE_DERIVATIVE_GAIN_BYTE_OFFSET,    out->impedance_derivative_gain);
}

static void read_slave_input_data(int slave_index, SlaveInputData *out)
{
    const uint8_t *inputs_base = ec_slave[slave_index].inputs;

    out->position_actual_value  = read_int32(inputs_base, POSITION_ACTUAL_VALUE_BYTE_OFFSET);
    out->torque_actual_value    = read_int16(inputs_base, TORQUE_ACTUAL_VALUE_BYTE_OFFSET);
    out->velocity_actual_value  = read_int32(inputs_base, VELOCITY_ACTUAL_VALUE_BYTE_OFFSET);
    out->statusword             = read_uint8(inputs_base, STATUSWORD_BYTE_OFFSET);
    out->temperature            = read_uint8(inputs_base, TEMPERATURE_BYTE_OFFSET);
    out->error_code             = read_uint8(inputs_base, ERROR_CODE_BYTE_OFFSET);
}
//
// static void read_all_slaves_input_data(SlaveInputData *out_array)
// {
//     for (int s = 1; s <= ec_slavecount; s++)
//     {
//         read_slave_input_data(s, &out_array[s]);
//     }
// }

static void print_slave_input_data(const SlaveInputData *in)
{
    printf(" | pos=%" PRId32 " vel=%" PRId32 " tor=%" PRId16
           " SWlo=0x%02X temp=%u err=0x%02X",
           in->position_actual_value,
           in->velocity_actual_value,
           in->torque_actual_value,
           in->statusword,
           (unsigned)in->temperature,
           (unsigned)in->error_code);
}

static void print_process_data(int cycle_index, int wkc, int expectedWKC,
                               int oloop, int iloop, int slave_index)
{
    printf("Processdata cycle %4d, WKC %d , O:", cycle_index, wkc);

    // Outputs (master → slave = RxPDO)
    for (int j = 0; j < oloop; j++)
        printf(" %2.2x", *(ec_slave[slave_index].outputs + j));

    printf(" I:");
    // Inputs (slave → master = TxPDO)
    for (int j = 0; j < iloop; j++)
        printf(" %2.2x", *(ec_slave[slave_index].inputs + j));

    printf(" T:%" PRId64 "\r", ec_DCtime);
    fflush(stdout);
}

//
// static void print_all_slaves_process_data(int cycle_index, int wkc, int expectedWKC)
// {
//     for (int s = 1; s <= ec_slavecount; s++)
//     {
//         int oloop = ec_slave[s].Obytes;
//         if ((oloop == 0) && (ec_slave[s].Obits > 0)) oloop = 1;
//         if (oloop > 8) oloop = 8;
//
//         int iloop = ec_slave[s].Ibytes;
//         if ((iloop == 0) && (ec_slave[s].Ibits > 0)) iloop = 1;
//         if (iloop > 8) iloop = 8;
//
//         print_process_data(cycle_index, wkc, expectedWKC, oloop, iloop, s);
//     }
// }



OSAL_THREAD_FUNC ecatcheck(void *ptr)
{
    (void)ptr;
    while (1)
    {
        if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf) { needlf = FALSE; printf("\n"); }

            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (int slave = 1; slave <= ec_slavecount; slave++)
            {
                if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
                {
                    ec_group[currentgroup].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                    {
                        printf("ERROR : slave %d SAFE_OP + ERROR, attempting ack.\n", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
                    {
                        printf("WARNING : slave %d SAFE_OP, change to OPERATIONAL.\n", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state > EC_STATE_NONE)
                    {
                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d reconfigured\n", slave);
                        }
                    }
                    else if (!ec_slave[slave].islost)
                    {
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (ec_slave[slave].state == EC_STATE_NONE)
                        {
                            ec_slave[slave].islost = TRUE;
                            printf("ERROR : slave %d lost\n", slave);
                        }
                    }
                }
                if (ec_slave[slave].islost)
                {
                    if (ec_slave[slave].state == EC_STATE_NONE)
                    {
                        if (ec_recover_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d recovered\n", slave);
                        }
                    }
                    else
                    {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d found\n", slave);
                    }
                }
            }
            if (!ec_group[currentgroup].docheckstate)
                printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(10000);
    }
}



static void simpletest(const char *ifname)
{
    needlf = FALSE;
    inOP = FALSE;

    printf("Starting Step-1 motor test on %s\n", ifname);

    if (!ec_init(ifname))
    {
        printf("No socket connection on %s\nExecute as root\n", ifname);
        return;
    }
    printf("ec_init on %s succeeded.\n", ifname);

    if (ec_config_init(FALSE) <= 0)
    {
        printf("No slaves found!\n");
        ec_close();
        return;
    }
    printf("%d slaves found and configured.\n", ec_slavecount);

    if (forceByteAlignment)
        ec_config_map_aligned(&IOmap);
    else
        ec_config_map(&IOmap);

    ec_configdc();

    printf("Slaves mapped, state to SAFE_OP.\n");
    ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);

    // Determine process image sizes for the selected slave
    int oloop = ec_slave[SLAVE_IDX].Obytes;
    if ((oloop == 0) && (ec_slave[SLAVE_IDX].Obits > 0)) oloop = 1;
    int iloop = ec_slave[SLAVE_IDX].Ibytes;
    if ((iloop == 0) && (ec_slave[SLAVE_IDX].Ibits > 0)) iloop = 1;

    printf("segments : %d : %d %d %d %d\n",
           ec_group[0].nsegments,
           ec_group[0].IOsegment[0], ec_group[0].IOsegment[1],
           ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);

    printf("Request operational state for all slaves\n");
    expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
    printf("Calculated workcounter %d\n", expectedWKC);
    ec_slave[0].state = EC_STATE_OPERATIONAL;

    // STEP-1 REQUIREMENT: send all zeros to the slave RxPDO
    memset(ec_slave[SLAVE_IDX].outputs, 0, ec_slave[SLAVE_IDX].Obytes);
    ec_send_processdata();
    ec_receive_processdata(EC_TIMEOUTRET);

    // request OP
    ec_writestate(0);
    int chk = 200;
    do
    {
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
    }
    while (chk-- && (ec_slave[0].state != EC_STATE_OPERATIONAL));

    if (ec_slave[0].state == EC_STATE_OPERATIONAL)
    {
        printf("Operational state reached for all slaves.\n");
        inOP = TRUE;

        // /* ---- Per-slave shadows ---- */
        // static DriveOutputs   desired_outputs_shadow[EC_MAXSLAVE + 1];
        // static SlaveInputData slave_inputs[EC_MAXSLAVE + 1];
        //
        //
        // /* Template you had */
        // DriveOutputs desired_outputs_template = {
        //     .controlword                 = CONTROLWORD_FAULTRESET,
        //     .mode_of_operation           = 0,
        //     .target_position             = 0,
        //     .target_velocity             = 0,
        //     .target_torque               = 0,
        //     .impedance_proportional_gain = 0,
        //     .impedance_derivative_gain   = 0
        // };
        //
        // /* Initialize identical defaults for all slaves */
        // for (uint16_t s = 1; s <= ec_slavecount; ++s) {
        //     desired_outputs_shadow[s] = desired_outputs_template;
        // }
        //
        // /* (Optional) If you still use this elsewhere keep it, otherwise remove */
        // ControlPhase control_phase = PHASE_SWITCH_ON;
        //
        // /* ---- cyclic loop ---- */
        // for (int i = 1; i <= 10000; ++i)
        // {
        //     /* 1) Write outputs for every slave into PDO image */
        //     for (uint16_t s = 1; s <= ec_slavecount; ++s) {
        //         apply_drive_outputs(ec_slave[s].outputs, &desired_outputs_shadow[s]);
        //     }
        //
        //     /* 2) Exchange process data once for the whole bus */
        //     ec_send_processdata();
        //     wkc = ec_receive_processdata(EC_TIMEOUTRET);
        //
        //     if (wkc >= expectedWKC)
        //     {
        //         /* 3) Per-slave print + read inputs */
        //         for (uint16_t s = 1; s <= ec_slavecount; ++s) {
        //             int oloop = ec_slave[s].Obytes;
        //             if ((oloop == 0) && (ec_slave[s].Obits > 0)) oloop = 1;
        //             if (oloop > 8) oloop = 8;
        //
        //             int iloop = ec_slave[s].Ibytes;
        //             if ((iloop == 0) && (ec_slave[s].Ibits > 0)) iloop = 1;
        //             if (iloop > 8) iloop = 8;
        //
        //             // print_process_data(i, wkc, expectedWKC, oloop, iloop, s);
        //             // read_slave_input_data(s, &slave_inputs[s]);
        //             print_all_slaves_process_data(i, wkc, expectedWKC);
        //             read_all_slaves_input_data(slave_inputs);
        //
        //         }
        //         needlf = TRUE;
        //     }
        //
        //     /* 4) Update each slave's command for next cycle based on its statusword */
        //     for (uint16_t s = 1; s <= ec_slavecount; ++s) {
        //         const uint16_t sw = slave_inputs[s].statusword;
        //
        //         if (sw == STATUSWORD_STATE_READYTOSWITCHON) {
        //             // printf("\n[Slave %u] ReadyToSwitchOn -> SwitchOn\n", s);
        //             desired_outputs_shadow[s].controlword = CONTROLWORD_SWITCH_ON;          // 0x07
        //
        //         } else if (sw == STATUSWORD_STATE_SWITCHEDON) {
        //             // printf("\n[Slave %u] SwitchedOn -> EnableOperation\n", s);
        //             desired_outputs_shadow[s].controlword = CONTROLWORD_ENABLE_OPERATION;   // 0x0F
        //
        //         } else if (sw == STATUSWORD_STATE_OPERATIONENABLED) {
        //             // printf("\n[Slave %u] OperationEnabled\n", s);
        //             desired_outputs_shadow[s].controlword = CONTROLWORD_ENABLE_OPERATION;   // keep EO
        //             // desired_outputs_shadow[s].mode_of_operation = MODE_CYCLIC_SYNC_TORQUE_MODE;
        //             // desired_outputs_shadow[s].target_torque = 0; // set your torque here per slave if needed
        //         } else {
        //             /* Optional fallback */
        //             // desired_outputs_shadow[s].controlword = CONTROLWORD_FAULTRESET;
        //         }
        //     }
        //
        //     osal_usleep(PRINT_PERIOD_US);
        // }

        // Initialize desired outputs (you can modify these at runtime)
        DriveOutputs desired_outputs = {
            .controlword                 = CONTROLWORD_FAULTRESET,
            .mode_of_operation           = 0,
            .target_position             = 0,
            .target_velocity             = 0,
            .target_torque               = 0,
            .impedance_proportional_gain = 0,
            .impedance_derivative_gain   = 0
        };

        ControlPhase control_phase = PHASE_SWITCH_ON;
        SlaveInputData slave_input_data;

		FILE *f = fopen("/tmp/input_data.txt", "w");

        // cyclic loop
        for (int i = 1; i <= 10000; i++)
        {


            // --- Apply outputs into PDO image ---
            apply_drive_outputs(ec_slave[SLAVE_IDX].outputs, &desired_outputs);

            ec_send_processdata();
            wkc = ec_receive_processdata(EC_TIMEOUTRET);
            if (wkc >= expectedWKC)
            {
                print_process_data(i, wkc, expectedWKC, oloop, iloop, SLAVE_IDX);
                read_slave_input_data(SLAVE_IDX, &slave_input_data);
                                        
				fprintf(f, "%u\n", slave_input_data.statusword); 
				fflush(f);
                
                needlf = TRUE;
            }



            if (slave_input_data.statusword == STATUSWORD_STATE_READYTOSWITCHON) {
                printf("\nphase switch ready for turn on, moving to switch on\n");
                desired_outputs.controlword = CONTROLWORD_SWITCH_ON;          // 0x07

            } else if (slave_input_data.statusword == STATUSWORD_STATE_SWITCHEDON) {
                printf("\nphase switched on, moving to enable operation mode\n");
                desired_outputs.controlword = CONTROLWORD_ENABLE_OPERATION;    // 0x0F

            } else if (slave_input_data.statusword == STATUSWORD_STATE_OPERATIONENABLED) {
                printf("\noperation mode enabled");
                desired_outputs.mode_of_operation = MODE_CYCLIC_SYNC_TORQUE_MODE;
                // desired_outputs.mode_of_operation = MODE_PROFILE_COMMUTATION;

                desired_outputs.controlword = CONTROLWORD_ENABLE_OPERATION;    // 0x0F
                desired_outputs.target_torque = 100;
            }



            osal_usleep(PRINT_PERIOD_US);
        }

		fclose(f);

        inOP = FALSE;
    }
    else
    {
        printf("Not all slaves reached operational state.\n");
        ec_readstate();
        for (int i = 1; i <= ec_slavecount; i++)
        {
            if (ec_slave[i].state != EC_STATE_OPERATIONAL)
            {
                printf("Slave %d State=0x%02x StatusCode=0x%04x : %s\n",
                       i, ec_slave[i].state, ec_slave[i].ALstatuscode,
                       ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
            }
        }
    }

    printf("\nRequest init state for all slaves\n");
    ec_slave[0].state = EC_STATE_INIT;
    ec_writestate(0);

    printf("End step-1 test, close socket\n");
    ec_close();
}

int main(int argc, char *argv[])
{
    printf("SOEM Step-1 (zeros to RxPDO, monitor)\n");

    if (argc > 1)
    {
        osal_thread_create(&thread1, 128000, &ecatcheck, NULL);
        simpletest(argv[1]);
    }
    else
    {
        ec_adaptert *adapter = NULL;
        printf("Usage: simple_test_step1_raidrive ifname1\nifname = eth0 for example\n");
        printf("\nAvailable adapters:\n");
        adapter = ec_find_adapters();
        while (adapter != NULL)
        {
            printf("    - %s  (%s)\n", adapter->name, adapter->desc);
            adapter = adapter->next;
        }
        ec_free_adapters(adapter);
    }

    printf("End program\n");
    return 0;
}
