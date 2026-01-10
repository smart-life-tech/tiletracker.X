/*
 * PIC16F15324 VL53L1X Advanced Implementation
 *
 * Based on the Arduino VL53L1X library (STSW-IMG007)
 * Enhanced I2C communication with support for:
 * - Multiple distance modes (Short, Medium, Long)
 * - Continuous and single-shot ranging
 * - Measurement timing budget configuration
 * - Complete sensor initialization
 *
 * Oscillator: 8 MHz Internal
 * I2C Clock: 100 kHz
 */

#include <xc.h>
#include <stdint.h>

// Oscillator configuration
#pragma config FEXTOSC = OFF        // Disable external oscillator
#pragma config MCLRE = ON           // MCLR pin enabled
#pragma config BOREN = ON           // Brown-out reset enabled
#pragma config WDTE = OFF           // Watchdog timer off
#pragma config PWRTE = ON           // Power-up timer enabled


// ============================================================================
// VL53L1X Register Map (from Arduino library reference)
// ============================================================================

#define IDENTIFICATION__MODEL_ID           0x010F
#define IDENTIFICATION__MODULE_TYPE        0x0110

#define SOFT_RESET                         0x0000

#define RESULT__RANGE_STATUS               0x0089
#define RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS 0x008C
#define RESULT__AMBIENT_COUNT_RATE_MCPS    0x0091
#define RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM 0x0096

#define MEASUREMENT_TIMING_BUDGET_CONFIG   0x002A

#define SYSTEM__MODE_START                 0x0001
#define SYSTEM__INTERRUPT_CLEAR            0x0086
#define SYSTEM__INTERMEASUREMENT_PERIOD    0x0004

#define RANGE_CONFIG__VCSEL_PERIOD_A       0x0033
#define RANGE_CONFIG__VCSEL_PERIOD_B       0x0036
#define RANGE_CONFIG__VALID_PHASE_HIGH     0x0033
#define RANGE_CONFIG__TIMEOUT_MACROP_A     0x005E
#define RANGE_CONFIG__TIMEOUT_MACROP_B     0x0061

#define SD_CONFIG__WOI_SD0                 0x002E
#define SD_CONFIG__WOI_SD1                 0x002F
#define SD_CONFIG__INITIAL_PHASE_SD0       0x0030
#define SD_CONFIG__INITIAL_PHASE_SD1       0x0031

#define VL53L1X_I2C_ADDR                   0x52

// ============================================================================
// Type Definitions
// ============================================================================

typedef enum {
    Short = 0,
    Medium = 1,
    Long = 2
} DistanceMode;

typedef enum {
    RangeValid = 0,
    SigmaFail = 1,
    SignalFail = 2,
    OutOfBoundsFail = 4,
    HardwareFail = 5,
    ProcessingFail = 6
} RangeStatus;

typedef enum {
    LED_PATTERN_OFF = 0,
    LED_PATTERN_SOLID_ON = 1,
    LED_PATTERN_SOLID_OFF = 2,
    LED_PATTERN_SLOW_BLINK = 3,      // 500ms on/off
    LED_PATTERN_FAST_BLINK = 4,      // 200ms on/off
    LED_PATTERN_DOUBLE_PULSE = 5,    // Two quick pulses
    LED_PATTERN_TRIPLE_PULSE = 6,    // Three quick pulses
    LED_PATTERN_SOS = 7               // SOS pattern (3-3-3)
} LEDPattern;

typedef struct {
    int range_mm;
    int range_status;
    int signal_count_rate_mcps;
    int ambient_count_rate_mcps;
} MeasurementData;

// ============================================================================
// Global Variables
// ============================================================================

volatile int i2c_error = 0;
static MeasurementData measurement;
static DistanceMode current_distance_mode = Long;
static unsigned int heartbeat_counter = 0;  // Counter for heartbeat LED (RA5)

// LED pattern tracking
static LEDPattern status_led_pattern = LED_PATTERN_SLOW_BLINK;  // RA4 status
static LEDPattern heartbeat_pattern = LED_PATTERN_SLOW_BLINK;   // RA5 heartbeat
static unsigned int led_pattern_counter = 0;
static unsigned char led_state = 0;

// ============================================================================
// Function Prototypes
// ============================================================================

void I2C_Init(void);
void GPIO_Init(void);
void Delay_ms(unsigned int ms);
unsigned char LED_Update_Pattern(LEDPattern pattern);
void LED_Set_Pattern(LEDPattern *current_pattern, LEDPattern new_pattern);

// I2C Core Functions
void I2C_Start(void);
void I2C_Stop(void);
void I2C_Send_Byte(unsigned char data);
unsigned char I2C_Receive_Byte(unsigned char ack);

// Register Access Functions
unsigned char I2C_Write_Register(int reg, unsigned char value);
unsigned char I2C_Read_Register(int reg, unsigned char *value);
unsigned char I2C_Write_Register16(int reg, int value);
unsigned char I2C_Read_Register16(int reg, int *value);
unsigned char I2C_Write_Register32(int reg, int value);
unsigned char I2C_Read_Register32(int reg, int *value);

// VL53L1X Functions
unsigned char VL53L1X_Init(void);
unsigned char VL53L1X_Soft_Reset(void);
unsigned char VL53L1X_Check_Model_ID(void);
unsigned char VL53L1X_Set_Distance_Mode(DistanceMode mode);
unsigned char VL53L1X_Start_Single_Shot(void);
unsigned char VL53L1X_Start_Continuous(int period_ms);
unsigned char VL53L1X_Stop_Continuous(void);
unsigned char VL53L1X_Is_Data_Ready(void);
unsigned char VL53L1X_Read_Measurement(MeasurementData *data);

// ============================================================================
// LED Pattern Management Functions
// ============================================================================

void LED_Set_Pattern(LEDPattern *current_pattern, LEDPattern new_pattern)
{
    if(*current_pattern != new_pattern)
    {
        *current_pattern = new_pattern;
        led_pattern_counter = 0;  // Reset counter when pattern changes
    }
}

unsigned char LED_Update_Pattern(LEDPattern pattern)
{
    unsigned char led_state = 0;
    
    switch(pattern)
    {
        case LED_PATTERN_OFF:
            led_state = 0;
            break;
            
        case LED_PATTERN_SOLID_ON:
            led_state = 1;
            break;
            
        case LED_PATTERN_SOLID_OFF:
            led_state = 0;
            break;
            
        case LED_PATTERN_SLOW_BLINK:
            // 500ms on, 500ms off (10 ticks of 50ms each)
            if(led_pattern_counter < 10)
                led_state = 1;
            else if(led_pattern_counter < 20)
                led_state = 0;
            else
                led_pattern_counter = 0;
            break;
            
        case LED_PATTERN_FAST_BLINK:
            // 200ms on, 200ms off (4 ticks of 50ms each)
            if(led_pattern_counter < 4)
                led_state = 1;
            else if(led_pattern_counter < 8)
                led_state = 0;
            else
                led_pattern_counter = 0;
            break;
            
        case LED_PATTERN_DOUBLE_PULSE:
            // Two quick 100ms pulses with 100ms gaps
            // Pattern: on(2) off(2) on(2) off(6) = 12 ticks
            if(led_pattern_counter < 2)
                led_state = 1;
            else if(led_pattern_counter < 4)
                led_state = 0;
            else if(led_pattern_counter < 6)
                led_state = 1;
            else if(led_pattern_counter < 12)
                led_state = 0;
            else
                led_pattern_counter = 0;
            break;
            
        case LED_PATTERN_TRIPLE_PULSE:
            // Three quick 100ms pulses with 100ms gaps
            // Pattern: on(2) off(2) on(2) off(2) on(2) off(6) = 16 ticks
            if(led_pattern_counter < 2)
                led_state = 1;
            else if(led_pattern_counter < 4)
                led_state = 0;
            else if(led_pattern_counter < 6)
                led_state = 1;
            else if(led_pattern_counter < 8)
                led_state = 0;
            else if(led_pattern_counter < 10)
                led_state = 1;
            else if(led_pattern_counter < 16)
                led_state = 0;
            else
                led_pattern_counter = 0;
            break;
            
        case LED_PATTERN_SOS:
            // SOS pattern: dot(1) dot(1) dot(1) gap(3) dash(3) dash(3) dash(3) gap(3) dot(1) dot(1) dot(1)
            // Morse: S=3 dots, O=3 dashes, S=3 dots
            // Simplified: on(1) off(1) on(1) off(1) on(1) off(3) on(3) off(1) on(3) off(1) on(3) off(3)
            if(led_pattern_counter < 1)
                led_state = 1;
            else if(led_pattern_counter < 2)
                led_state = 0;
            else if(led_pattern_counter < 3)
                led_state = 1;
            else if(led_pattern_counter < 4)
                led_state = 0;
            else if(led_pattern_counter < 5)
                led_state = 1;
            else if(led_pattern_counter < 8)
                led_state = 0;
            else if(led_pattern_counter < 11)
                led_state = 1;
            else if(led_pattern_counter < 12)
                led_state = 0;
            else if(led_pattern_counter < 15)
                led_state = 1;
            else if(led_pattern_counter < 16)
                led_state = 0;
            else if(led_pattern_counter < 19)
                led_state = 1;
            else if(led_pattern_counter < 22)
                led_state = 0;
            else
                led_pattern_counter = 0;
            break;
            
        default:
            led_state = 0;
            break;
    }
    
    led_pattern_counter++;
    return led_state;
}

// ============================================================================
// Main Program
// ============================================================================

void main(void)
{
    GPIO_Init();
    I2C_Init();
    Delay_ms(100);

    // Initialization sequence - triple blink on status LED
    for(int i = 0; i < 3; i++)
    {
        RA4 = 1;
        Delay_ms(100);
        RA4 = 0;
        Delay_ms(100);
    }

    // Initialize sensor
    if(!VL53L1X_Init())
    {
        // Initialization failed - SOS pattern on status LED
        LED_Set_Pattern(&status_led_pattern, LED_PATTERN_SOS);
        while(1)
        {
            RA4 = LED_Update_Pattern(status_led_pattern);
            Delay_ms(50);
        }
    }

    // Sensor initialized successfully - double pulse pattern briefly
    for(int i = 0; i < 2; i++)
    {
        RA4 = 1;
        Delay_ms(150);
        RA4 = 0;
        Delay_ms(150);
    }

    // Start continuous ranging with 100ms interval
    VL53L1X_Start_Continuous(100);
    Delay_ms(100);

    // Initialize LED patterns - heartbeat uses slow blink
    LED_Set_Pattern(&heartbeat_pattern, LED_PATTERN_SLOW_BLINK);

    // Main loop - read distance and control LED
    while(1)
    {
        if(VL53L1X_Is_Data_Ready())
        {
            if(VL53L1X_Read_Measurement(&measurement))
            {
                // Check measurement status and set LED pattern accordingly
                switch(measurement.range_status)
                {
                    case RangeValid:
                        // Valid measurement - LED pattern depends on distance
                        if(measurement.range_mm < 150)
                        {
                            // Very close - solid on (red zone)
                            LED_Set_Pattern(&status_led_pattern, LED_PATTERN_SOLID_ON);
                        }
                        else if(measurement.range_mm < 300)
                        {
                            // Close - fast blink (yellow zone)
                            LED_Set_Pattern(&status_led_pattern, LED_PATTERN_FAST_BLINK);
                        }
                        else if(measurement.range_mm < 500)
                        {
                            // Medium distance - slow blink (green zone)
                            LED_Set_Pattern(&status_led_pattern, LED_PATTERN_SLOW_BLINK);
                        }
                        else
                        {
                            // Far away - off (no detection)
                            LED_Set_Pattern(&status_led_pattern, LED_PATTERN_OFF);
                        }
                        break;
                        
                    case SigmaFail:
                        // Quality/Sigma error - double pulse pattern
                        LED_Set_Pattern(&status_led_pattern, LED_PATTERN_DOUBLE_PULSE);
                        break;
                        
                    case SignalFail:
                        // Signal too weak - triple pulse pattern
                        LED_Set_Pattern(&status_led_pattern, LED_PATTERN_TRIPLE_PULSE);
                        break;
                        
                    case OutOfBoundsFail:
                        // Out of measurement range - fast blink
                        LED_Set_Pattern(&status_led_pattern, LED_PATTERN_FAST_BLINK);
                        break;
                        
                    case HardwareFail:
                    case ProcessingFail:
                    default:
                        // Error condition - SOS pattern
                        LED_Set_Pattern(&status_led_pattern, LED_PATTERN_SOS);
                        break;
                }
            }
            else
            {
                // I2C read failed - error pattern
                LED_Set_Pattern(&status_led_pattern, LED_PATTERN_TRIPLE_PULSE);
            }
        }
        else
        {
            // No data ready - slow pulse to show waiting
            LED_Set_Pattern(&status_led_pattern, LED_PATTERN_DOUBLE_PULSE);
        }

        // Update status LED (RA4) with current pattern
        RA4 = LED_Update_Pattern(status_led_pattern);
        
        // Update heartbeat LED (RA5) with heartbeat pattern
        RA5 = LED_Update_Pattern(heartbeat_pattern);

        Delay_ms(50);
    }
}

// ============================================================================
// GPIO Initialization
// ============================================================================

void GPIO_Init(void)
{
    // Configure Port A - LED on RA4 (proximity), RA5 (heartbeat)
    ANSELA = 0x00;           // All digital I/O
    TRISA = 0b00010011;      // RA4 and RA5 as outputs
    PORTA = 0x00;

    // Configure Port C - I2C on RC0 (SCL), RC1 (SDA)
    ANSELC = 0x00;           // All digital I/O
    TRISC = 0b11111100;      // RC0, RC1 as open-drain inputs
    PORTC = 0x00;
}

// ============================================================================
// I2C Initialization
// ============================================================================

void I2C_Init(void)
{
    // Baud rate: 100 kHz @ 8MHz
    // BAUD = (Fosc / (4 * Speed)) - 1 = (8MHz / 400kHz) - 1 = 19
    SSP1ADD = 19;

    // I2C Master Mode
    SSP1CON1 = 0b00101000;
    SSP1CON2 = 0x00;
    SSP1CON3 = 0x00;
    SSP1STATbits.SMP = 1;    // Slew rate disabled
}

// ============================================================================
// I2C Core Functions
// ============================================================================

void I2C_Start(void)
{
    SSP1CON2bits.SEN = 1;
    while(SSP1CON2bits.SEN);
}

void I2C_Stop(void)
{
    SSP1CON2bits.PEN = 1;
    while(SSP1CON2bits.PEN);
}

void I2C_Send_Byte(unsigned char data)
{
    SSP1BUF = data;
    while(SSP1STATbits.BF);
    while(SSP1CON2bits.ACKEN);
}

unsigned char I2C_Receive_Byte(unsigned char ack)
{
    unsigned char data;

    SSP1CON2bits.RCEN = 1;
    while(SSP1STATbits.BF == 0);
    data = SSP1BUF;

    SSP1CON2bits.ACKDT = (ack ? 0 : 1);
    SSP1CON2bits.ACKEN = 1;
    while(SSP1CON2bits.ACKEN);

    return data;
}

// ============================================================================
// Register Access - 8-bit
// ============================================================================

unsigned char I2C_Write_Register(int reg, unsigned char value)
{
    unsigned char reg_high = (unsigned char)((reg >> 8) & 0xFF);
    unsigned char reg_low = (unsigned char)(reg & 0xFF);

    I2C_Start();
    I2C_Send_Byte(VL53L1X_I2C_ADDR);  // Write address

    if(SSP1CON2bits.ACKSTAT)
    {
        I2C_Stop();
        return 0;
    }

    I2C_Send_Byte(reg_high);
    if(SSP1CON2bits.ACKSTAT) { I2C_Stop(); return 0; }

    I2C_Send_Byte(reg_low);
    if(SSP1CON2bits.ACKSTAT) { I2C_Stop(); return 0; }

    I2C_Send_Byte(value);
    if(SSP1CON2bits.ACKSTAT) { I2C_Stop(); return 0; }

    I2C_Stop();
    return 1;
}

unsigned char I2C_Read_Register(int reg, unsigned char *value)
{
    unsigned char reg_high = (unsigned char)((reg >> 8) & 0xFF);
    unsigned char reg_low = (unsigned char)(reg & 0xFF);

    // Set address
    I2C_Start();
    I2C_Send_Byte(VL53L1X_I2C_ADDR);  // Write address
    if(SSP1CON2bits.ACKSTAT) { I2C_Stop(); return 0; }

    I2C_Send_Byte(reg_high);
    if(SSP1CON2bits.ACKSTAT) { I2C_Stop(); return 0; }

    I2C_Send_Byte(reg_low);
    if(SSP1CON2bits.ACKSTAT) { I2C_Stop(); return 0; }

    // Repeated START and read
    I2C_Start();
    I2C_Send_Byte(VL53L1X_I2C_ADDR | 0x01);  // Read address
    if(SSP1CON2bits.ACKSTAT) { I2C_Stop(); return 0; }

    *value = I2C_Receive_Byte(0);  // NACK for last byte

    I2C_Stop();
    return 1;
}

// ============================================================================
// Register Access - 16-bit
// ============================================================================

unsigned char I2C_Write_Register16(int reg, int value)
{
    unsigned char reg_high = (unsigned char)((reg >> 8) & 0xFF);
    unsigned char reg_low = (unsigned char)(reg & 0xFF);
    unsigned char val_high = (unsigned char)((value >> 8) & 0xFF);
    unsigned char val_low = (unsigned char)(value & 0xFF);

    I2C_Start();
    I2C_Send_Byte(VL53L1X_I2C_ADDR);
    if(SSP1CON2bits.ACKSTAT) { I2C_Stop(); return 0; }

    I2C_Send_Byte(reg_high);
    if(SSP1CON2bits.ACKSTAT) { I2C_Stop(); return 0; }

    I2C_Send_Byte(reg_low);
    if(SSP1CON2bits.ACKSTAT) { I2C_Stop(); return 0; }

    I2C_Send_Byte(val_high);
    if(SSP1CON2bits.ACKSTAT) { I2C_Stop(); return 0; }

    I2C_Send_Byte(val_low);
    if(SSP1CON2bits.ACKSTAT) { I2C_Stop(); return 0; }

    I2C_Stop();
    return 1;
}

unsigned char I2C_Read_Register16(int reg, int *value)
{
    unsigned char reg_high = (unsigned char)((reg >> 8) & 0xFF);
    unsigned char reg_low = (unsigned char)(reg & 0xFF);
    unsigned char val_high, val_low;

    I2C_Start();
    I2C_Send_Byte(VL53L1X_I2C_ADDR);
    if(SSP1CON2bits.ACKSTAT) { I2C_Stop(); return 0; }

    I2C_Send_Byte(reg_high);
    if(SSP1CON2bits.ACKSTAT) { I2C_Stop(); return 0; }

    I2C_Send_Byte(reg_low);
    if(SSP1CON2bits.ACKSTAT) { I2C_Stop(); return 0; }

    I2C_Start();
    I2C_Send_Byte(VL53L1X_I2C_ADDR | 0x01);
    if(SSP1CON2bits.ACKSTAT) { I2C_Stop(); return 0; }

    val_high = I2C_Receive_Byte(1);  // ACK for high byte
    val_low = I2C_Receive_Byte(0);   // NACK for low byte

    *value = (val_high << 8) | val_low;

    I2C_Stop();
    return 1;
}

// ============================================================================
// Register Access - 32-bit
// ============================================================================

unsigned char I2C_Write_Register32(int reg, int value)
{
    unsigned char reg_high = (unsigned char)((reg >> 8) & 0xFF);
    unsigned char reg_low = (unsigned char)(reg & 0xFF);

    I2C_Start();
    I2C_Send_Byte(VL53L1X_I2C_ADDR);
    if(SSP1CON2bits.ACKSTAT) { I2C_Stop(); return 0; }

    I2C_Send_Byte(reg_high);
    if(SSP1CON2bits.ACKSTAT) { I2C_Stop(); return 0; }

    I2C_Send_Byte(reg_low);
    if(SSP1CON2bits.ACKSTAT) { I2C_Stop(); return 0; }

    // Send 4 bytes (MSB first)
    I2C_Send_Byte((unsigned char)((value >> 24) & 0xFF));
    if(SSP1CON2bits.ACKSTAT) { I2C_Stop(); return 0; }

    I2C_Send_Byte((unsigned char)((value >> 16) & 0xFF));
    if(SSP1CON2bits.ACKSTAT) { I2C_Stop(); return 0; }

    I2C_Send_Byte((unsigned char)((value >> 8) & 0xFF));
    if(SSP1CON2bits.ACKSTAT) { I2C_Stop(); return 0; }

    I2C_Send_Byte((unsigned char)(value & 0xFF));
    if(SSP1CON2bits.ACKSTAT) { I2C_Stop(); return 0; }

    I2C_Stop();
    return 1;
}

unsigned char I2C_Read_Register32(int reg, int *value)
{
    unsigned char reg_high = (unsigned char)((reg >> 8) & 0xFF);
    unsigned char reg_low = (unsigned char)(reg & 0xFF);
    unsigned char byte0, byte1, byte2, byte3;

    I2C_Start();
    I2C_Send_Byte(VL53L1X_I2C_ADDR);
    if(SSP1CON2bits.ACKSTAT) { I2C_Stop(); return 0; }

    I2C_Send_Byte(reg_high);
    if(SSP1CON2bits.ACKSTAT) { I2C_Stop(); return 0; }

    I2C_Send_Byte(reg_low);
    if(SSP1CON2bits.ACKSTAT) { I2C_Stop(); return 0; }

    I2C_Start();
    I2C_Send_Byte(VL53L1X_I2C_ADDR | 0x01);
    if(SSP1CON2bits.ACKSTAT) { I2C_Stop(); return 0; }

    // Read 4 bytes (MSB first)
    byte0 = I2C_Receive_Byte(1);  // ACK
    byte1 = I2C_Receive_Byte(1);  // ACK
    byte2 = I2C_Receive_Byte(1);  // ACK
    byte3 = I2C_Receive_Byte(0);  // NACK (last byte)

    *value = ((int)byte0 << 24) | ((int)byte1 << 16) |
             ((int)byte2 << 8) | byte3;

    I2C_Stop();
    return 1;
}

// ============================================================================
// VL53L1X Sensor Functions
// ============================================================================

unsigned char VL53L1X_Soft_Reset(void)
{
    // Write 0x00 to SOFT_RESET
    if(!I2C_Write_Register(SOFT_RESET, 0x00)) return 0;
    Delay_ms(1);

    // Write 0x01 to SOFT_RESET
    if(!I2C_Write_Register(SOFT_RESET, 0x01)) return 0;
    Delay_ms(1);

    return 1;
}

unsigned char VL53L1X_Check_Model_ID(void)
{
    int model_id;

    if(!I2C_Read_Register16(IDENTIFICATION__MODEL_ID, &model_id)) return 0;

    // Model ID should be 0xEACC
    if(model_id != 0xEACC) return 0;

    return 1;
}

unsigned char VL53L1X_Init(void)
{
    // Software reset
    if(!VL53L1X_Soft_Reset()) return 0;
    Delay_ms(10);

    // Check model ID
    if(!VL53L1X_Check_Model_ID()) return 0;

    // Set default distance mode to Long
    if(!VL53L1X_Set_Distance_Mode(Long)) return 0;

    return 1;
}

unsigned char VL53L1X_Set_Distance_Mode(DistanceMode mode)
{
    unsigned char vcsel_period_a, vcsel_period_b;

    switch(mode)
    {
        case Short:
            vcsel_period_a = 0x07;
            vcsel_period_b = 0x05;
            break;

        case Medium:
            vcsel_period_a = 0x0B;
            vcsel_period_b = 0x09;
            break;

        case Long:
        default:
            vcsel_period_a = 0x0F;
            vcsel_period_b = 0x0D;
            break;
    }

    // Set VCSEL periods
    if(!I2C_Write_Register(RANGE_CONFIG__VCSEL_PERIOD_A, vcsel_period_a)) return 0;
    if(!I2C_Write_Register(RANGE_CONFIG__VCSEL_PERIOD_B, vcsel_period_b)) return 0;

    current_distance_mode = mode;
    return 1;
}

unsigned char VL53L1X_Start_Single_Shot(void)
{
    // Clear interrupt
    if(!I2C_Write_Register(SYSTEM__INTERRUPT_CLEAR, 0x01)) return 0;

    // Start single shot measurement (mode 0x10)
    if(!I2C_Write_Register(SYSTEM__MODE_START, 0x10)) return 0;

    return 1;
}

unsigned char VL53L1X_Start_Continuous(int period_ms)
{
    // Set inter-measurement period
    if(!I2C_Write_Register32(SYSTEM__INTERMEASUREMENT_PERIOD, period_ms * 1000)) return 0;

    // Clear interrupt
    if(!I2C_Write_Register(SYSTEM__INTERRUPT_CLEAR, 0x01)) return 0;

    // Start continuous ranging (mode 0x40 = timed)
    if(!I2C_Write_Register(SYSTEM__MODE_START, 0x40)) return 0;

    return 1;
}

unsigned char VL53L1X_Stop_Continuous(void)
{
    // Abort ranging (mode 0x80)
    if(!I2C_Write_Register(SYSTEM__MODE_START, 0x80)) return 0;

    Delay_ms(10);
    return 1;
}

unsigned char VL53L1X_Is_Data_Ready(void)
{
    unsigned char status;

    if(!I2C_Read_Register(RESULT__RANGE_STATUS, &status)) return 0;

    // Check if data ready (bit 0)
    return (status & 0x01) ? 0 : 1;  // Inverted logic
}

unsigned char VL53L1X_Read_Measurement(MeasurementData *data)
{
    unsigned char status;
    int range_mm;
    int signal_rate, ambient_rate;

    if(!I2C_Read_Register(RESULT__RANGE_STATUS, &status)) return 0;
    if(!I2C_Read_Register16(RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM, &range_mm)) return 0;
    if(!I2C_Read_Register16(RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS, &signal_rate)) return 0;
    if(!I2C_Read_Register16(RESULT__AMBIENT_COUNT_RATE_MCPS, &ambient_rate)) return 0;

    // Parse status
    unsigned char range_status = (status >> 4) & 0x0F;

    data->range_mm = range_mm;
    data->signal_count_rate_mcps = signal_rate;
    data->ambient_count_rate_mcps = ambient_rate;

    // Map status to RangeStatus enum
    switch(range_status)
    {
        case 0:
            data->range_status = RangeValid;
            break;
        case 1:
            data->range_status = SigmaFail;
            break;
        case 2:
            data->range_status = SignalFail;
            break;
        case 5:
            data->range_status = OutOfBoundsFail;
            break;
        default:
            data->range_status = ProcessingFail;
    }

    // Clear interrupt
    I2C_Write_Register(SYSTEM__INTERRUPT_CLEAR, 0x01);

    return 1;
}

// ============================================================================
// Utility Functions
// ============================================================================

void Delay_ms(unsigned int ms)
{
    unsigned int i, j;
    for(i = 0; i < ms; i++)
        for(j = 0; j < 124; j++);  // Calibrated for 8MHz
}

/*
 * Hardware Connections:
 * VL53L1X VCC  -> VDD (Pin 15)
 * VL53L1X GND  -> VSS (Pin 8/16)
 * VL53L1X SCL  -> RC0 (Pin 7) with 4.7kO pull-up
 * VL53L1X SDA  -> RC1 (Pin 6) with 4.7kO pull-up
 * Proximity LED -> 220O resistor -> RA4 (Pin 3)
 * Heartbeat LED -> 220O resistor -> RA5 (Pin 2)
 * LED Cathode  -> VSS (Ground)
 */
