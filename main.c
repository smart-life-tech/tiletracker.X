/*
 * PIC16F15324 VL53L1X Advanced Implementation
 *
 * Based on the  VL53L1X library (STSW-IMG007)
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
// VL53L1X Register Map (from  library reference)
// ============================================================================

#define IDENTIFICATION__MODEL_ID           0x010F
#define IDENTIFICATION__MODULE_TYPE        0x0110

#define SOFT_RESET                         0x0000
#define SYSTEM__BOOT_STATE                 0x0006  // Critical: check firmware loaded (bit 0)
#define RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS 0x008C
#define RESULT__AMBIENT_COUNT_RATE_MCPS    0x0091
#define RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM 0x0096
#define RESULT__RANGE_STATUS               0x0089

#define MEASUREMENT_TIMING_BUDGET_CONFIG   0x002A

#define SYSTEM__MODE_START                 0x0001
#define SYSTEM__INTERRUPT_CLEAR            0x0086
#define SYSTEM__INTERMEASUREMENT_PERIOD    0x0004

#define RANGE_CONFIG__VCSEL_PERIOD_A       0x0033
#define RANGE_CONFIG__VCSEL_PERIOD_B       0x0036
#define RANGE_CONFIG__VALID_PHASE_HIGH     0x0033
#define RANGE_CONFIG__TIMEOUT_MACROP_A     0x005E
#define RANGE_CONFIG__TIMEOUT_MACROP_B     0x0061

#define PHASECAL_CONFIG__TIMEOUT_MACROP    0x004B

#define SD_CONFIG__WOI_SD0                 0x002E
#define SD_CONFIG__WOI_SD1                 0x002F
#define SD_CONFIG__INITIAL_PHASE_SD0       0x0030
#define SD_CONFIG__INITIAL_PHASE_SD1       0x0031

// Distance Mode Timeout Values
#define DISTANCE_MODE_SHORT_TIMEOUT        0x14
#define DISTANCE_MODE_MEDIUM_TIMEOUT       0x0B
#define DISTANCE_MODE_LONG_TIMEOUT         0x0A

#define VL53L1X_I2C_ADDR                   0x52

// VL53L1X Default Configuration Array (required for proper initialization)
// This is the essential configuration block from ST's API
const unsigned char VL53L1X_DEFAULT_CONFIG[] = {
    0x00, 0x00, 0x00, 0x01, 0x02, 0x00, 0x02, 0x08, 0x00, 0x08, 0x10, 0x01,
    0x01, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x20, 0x0B, 0x00, 0x00, 0x02, 0x0A, 0x21, 0x00, 0x00, 0x05, 0x00,
    0x00, 0x00, 0x00, 0xC8, 0x00, 0x00, 0x38, 0xFF, 0x01, 0x00, 0x08, 0x00,
    0x00, 0x01, 0xCC, 0x0F, 0x01, 0xF1, 0x0D, 0x01, 0x68, 0x00, 0x80, 0x08,
    0xB8, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x89, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x01, 0x0F, 0x0D, 0x0E, 0x0E, 0x00, 0x00, 0x02, 0xC7, 0xFF,
    0x9B, 0x00, 0x00, 0x00, 0x01, 0x01, 0x40
};

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
unsigned char VL53L1X_Set_Measurement_Timing_Budget(unsigned int budget_us);
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
        RA4 = (unsigned char)1;
        Delay_ms(100);
        RA4 = (unsigned char)0;
        Delay_ms(100);
    }

    // Initialize sensor
    if(!VL53L1X_Init())
    {
        // Initialization failed - SOS pattern on status LED
        LED_Set_Pattern(&status_led_pattern, LED_PATTERN_SOS);
        while(1)
        {
            RA4 = LED_Update_Pattern(status_led_pattern) ? 1 : 0;
            Delay_ms(50);
        }
    }

    // Sensor initialized successfully - double pulse pattern briefly
    for(int i = 0; i < 2; i++)
    {
        RA4 = (unsigned char)1;
        Delay_ms(150);
        RA4 = (unsigned char)0;
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
        RA4 = LED_Update_Pattern(status_led_pattern) ? 1 : 0;
        
        // Update heartbeat LED (RA5) with heartbeat pattern
        RA5 = LED_Update_Pattern(heartbeat_pattern) ? 1 : 0;

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
    TRISA = 0b11001111;      // RA4 and RA5 as outputs (0 = output)
    PORTA = 0x00;

    // Configure Port C - I2C on RC0 (SCL), RC1 (SDA)
    ANSELC = 0x00;           // All digital I/O
    TRISC = 0b11111111;      // RC0, RC1 as inputs initially (I2C will control)
    PORTC = 0x00;
    
    // Configure PPS (Peripheral Pin Select) for I2C
    // Unlock PPS
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 0;
    
    // Map I2C to RC0 (SCL) and RC1 (SDA)
    SSP1CLKPPS = 0x10;       // RC0 -> SCL input (0x10 = RC0)
    SSP1DATPPS = 0x11;       // RC1 -> SDA input (0x11 = RC1)
    RC0PPS = 0x15;           // SCL output -> RC0 (0x15 = SCL)
    RC1PPS = 0x16;           // SDA output -> RC1 (0x16 = SDA)
    
    // Lock PPS
    PPSLOCK = 0x55;
    PPSLOCK = 0xAA;
    PPSLOCKbits.PPSLOCKED = 1;
    
    // Set RC0 and RC1 as open-drain
    ODCONC = 0b00000011;     // RC0, RC1 open-drain enabled
}

// ============================================================================
// I2C Initialization
// ============================================================================

void I2C_Init(void)
{
    // Disable I2C before configuration
    SSP1CON1 = 0x00;
    
    // Baud rate: 100 kHz @ 8MHz
    // BAUD = (Fosc / (4 * Speed)) - 1 = (8MHz / (4 * 100kHz)) - 1 = 19
    SSP1ADD = 19;

    // I2C Master Mode, enable MSSP
    SSP1CON1bits.SSPM = 0b1000;  // I2C Master mode, clock = FOSC/(4 * (SSP1ADD+1))
    SSP1CON1bits.SSPEN = 1;      // Enable MSSP module
    
    SSP1CON2 = 0x00;
    SSP1CON3 = 0x00;
    SSP1STATbits.SMP = 1;        // Slew rate control disabled for 100kHz
    SSP1STATbits.CKE = 0;        // Disable SMBus specific inputs
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
    while(SSP1STATbits.BF);  // Wait for transfer complete
    
    // Check for NACK - if received, set error flag
    if(SSP1CON2bits.ACKSTAT)
    {
        i2c_error = 1;  // NACK received
    }
    while(SSP1CON2bits.ACKEN);  // Wait for ACK to complete
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

    i2c_error = 0;
    I2C_Start();
    I2C_Send_Byte(VL53L1X_I2C_ADDR);  // Write address
    if(i2c_error) { I2C_Stop(); return 0; }

    I2C_Send_Byte(reg_high);
    if(i2c_error) { I2C_Stop(); return 0; }

    I2C_Send_Byte(reg_low);
    if(i2c_error) { I2C_Stop(); return 0; }

    I2C_Send_Byte(value);
    if(i2c_error) { I2C_Stop(); return 0; }

    I2C_Stop();
    return 1;
}

unsigned char I2C_Read_Register(int reg, unsigned char *value)
{
    unsigned char reg_high = (unsigned char)((reg >> 8) & 0xFF);
    unsigned char reg_low = (unsigned char)(reg & 0xFF);

    // Set address phase
    i2c_error = 0;
    I2C_Start();
    I2C_Send_Byte(VL53L1X_I2C_ADDR);  // Write address
    if(i2c_error) { I2C_Stop(); return 0; }

    I2C_Send_Byte(reg_high);
    if(i2c_error) { I2C_Stop(); return 0; }

    I2C_Send_Byte(reg_low);
    if(i2c_error) { I2C_Stop(); return 0; }

    // Repeated START and read
    I2C_Start();
    I2C_Send_Byte(VL53L1X_I2C_ADDR | 0x01);  // Read address
    if(i2c_error) { I2C_Stop(); return 0; }

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
    i2c_error = 0;
    I2C_Send_Byte(VL53L1X_I2C_ADDR);
    if(i2c_error) { I2C_Stop(); return 0; }

    I2C_Send_Byte(reg_high);
    if(i2c_error) { I2C_Stop(); return 0; }

    I2C_Send_Byte(reg_low);
    if(i2c_error) { I2C_Stop(); return 0; }

    I2C_Send_Byte(val_high);
    if(i2c_error) { I2C_Stop(); return 0; }

    I2C_Send_Byte(val_low);
    if(i2c_error) { I2C_Stop(); return 0; }

    I2C_Stop();
    return 1;
}

unsigned char I2C_Read_Register16(int reg, int *value)
{
    unsigned char reg_high = (unsigned char)((reg >> 8) & 0xFF);
    unsigned char reg_low = (unsigned char)(reg & 0xFF);
    unsigned char val_high, val_low;

    // ADDRESS PHASE - set register address for read
    i2c_error = 0;
    I2C_Start();
    I2C_Send_Byte(VL53L1X_I2C_ADDR);  // Write address
    if(i2c_error) { I2C_Stop(); return 0; }

    I2C_Send_Byte(reg_high);
    if(i2c_error) { I2C_Stop(); return 0; }

    I2C_Send_Byte(reg_low);
    if(i2c_error) { I2C_Stop(); return 0; }

    // REPEATED START - critical for VL53L1X compliance
    I2C_Start();
    I2C_Send_Byte(VL53L1X_I2C_ADDR | 0x01);  // Read address
    if(i2c_error) { I2C_Stop(); return 0; }

    // READ PHASE - read two bytes
    val_high = I2C_Receive_Byte(1);  // ACK for high byte (more data coming)
    val_low = I2C_Receive_Byte(0);   // NACK for low byte (last byte)

    I2C_Stop();
    
    // Combine bytes to 16-bit signed integer
    *value = (int)(((unsigned int)val_high << 8) | (unsigned int)val_low);
    
    return 1;
}

// ============================================================================
// Register Access - 32-bit
// ============================================================================

unsigned char I2C_Write_Register32(int reg, int value)
{
    unsigned char reg_high = (unsigned char)((reg >> 8) & 0xFF);
    unsigned char reg_low = (unsigned char)(reg & 0xFF);
    unsigned long uvalue = (unsigned long)value;  // Cast to unsigned long once

    i2c_error = 0;
    I2C_Start();
    I2C_Send_Byte(VL53L1X_I2C_ADDR);
    if(i2c_error) { I2C_Stop(); return 0; }

    I2C_Send_Byte(reg_high);
    if(i2c_error) { I2C_Stop(); return 0; }

    I2C_Send_Byte(reg_low);
    if(i2c_error) { I2C_Stop(); return 0; }

    // Send 4 bytes (MSB first)
    I2C_Send_Byte((unsigned char)((uvalue >> 24) & 0xFF));
    if(i2c_error) { I2C_Stop(); return 0; }

    I2C_Send_Byte((unsigned char)((uvalue >> 16) & 0xFF));
    if(i2c_error) { I2C_Stop(); return 0; }

    I2C_Send_Byte((unsigned char)((uvalue >> 8) & 0xFF));
    if(i2c_error) { I2C_Stop(); return 0; }

    I2C_Send_Byte((unsigned char)(uvalue & 0xFF));
    if(i2c_error) { I2C_Stop(); return 0; }

    I2C_Stop();
    return 1;
}

unsigned char I2C_Read_Register32(int reg, int *value)
{
    unsigned char reg_high = (unsigned char)((reg >> 8) & 0xFF);
    unsigned char reg_low = (unsigned char)(reg & 0xFF);
    unsigned char byte0, byte1, byte2, byte3;

    i2c_error = 0;
    I2C_Start();
    I2C_Send_Byte(VL53L1X_I2C_ADDR);
    if(i2c_error) { I2C_Stop(); return 0; }

    I2C_Send_Byte(reg_high);
    if(i2c_error) { I2C_Stop(); return 0; }

    I2C_Send_Byte(reg_low);
    if(i2c_error) { I2C_Stop(); return 0; }

    // Repeated START for read
    I2C_Start();
    I2C_Send_Byte(VL53L1X_I2C_ADDR | 0x01);
    if(i2c_error) { I2C_Stop(); return 0; }

    // Read 4 bytes (MSB first)
    byte0 = I2C_Receive_Byte(1);  // ACK
    byte1 = I2C_Receive_Byte(1);  // ACK
    byte2 = I2C_Receive_Byte(1);  // ACK
    byte3 = I2C_Receive_Byte(0);  // NACK (last byte)

    *value = (int)(((unsigned long)byte0 << 24) | ((unsigned long)byte1 << 16) |
             ((unsigned long)byte2 << 8) | (unsigned long)byte3);

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
    unsigned char boot_state;
    unsigned int timeout;
    
    // Delay after power-up (critical - at least 20ms required per ST)
    Delay_ms(20);
    
    // Software reset
    if(!VL53L1X_Soft_Reset()) return 0;
    Delay_ms(50);

    // CRITICAL: Wait for firmware to boot (SYSTEM__BOOT_STATE register bit 0)
    // The VL53L1X will NOT respond properly until firmware is booted
    timeout = 5000;
    while(timeout > 0)
    {
        i2c_error = 0;
        if(!I2C_Read_Register(SYSTEM__BOOT_STATE, &boot_state)) 
        {
            Delay_ms(1);
            timeout--;
            continue;
        }
        
        if(boot_state & 0x01) break;  // Bit 0 = 1 means firmware is booted
        Delay_ms(1);
        timeout--;
    }
    if(timeout == 0) return 0;  // Boot timeout - firmware did not load
    
    Delay_ms(10);
    
    // Check model ID (now that firmware is loaded)
    if(!VL53L1X_Check_Model_ID()) return 0;
    
    // Load default configuration block (90 bytes from ST STSW-IMG007)
    // This is absolutely required for VL53L1X to function
    I2C_Start();
    i2c_error = 0;
    I2C_Send_Byte(VL53L1X_I2C_ADDR);
    if(i2c_error) { I2C_Stop(); return 0; }
    
    // Write start address 0x002D
    I2C_Send_Byte(0x00);
    if(i2c_error) { I2C_Stop(); return 0; }
    I2C_Send_Byte(0x2D);
    if(i2c_error) { I2C_Stop(); return 0; }
    
    // Write configuration block (90 bytes)
    for(unsigned char i = 0; i < sizeof(VL53L1X_DEFAULT_CONFIG); i++)
    {
        I2C_Send_Byte(VL53L1X_DEFAULT_CONFIG[i]);
        if(i2c_error) { I2C_Stop(); return 0; }
    }
    I2C_Stop();
    Delay_ms(10);
    
    // Set distance mode (Long mode - ~1.3m max range)
    if(!VL53L1X_Set_Distance_Mode(Long)) return 0;
    Delay_ms(5);
    
    // Set measurement timing budget to 50ms (50000 microseconds)
    // This controls measurement accuracy and speed
    if(!VL53L1X_Set_Measurement_Timing_Budget(50000)) return 0;
    Delay_ms(5);
    
    return 1;
}



unsigned char VL53L1X_Set_Distance_Mode(DistanceMode mode)
{
    unsigned char timeout_val;

    switch(mode)
    {
        case Short:
            timeout_val = DISTANCE_MODE_SHORT_TIMEOUT;  // 0x14
            break;

        case Medium:
            timeout_val = DISTANCE_MODE_MEDIUM_TIMEOUT;  // 0x0B
            break;

        case Long:
        default:
            timeout_val = DISTANCE_MODE_LONG_TIMEOUT;  // 0x0A
            break;
    }

    // Set distance mode timeout in PHASECAL_CONFIG__TIMEOUT_MACROP register
    // This register controls the distance measurement mode
    if(!I2C_Write_Register(PHASECAL_CONFIG__TIMEOUT_MACROP, timeout_val)) return 0;

    current_distance_mode = mode;
    return 1;
}

unsigned char VL53L1X_Set_Measurement_Timing_Budget(unsigned int budget_us)
{
    // Convert microseconds to timing budget value
    // The sensor uses internal timing units
    // Timing budget in register: roughly (budget_us / 1000) - offset
    
    unsigned int timing_val;
    
    // Convert 50000us (50ms) to register value
    // Typical values: 20000us=20, 33000us=33, 50000us=50
    if(budget_us >= 50000)
    {
        // Long mode timing budget
        timing_val = 0x00C8;  // 50ms
    }
    else if(budget_us >= 33000)
    {
        // Medium mode timing budget
        timing_val = 0x0080;  // 33ms
    }
    else
    {
        // Short mode timing budget
        timing_val = 0x0050;  // 20ms
    }
    
    // Write timing budget to register
    if(!I2C_Write_Register16(MEASUREMENT_TIMING_BUDGET_CONFIG, (int)timing_val)) return 0;
    
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
    // Inter-measurement period must be >= timing budget
    // Convert ms to internal units (approximately period_ms * 1000)
    unsigned int period_us = (unsigned int)period_ms * 1000;
    
    // Clear interrupt before starting
    if(!I2C_Write_Register(SYSTEM__INTERRUPT_CLEAR, 0x01)) return 0;
    Delay_ms(1);

    // Start continuous ranging (0x40 = timed mode)
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
    
    // Parse status byte - range status is in upper nibble (bits 7:4)
    unsigned char range_status = (status >> 4) & 0x0F;

    if(!I2C_Read_Register16(RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM, &range_mm)) return 0;
    if(!I2C_Read_Register16(RESULT__PEAK_SIGNAL_COUNT_RATE_MCPS, &signal_rate)) return 0;
    if(!I2C_Read_Register16(RESULT__AMBIENT_COUNT_RATE_MCPS, &ambient_rate)) return 0;

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

    // Clear interrupt for next measurement
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
