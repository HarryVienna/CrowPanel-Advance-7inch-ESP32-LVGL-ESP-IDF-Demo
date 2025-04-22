
// Buzzer Configuration
#define BUZZER_GPIO GPIO_NUM_8 
#define BUZZER_LEDC_TIMER LEDC_TIMER_1
#define BUZZER_LEDC_CHANNEL LEDC_CHANNEL_1

// I2C
#define I2C_SCL          GPIO_NUM_16
#define I2C_SDA          GPIO_NUM_15
#define I2C_RST          GPIO_NUM_NC
#define I2C_CLK_SPEED_HZ 400000
#define I2C_NUM          I2C_NUM_0

// LCD
#define LCD_PIXEL_CLOCK_HZ     (18 * 1000 * 1000)

#define PIN_NUM_HSYNC          GPIO_NUM_40
#define PIN_NUM_VSYNC          GPIO_NUM_41
#define PIN_NUM_DE             GPIO_NUM_42
#define PIN_NUM_PCLK           GPIO_NUM_39
#define PIN_NUM_DATA0          GPIO_NUM_21   // B0
#define PIN_NUM_DATA1          GPIO_NUM_47   // B1
#define PIN_NUM_DATA2          GPIO_NUM_48   // B2
#define PIN_NUM_DATA3          GPIO_NUM_45   // B3
#define PIN_NUM_DATA4          GPIO_NUM_38   // B4
#define PIN_NUM_DATA5          GPIO_NUM_9    // G0
#define PIN_NUM_DATA6          GPIO_NUM_10   // G1
#define PIN_NUM_DATA7          GPIO_NUM_11   // G2
#define PIN_NUM_DATA8          GPIO_NUM_12   // G3
#define PIN_NUM_DATA9          GPIO_NUM_13   // G4
#define PIN_NUM_DATA10         GPIO_NUM_14   // G5
#define PIN_NUM_DATA11         GPIO_NUM_7    // R0
#define PIN_NUM_DATA12         GPIO_NUM_17   // R1
#define PIN_NUM_DATA13         GPIO_NUM_18   // R2
#define PIN_NUM_DATA14         GPIO_NUM_3    // R3
#define PIN_NUM_DATA15         GPIO_NUM_46   // R4
#define PIN_NUM_DISP_EN        GPIO_NUM_NC

#define HSYNC_BACK_PORCH  8
#define HSYNC_FRONT_PORCH 8
#define HSYNC_PULSE_WIDTH 4
#define VSYNC_BACK_PORCH  8
#define VSYNC_FRONT_PORCH 8
#define VSYNC_PULSE_WIDTH 4

#define LCD_H_RES         800
#define LCD_V_RES         480

// LVGL
#define LVGL_TASK_DELAY_MS   10
#define LVGL_TASK_STACK_SIZE (4 * 1024)
#define LVGL_TASK_PRIORITY   0