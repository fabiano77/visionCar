#ifndef RPY_PWM_HAT_PCA9685_H
#define RPY_PWM_HAT_PCA9685_H

#include <string>//

class PCA9685 {
public:
    explicit PCA9685(const std::string &device = "/dev/i2c-1", int address = 0x40);

    void set_pwm_freq(double freq_hz);

    void set_pwm(int channel, uint16_t on, uint16_t off);

    void set_all_pwm(uint16_t on, uint16_t off);

    void set_pwm_ms(int channel, double ms);

private:
    int i2c_fd;
    
    // Default frequency pulled from PCA9685 datasheet.
    double frequency = 200.0;

    void check_ret(int ret, std::string msg = "");

};

// Registers/etc:
constexpr uint8_t MODE1              = 0x00;
constexpr uint8_t MODE2              = 0x01;
constexpr uint8_t SUBADR1            = 0x02;
constexpr uint8_t SUBADR2            = 0x03;
constexpr uint8_t SUBADR3            = 0x04;
constexpr uint8_t PRESCALE           = 0xFE;
constexpr uint8_t LED0_ON_L          = 0x06;
constexpr uint8_t LED0_ON_H          = 0x07;
constexpr uint8_t LED0_OFF_L         = 0x08;
constexpr uint8_t LED0_OFF_H         = 0x09;
constexpr uint8_t ALL_LED_ON_L       = 0xFA; //
constexpr uint8_t ALL_LED_ON_H       = 0xFB;
constexpr uint8_t ALL_LED_OFF_L      = 0xFC;
constexpr uint8_t ALL_LED_OFF_H      = 0xFD;

// Bits:
constexpr uint8_t RESTART            = 0x80;
constexpr uint8_t SLEEP              = 0x10;
constexpr uint8_t ALLCALL            = 0x01;
constexpr uint8_t INVRT              = 0x10;
constexpr uint8_t OUTDRV             = 0x04;

#endif //RPY_PWM_HAT_PCA9685_H
