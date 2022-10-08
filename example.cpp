#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "include/PCF8575.hpp"

#define SDA 4
#define SCL 5
#define I2C i2c0
#define PCF8575_addr 0x20

i2c_inst_t* i2c = I2C;

int main(){
    stdio_init_all();
    PCF8575 expander(SDA,SCL,i2c,PCF8575_addr);
    expander.init();

    while(true){
        // uint8_t cmd = (_addr) | (0x00);
        // i2c_write_blocking(_i2c,cmd,_write_port,2,false);
        for(int i = 0; i < 8; i++){
            expander.gpio_put(i,1);
            sleep_ms(50);
        }
        for(int i = 10; i < 18; i++){
            expander.gpio_put(i,1);
            sleep_ms(50);
        }
        sleep_ms(1000);
        for(int i = 0; i < 8; i++){
            expander.gpio_put(i,0);
            sleep_ms(50);
        }
        for(int i = 10; i < 18; i++){
            expander.gpio_put(i,0);
            sleep_ms(50);
        }
        sleep_ms(1000);
        // uint8_t data[2];
        // i2c_read_blocking(_i2c,cmd,data,2,false);
        // sleep_ms(1000);
    }

}

