#ifndef _PCF8575_H
#define _PCF8575_H

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"


class PCF8575 {
    private:
        int _sda,_scl;
        i2c_inst_t* _i2c;
        int _addr;
        uint8_t _write_port[2];
    public:
        PCF8575(int sda, int scl, i2c_inst_t * i2c, int addr);
        void init();
        void gpio_put(int _pin, int state);
        int gpio_get(int _pin);
        void reset();
};

PCF8575::PCF8575(int sda, int scl, i2c_inst_t * i2c, int addr){
    _sda = sda;
    _scl = scl;
    _i2c = i2c;
    _addr = addr;
}

void PCF8575::init(){
            
    _write_port[0] = 0x00;
    _write_port[1] = 0x00;

    i2c_init(_i2c, 400 * 1000);

    gpio_set_function(_sda, GPIO_FUNC_I2C);
    gpio_set_function(_scl, GPIO_FUNC_I2C);
    gpio_pull_up(_sda);
    gpio_pull_up(_sda);

}

void PCF8575::gpio_put(int _pin, int state){
    uint8_t cmd = (_addr);
    if(_pin >= 0 || _pin < 18){
        if(_pin == 8 || _pin ==9){
            return;
        }

        if(state == 1){
            if(_pin < 8){
                //port1
                _write_port[0] |= (0x01 << _pin);
            }
            else if( _pin > 9){
                //port2
                _write_port[1] |= (0x01 << (_pin - 10));
            }
        }

        else if(state == 0){
            if(_pin < 8){
                //port1
                _write_port[0] &= ~(0x01 << _pin);
            }
            else if( _pin > 9){
                //port2
                _write_port[1] &= ~(0x01 << (_pin - 10));
            }
            else{
                return;
            }
        }

        i2c_write_blocking(_i2c,cmd,_write_port,2,false);

    }
    else{
        return;    
    }
}

int PCF8575::gpio_get(int _pin){
    uint8_t cmd = (_addr);
    uint8_t data[2];
    if(_pin >= 0 || _pin < 18){
        if(_pin == 8 || _pin ==9){
            return -1;
        }

        i2c_read_blocking(_i2c,cmd,data,2,false);

        if(_pin<8){
            int _bit = ((data[0]) & (0x01 << (_pin - 1)))>>(_pin - 1);
            return _bit;
        }
        else if( _pin >9){
            int _bit = ((data[0]) & (0x01 << (_pin - 1)))>>(_pin - 1);
            return _bit;
        }
        else{
            return -1;
        }
    }
    else{
        return -1;    
    }
}

void PCF8575::reset(){
    uint8_t cmd = (_addr << 1) | (0x00);
    _write_port[0] = 0x00;
    _write_port[1] = 0x00;
    i2c_write_blocking(_i2c,cmd,_write_port,2,false);
}
#endif