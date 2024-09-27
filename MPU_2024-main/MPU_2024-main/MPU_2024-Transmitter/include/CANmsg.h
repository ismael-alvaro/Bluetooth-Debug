#ifndef CANMSG_H
#define CANMSG_H

#include "CAN/can_common.h"
#include "CAN/esp32_can_builtin.h"
#include "CAN/esp32_can.h"

CAN_FRAME Msg;

#define CAN CAN0

class CANmsg
{
    private:
    protected:
        CAN_FRAME msg;
        int len = 0;
        bool f = false;

        void setup()
        {
            msg.id = 0x00;
            msg.length = 8;
            msg.extended = 0; 
            msg.rtr = 0;
        }
    public:
        CANmsg(gpio_num_t rx_id, gpio_num_t tx_id, uint32_t Baudrate) : msg(Msg) 
        { 
            CAN.setCANPins(rx_id, tx_id); 
            CAN.begin(Baudrate); 
        };

        ~CANmsg() 
        { 
            if(f)
            { 
                CAN.removeCallback(); 
                f = false;
            }
        };

        void init(void (*callback)(CAN_FRAME *)) 
        {
            f = true;
            CAN.watchFor();
            CAN.setCallback(0, callback);
            this->setup();
        } 

        void clear(uint32_t new_id)
        {
            len = 0;
            msg.id = new_id;
            msg.length = 8;
            msg.extended = 0; 
            msg.rtr = 0;
            memset(msg.data.uint8, 0, 8);
        }

        bool write()
        {
            return CAN.sendFrame(msg);
        }

        template<class T>
        CANmsg &operator<<(const T value)
        {
            //if(len + sizeof(T) > 8) return NULL
            memcpy(&msg.data.uint8[len], (uint8_t *)&value, sizeof(T));
            len += sizeof(T);
            return *this;
        }
};


#endif