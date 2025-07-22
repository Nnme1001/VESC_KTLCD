#include "commands.h"
#include "ch.h"
#include "hal.h"
#include "mc_interface.h"
#include "stm32f4xx_conf.h"
#include "buffer.h"
#include "terminal.h"
#include "hw.h"
#include "app.h"
#include "timeout.h"
#include "utils_sys.h"
#include "packet.h"
#include "qmlui.h"
#include "crc.h"
#include "main.h"
#include "utils.h"
#include "comm_can.h"
#include "servo_dec.h"

#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>


extern void app_uartcomm_send_raw_packet(unsigned char *data, unsigned int len, UART_PORT port_number);

#define LCD3_REPLY_PACKET_SIZE 12

#define MOVING_ANIMATION_THROTTLE (1 << 1)
#define MOVING_ANIMATION_CRUISE  (1 << 3)
#define MOVING_ANIMATION_ASSIST  (1 << 4)
#define MOVING_ANIMATION_BRAKE  (1 << 5)

// Глобальная переменная для отслеживания времени переключения вывода температуры
static systime_t last_temp_toggle_time = 0;

void lcd3_process_packet(unsigned char *data, unsigned int len,
    void(*reply_func)(unsigned char *data, unsigned int len)) {

    (void)len;
    (void)reply_func;

    volatile mc_configuration *mcconf = (volatile mc_configuration*) mc_interface_get_configuration();

    uint8_t lcd_pas_mode = data[1] & 7; //speedbutton. Ignore first 5 bit
    bool fixed_throttle_level = (data[4] >> 4) & 1; //p4
    bool temp_mode = (data[10] >> 2) & 1; //c13
    bool l3 = (data[10] >> 0) & 1; //l3
    bool pas_one_magnet = (data[11] >> 6) & 1; //l1
    bool light_kt_on = (data[1] >> 7) & 1; //Lights
    bool adc_scaling = (data[6] >> 0) & 1; //c2

    #ifndef HW_HAS_WHEEL_SPEED_SENSOR
    bool servo_stop;
    bool read_servo;
    if (servodec_is_running()) {
        servodec_stop();
        servo_stop = 0;
        read_servo = 0;
    } else {
        servo_stop = 1;
    }
    if (servo_stop && !read_servo) {
        palSetPadMode(HW_ICU_GPIO, HW_ICU_PIN, PAL_MODE_INPUT_PULLUP);
        read_servo = 1;
    }

    #endif

    if(pas_one_magnet) {
        if(light_kt_on) {
            palSetPadMode(GPIOA, 14, PAL_HIGH);
        } else {
            palSetPadMode(GPIOA, 14, PAL_LOW);
        }
    }

    if(pas_one_magnet) {
        app_pas_set_one_magnet(true);
    } else if (pas_one_magnet == 0) {
        app_pas_set_one_magnet(false);
    }

// Изменение порогов напряжения батреи 13S в зависимости от того, подключена ли повышайка (+16,8 вольт)
	float v = (float)GET_INPUT_VOLTAGE();
    if (v < 56.0) { // повышайка не подключена, но диоды повышайки в выключенном состоянии крадут 0,5 вольта
    mcconf->l_min_vin = 39.5; //40-0.5
	mcconf->l_max_vin = 54.1; //54.6-0.5
	mcconf->l_battery_cut_start = 42.5; //43-0.5
	mcconf->l_battery_cut_end = 39.5; //40-0.5
	}
    else { // повышайка подключена, к напряжению добавлено +16,8 вольт
    mcconf->l_min_vin = 56.8; //40+16.8
	mcconf->l_max_vin = 72.6; //54.6+16.8+1.2 максимальный порог здесь чуть выше, чем +16,8, т.к. на холостом ходу повышайка добавляет 18 вольт
	mcconf->l_battery_cut_start = 59.8; //43+16.8
	mcconf->l_battery_cut_end = 56.8; //40+16.8
	}

// Теперь не только коэффициент тока, но и лимиты мощности и оборотов задаются в зависимости от выбранной передачи (по сути - здесь зашиты 5 профилей)
    float current_scale = 0.0;
	float current_power = 1000;
	float current_rpm = 9150; //~40км/ч

    if (lcd_pas_mode == 1) {
    current_scale = 0.33;
	current_power = 600;
	current_rpm = 6900; //~30км/ч
	}
    else if (lcd_pas_mode == 2) {
    current_scale = 0.5;
	current_power = 900;
	current_rpm = 9150; //~40км/ч
	}
    else if (lcd_pas_mode == 3) {
    current_scale = 0.72;
	current_power = 1300;
	current_rpm = 9150; //~40км/ч
	}
    else if (lcd_pas_mode == 4) {
    current_scale = 0.89;
	current_power = 1600;
	current_rpm = 11500; //~50км/ч
	}
    else if (lcd_pas_mode == 5) {
    current_scale = 1;
	current_power = 1800;
	current_rpm = 16000; //~70км/ч
	}



    if(fixed_throttle_level == 0) {
        mcconf->l_current_max_scale = 1.0;
        app_pas_set_current_sub_scaling(current_scale);
    } else {
        if(adc_scaling) {
            mcconf->l_current_max_scale = 1.0;
            app_adc_set_op_scaling(current_scale);
        } else {
            mcconf->l_current_max_scale = current_scale;
			mcconf->l_watt_max = current_power;
			mcconf->l_max_erpm = current_rpm;
        }
    }

    if((current_scale == 0.0) && l3) {
        if(adc_scaling) {
            app_adc_set_op_scaling(current_scale);
        } else {
            mcconf->l_current_max_scale = current_scale;
			mcconf->l_watt_max = current_power;
			mcconf->l_max_erpm = current_rpm;
        }
    }


    uint8_t sb[LCD3_REPLY_PACKET_SIZE];


    // Получаем скорость в км/ч
    float kmh = mc_interface_get_speed() * 3.6f;

    uint16_t pms = 0;

    // Если скорость меньше 0.5 км/ч, считаем её нулевой
    if (kmh < 0.5f) {
        pms = 0xFFFF; // специальное значение, которое дисплей интерпретирует как 0 км/ч
    } else {
        pms = (uint16_t)(7360.0f / kmh); //7360 - значение для колеса поумолчанию - 26"
        //окончательная подгонка спидометра к реальности - в vesctools по диаметру колеса (motor settings/aditional info/setup)
        if (pms > 0xFFFE) {
            pms = 0xFFFE; // ограничим максимум, чтобы не переполнить
        }
    }


    uint8_t batteryLevel;
    uint8_t batFlashing = 0;

    float l = mc_interface_get_battery_level(NULL);



    if (l > 0.7)
    batteryLevel = 4;
    else if (l > 0.4)
    batteryLevel = 3;
    else if (l > 0.2)
    batteryLevel = 2;
    else if (l > 0.1)
    batteryLevel = 1;
    else
    {
        batteryLevel = 0;
        if (l <= 0)
        batFlashing = 1;
    }


    int16_t can_current = 0;

    can_status_msg_4 *msg4 = comm_can_get_status_msg_4_index(0);
    if (msg4->id >= 0 && UTILS_AGE_S(msg4->rx_time) < 0.9) {
        can_current = msg4->current_in;
    } else {
        can_current = 0;
    }


    float w = (float)GET_INPUT_VOLTAGE() * (can_current + mc_interface_get_tot_current_in_filtered()) / 12;
    if (w < 0)
    w = 0;
    if (w > 255)
    w = 255;

    sb[0] = 0x41;

    // --- начало блока с цикличным выводом температуры ---
    systime_t now = chVTGetSystemTimeX();
    uint32_t elapsed_ms = (now - last_temp_toggle_time) * 1000 / CH_CFG_ST_FREQUENCY; // время в миллисекундах
    if (elapsed_ms >= 10000) {
        last_temp_toggle_time = now;
        elapsed_ms = 0;
    }

    bool show_temp = (elapsed_ms < 4000);

    int8_t temp_display;

    if (temp_mode && show_temp) {
        float motor_temp = mc_interface_temp_motor_filtered();
        float motor_temp_start = mcconf->l_temp_motor_start;
        float motor_temp_end = mcconf->l_temp_motor_end;

        if (motor_temp <= motor_temp_start) { 
            temp_display = 119;//Фиктивное значение 119 градусов, не приводящее к отображению. Отображение начинается от 120 град
        } else if (motor_temp >= motor_temp_end) {
            temp_display = 130;//крайнее значение отображаемой температуры. Можно расширить диапазон до отображаемых 140 град
        } else {
            float ratio = (motor_temp - motor_temp_start) / (motor_temp_end - motor_temp_start);
            temp_display = 120 + (int8_t)(ratio * 10.0f); // 10 - разница между краями приводимого диапазона = 130 - 120
        }
    } else if (temp_mode && !show_temp) {
            temp_display = 119;// Фиктивное значение 119 градусов для выключения отображения температуры на время 10000-4000=6000 мсек
    } else {
        // Если режим не temp_mode, выводим температуру FET как раньше
        temp_display = (int8_t) mc_interface_temp_fet_filtered();
    }
    // --- Конец блока ---


    //b1: battery level:
    // bit 0: border flashing,
    // bit 1: animated charging,
    // bit 3-5: level, (0-4)
    sb[1] = (batteryLevel << 2) | batFlashing;

    sb[2] = 0x30; //battery voltage

    // Кодируем pms в два байта
    sb[3] = (uint8_t)(pms >> 8); // старший байт
    sb[4] = (uint8_t)(pms & 0xFF); // младший байт

    sb[5] = 0; //b5: B5 error info display: 0x20: "0info  - Comunication line gault - all screen", 0x21: "6info - Motor or controller short circuit fault - all screen", 0x22: "1info -throtle fault - all screen", 0x23: "2info - Comunication line gault - all scren", 0x24: "3info - motor position sensor fault - all screen", 0x25: "0info - Comunication line gault - all screen", 0x26: "4info - Comunication line gault - all screen", 0x28: "0info - Comunication line gault - all screen"
    sb[6] = 0;

    //b7: moving animation ()
    // bit 0: -
    // bit 1: throttle
    // bit 2: -
    // bit 3: cruise
    // bit 4: assist
    // bit 5: brake
    // bit 6: -
    // bit 7: -
    sb[7] =
    ((app_adc_get_decoded_level() > 0) ? MOVING_ANIMATION_THROTTLE: 0) |
    (app_pas_get_reverse_pedaling() ? MOVING_ANIMATION_CRUISE: 0) |
    ((app_pas_get_pedal_rpm() > 11) ? MOVING_ANIMATION_ASSIST: 0) |
    ((app_adc_get_decoded_level2() > 0) ? MOVING_ANIMATION_BRAKE: 0);

    sb[8] = w; //b8: power in 13 wt increments (48V version of the controller)
    sb[9] = temp_display - 15; //b9: Дисплей отображает температуру, прибавляя к значению 15 градусов.

    sb[10] = 0; //
    sb[11] = 0; //

    uint8_t crc = 0;
    for (int n = 1; n < LCD3_REPLY_PACKET_SIZE; n++)
    crc ^= sb[n];

    sb[6] = crc;

    app_uartcomm_send_raw_packet(sb, len, UART_PORT_COMM_HEADER);
}

#define LCD3_RX_PACKET_SIZE 13
static uint8_t buffer[LCD3_RX_PACKET_SIZE];

void lcd3_process_byte(uint8_t rx_data, PACKET_STATE_t *state) {
    (void)state;

    memmove(buffer, &buffer[1], LCD3_RX_PACKET_SIZE - 1);
    buffer[LCD3_RX_PACKET_SIZE - 1] = rx_data;

    if (buffer[12] == 0x0e) {
        lcd3_process_packet(buffer, LCD3_RX_PACKET_SIZE, UART_PORT_COMM_HEADER);
    }
}



volatile float wheel_rpm_filtered = 0;
volatile float trip_odometer = 1.0;

void hw_update_speed_sensor(void) {
    static float wheel_rpm = 0;
    static uint8_t sensor_state = 0;
    static uint8_t sensor_state_old = 0;
    static float last_sensor_event_time = 0;
    float current_time = (float)chVTGetSystemTimeX() / (float)CH_CFG_ST_FREQUENCY;

    sensor_state = palReadPad(HW_ICU_GPIO, HW_ICU_PIN);

    if(sensor_state == 0 && sensor_state_old == 1) {
        float revolution_duration = current_time - last_sensor_event_time;

        if (revolution_duration > 0.09) {
            //ignore periods <110ms, which is about 83km/h
            last_sensor_event_time = current_time;
            wheel_rpm = 60.0 / revolution_duration;
            UTILS_LP_FAST(wheel_rpm_filtered, (float)wheel_rpm, 0.5);

            // For some reason a race condition on startup crashes the OS if this is executed too soon.
            // So don't track odometer for the first 4 seconds
            if(current_time > 4.0) {
                trip_odometer += mc_interface_get_configuration()->si_wheel_diameter * M_PI;
            }
        }
    } else {
        // After 3 seconds without sensor signal, set RPM as zero
        if ((current_time - last_sensor_event_time) > 3.0) {
            wheel_rpm_filtered = 0.0;
        }
    }
    sensor_state_old = sensor_state;
}

/* Get speed in m/s */
float hw_get_speed(void) {
    const volatile mc_configuration *conf = mc_interface_get_configuration();
    float speed = wheel_rpm_filtered * conf->si_wheel_diameter * M_PI / 60.0;
    return speed;
}

/* Get trip distance в meters */
float hw_get_distance(void) {
    return trip_odometer;
}

float hw_get_distance_abs(void) {
    return trip_odometer;
}