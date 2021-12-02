/*
  Simple test of RC output interface with Menu
  Attention: If your board has safety switch,
  don't forget to push it to enable the PWM output.
*/

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Menu/AP_Menu.h>
#include <AP_Param/AP_Param.h>

// #if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
// #include <AP_BoardConfig/AP_BoardConfig.h>
// #include <AP_IOMCU/AP_IOMCU.h>
// AP_BoardConfig BoardConfig;
// #endif

// we need a boardconfig created so that the io processor's enable
// parameter is available
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_IOMCU/AP_IOMCU.h>
AP_BoardConfig BoardConfig;
#endif

void setup();
void loop();
void drive(uint16_t hz_speed);

#define MENU_FUNC(func) FUNCTOR_BIND(&commands, &Menu_Commands::func, int8_t, uint8_t, const Menu::arg *)

// Just so that it’s completely clear…
#define ENABLED 1
#define DISABLED 0

// this avoids a very common config error
#define ENABLE ENABLED
#define DISABLE DISABLED
#define ESC_HZ     490
#define SERVO_HZ    50

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

class Menu_Commands {
public:
    /* Menu commands to drive a SERVO type with
     * repective PWM output freq defined by SERVO_HZ
     */
    int8_t menu_servo(uint8_t argc, const Menu::arg *argv);

    /* Menu commands to drive a ESC type with
     * repective PWM output freq defined by ESC_HZ
     */
    int8_t menu_esc(uint8_t argc, const Menu::arg *argv);
};



Menu_Commands commands;

void setup(void) {
    hal.console->printf("Starting AP_HAL::RCOutput test\n");

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    BoardConfig.init();
#endif
    for (uint8_t i = 0; i < 14; i++) {
        hal.rcout->enable_ch(i);
        hal.console->printf("Enable Channel %u \n", i);
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
//Change parameters for board
float SET_PWM_PARAMETER = 14;
AP_Param::set_object_value(&BoardConfig, BoardConfig.var_info, "PWM_COUNT", SET_PWM_PARAMETER);
AP_Param::set_object_value(&BoardConfig, BoardConfig.var_info, "SAFETYENABLE", DISABLE);
AP_Param::set_object_value(&BoardConfig, BoardConfig.var_info, "SAFETYOPTION", 0);// 0 1 2
AP_Param::set_object_value(&BoardConfig, BoardConfig.var_info, "SAFETY_MASK", 8191);
#endif

}

static uint16_t pwm = 1300;
//static int8_t delta = 50;

/* Function to drive a RC output TYPE especified */
void drive(uint16_t hz_speed) {
    hal.rcout->set_freq(0xFF, hz_speed);

    while (1) {
        for (uint8_t i = 0; i < 14; i++) {
            hal.rcout->enable_ch(i);
            hal.console->printf("Enabled Channel %u \n", i);
            hal.rcout->write(i, pwm);
            hal.console->printf("%d constant\n",pwm);

            hal.console->printf("Driving Channel %u at %u Hz and PWM %u, last sent %u, read output value: %u\n",
            i,
            hal.rcout->get_freq(i),
            pwm,
            hal.rcout->read_last_sent(i),
            hal.rcout->read(i));

            // // hal.console->printf("Enabled PWMs: %u, Safety Mask: %u, IO Enable: %u \n",
            // BoardConfig.get_safety_mask(),
            // BoardConfig.io_enabled());

            
           /* if (delta > 0 && pwm <1500) 
            {
                pwm += delta;
                hal.console->printf("%d speeding\n",pwm);
            } 
            else if(pwm <= 1500 && pwm >1000) 
            {
                delta = -50;
                pwm += delta;
                hal.console->printf("%d dheere\n",pwm);
            }*/
        }
        hal.scheduler->delay(5);
        if (hal.console->available()) {
            break;
        }
    }
    if(hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_ARMED){
    hal.console->printf("SAFETY_ARMED \n");
    }
    if(hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_DISARMED){
        hal.console->printf("SAFETY_DISARMED \n");
    }
    if(hal.util->safety_switch_state() == AP_HAL::Util::SAFETY_NONE){
        hal.console->printf("SAFETY_NONE \n");
    }

    hal.console->printf("ARMED?: %d",hal.util->get_soft_armed());

    hal.scheduler->delay(5); //was 5
    
}

int8_t Menu_Commands::menu_servo(uint8_t argc, const Menu::arg *argv) {
    drive(SERVO_HZ);
    return 0;
}

int8_t Menu_Commands::menu_esc(uint8_t argc, const Menu::arg *argv) {
    drive(ESC_HZ);
    return 0;
}

const struct Menu::command rcoutput_menu_commands[] = {
    { "servo",          MENU_FUNC(menu_servo) },
    { "esc",            MENU_FUNC(menu_esc) },
};

MENU(menu, "Menu: ", rcoutput_menu_commands);

/*void setup(void) {
    hal.console->printf("Starting AP_HAL::RCOutput test\n");

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    BoardConfig.init();
#endif
    for (uint8_t i = 0; i < 14; i++) {
        hal.rcout->enable_ch(i);
    }
}
*/
void loop(void) {
    /* We call and run the menu, you can type help into menu to show commands
     * available */
    hal.rcout->force_safety_off();

    menu.run();
}

AP_HAL_MAIN();
