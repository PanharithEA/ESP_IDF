#include <stdio.h>
#include <stddef.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rotenc.h"
#include "driver/mcpwm.h"
#include "soc/mcpwm_periph.h"

#define ROTENC_CLK_PIN 19
#define ROTENC_DT_PIN 21
static int rotenc_pos = 0;
static int motor_pos = 0;

#define GPIO_PWM0A_OUT 32   //Set GPIO 15 as PWM0A
#define GPIO_PWM0B_OUT 35   //Set GPIO 16 as PWM0B

static void mcpwm_example_gpio_initialize(void)
{
    printf("initializing mcpwm gpio...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);
}

static void brushed_motor_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}

static void mcpwm_example_brushed_motor_control(void *arg)
{
   //1. mcpwm gpio initialization
   mcpwm_example_gpio_initialize();

   //2. initial mcpwm configuration
   printf("Configuring Initial Parameters of mcpwm...\n");
   mcpwm_config_t pwm_config;
   pwm_config.frequency = 1000;    //frequency = 500Hz,
   pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
   pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
   pwm_config.counter_mode = MCPWM_UP_COUNTER;
   pwm_config.duty_mode = MCPWM_DUTY_MODE_0;
   mcpwm_init(MCPWM_UNIT_0, MCPWM_TIMER_0, &pwm_config);    //Configure PWM0A & PWM0B with above settings
   while (1) {
       brushed_motor_forward(MCPWM_UNIT_0, MCPWM_TIMER_0, 70.0);
       vTaskDelay(1000 / portTICK_RATE_MS);
   }
}
static void init_hw(void)
{
    rotenc_init(ROTENC_CLK_PIN, ROTENC_DT_PIN);

}


static void print_rotenc_pos(void *arg)
{
    while (1)
    {
        rotenc_pos = rotenc_getPos();
        printf("pos: %d\n", rotenc_pos);
        vTaskSuspend(NULL);
    }
}

void app_main()
{

	printf("rotory is in processing");
    init_hw();
    xTaskCreate(mcpwm_example_brushed_motor_control, "mcpwm_examlpe_brushed_motor_control", 4096, NULL, 5, NULL);
    rotenc_setPosChangedCallback(print_rotenc_pos);



}
