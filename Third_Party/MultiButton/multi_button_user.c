/*
 * multi_button_user.c
 *
 *  Created on: Oct 5, 2025
 *      Author: Diamo
 */
#include "multi_button_user.h"
#include "logger.h"
#include "main.h"

#define MAX_BUTTONS  (2)

static int demo_mode = 1;
static int verbose_mode = 0;

// Button instances
static Button buttons[MAX_BUTTONS];

// Hardware abstraction layer function
uint8_t read_button_gpio(uint8_t button_id) {
  uint8_t result;
  switch (button_id) {
    case 1:
      result = HAL_GPIO_ReadPin(USER_BUTTON1_GPIO_Port, USER_BUTTON1_Pin);
      break;
    case 2:
      result = HAL_GPIO_ReadPin(USER_BUTTON2_GPIO_Port, USER_BUTTON2_Pin);
    break;
    default:
      return 0;
  }
  return result;
}

// Generic event handler that shows button info
void generic_event_handler(Button* btn, const char* event_name)
{
    if (verbose_mode) {
        logger_debug("Button %d: %s (repeat: %d, pressed: %s)",
               btn->button_id,
               event_name,
               button_get_repeat_count(btn),
               button_is_pressed(btn) ? "Yes" : "No");
    } else {
      logger_info("Button %d: %s", btn->button_id, event_name);
    }
}

// Special handler for button configuration
void on_config_button_click(Button* btn)
{
    static int config_state = 0;

    logger_info("Config Button %d clicked!", btn->button_id);

    switch (config_state) {
        case 0:
            verbose_mode = !verbose_mode;
            logger_info("Verbose mode: %s", verbose_mode ? "ON" : "OFF");
            break;
        case 1:
            demo_mode = !demo_mode;
            logger_info("Demo mode: %s", demo_mode ? "ON" : "OFF");
            break;
        case 2:
          logger_info("Resetting all buttons...");
            for (int i = 0; i < MAX_BUTTONS; i++) {
                button_reset(&buttons[i]);
            }
            break;
        default:
            break;
    }
    config_state = (config_state + 1) % 3;
}

// Event handlers
void on_press_down(Button* btn) { generic_event_handler(btn, "Press Down"); }
void on_press_up(Button* btn) { generic_event_handler(btn, "Press Up"); }
void on_single_click(Button* btn) { generic_event_handler(btn, "Single Click"); }
void on_double_click(Button* btn) {
  generic_event_handler(btn, "Double Click");
  HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
}
void on_long_press_start(Button* btn) { generic_event_handler(btn, "Long Press Start"); }
void on_long_press_hold(Button* btn) { generic_event_handler(btn, "Long Press Hold"); }
void on_press_repeat(Button* btn) { generic_event_handler(btn, "Press Repeat"); }

// Initialize a single button with all event handlers
void init_button(int index, uint8_t button_id, int enable_all_events)
{
    button_init(&buttons[index], read_button_gpio, 0, button_id);
    if (enable_all_events) {
        button_attach(&buttons[index], BTN_PRESS_DOWN, on_press_down);
        button_attach(&buttons[index], BTN_PRESS_UP, on_press_up);
        button_attach(&buttons[index], BTN_SINGLE_CLICK, on_single_click);
        button_attach(&buttons[index], BTN_DOUBLE_CLICK, on_double_click);
        button_attach(&buttons[index], BTN_LONG_PRESS_START, on_long_press_start);
        button_attach(&buttons[index], BTN_LONG_PRESS_HOLD, on_long_press_hold);
        button_attach(&buttons[index], BTN_PRESS_REPEAT, on_press_repeat);
    } else {
        // Only essential events
        button_attach(&buttons[index], BTN_SINGLE_CLICK, on_single_click);
        button_attach(&buttons[index], BTN_DOUBLE_CLICK, on_double_click);
        button_attach(&buttons[index], BTN_LONG_PRESS_START, on_long_press_start);
    }
    button_start(&buttons[index]);
}

// Initialize all buttons
void buttons_init(void)
{
    init_button(0, 1, 0);
    init_button(1, 2, 0);
    // setting special event callback
    button_detach(&buttons[1], BTN_DOUBLE_CLICK);
    button_attach(&buttons[1], BTN_DOUBLE_CLICK, on_config_button_click);
}
