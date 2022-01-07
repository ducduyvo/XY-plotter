#include <stdio.h>
#include <string.h>
#include "parser.h"
#include "../printer.h"
#include "Gcode.h"
#include "../usb/user_vcom.h"


#include <stdio.h>
#include <string.h>

static Gcode::Data data;

static Gcode G1  = Gcode(Gcode::Letter::G, Gcode::Number::_1 , g1ExtractData);  // go to position
static Gcode G28 = Gcode(Gcode::Letter::G, Gcode::Number::_28, g28ExtractData); // Go to origin
static Gcode M1  = Gcode(Gcode::Letter::M, Gcode::Number::_1 , m1ExtractData);  // set penpos
static Gcode M2  = Gcode(Gcode::Letter::M, Gcode::Number::_2 , m2ExtractData);  // save pen up/do wn
static Gcode M4  = Gcode(Gcode::Letter::M, Gcode::Number::_4 , m4ExtractData);  // set laser po wer
static Gcode M5  = Gcode(Gcode::Letter::M, Gcode::Number::_5 , m5ExtractData);  // save stepper directions, area and speed
static Gcode M10 = Gcode(Gcode::Letter::M, Gcode::Number::_10, m10ExtractData); // reply to mdraw with all values
static Gcode M11 = Gcode(Gcode::Letter::M, Gcode::Number::_11, m11ExtractData); // get the limit switches from plotter

#define GCODE_SIZE 8
static Gcode *gcodes[GCODE_SIZE] = {
    &G1,
    &G28,
    &M1,
    &M2,
    &M4,
    &M5,
    &M10,
    &M11,
};

void parseCode(const char *str, QueueHandle_t &queue) {
    char gcode[5];
    bool found = false;
    char letter;
    uint8_t number;
    strncpy(gcode, str, 5);

    if (sscanf(gcode, "%c%hhu", &letter, &number) != 2) {
        ITM_print("Letter and/or number not found\n");
        return;
    }

    Gcode::Id id = CREATE_GCODE_ID(letter, number);
    for (uint8_t i = 0; i < GCODE_SIZE; ++i) {
        if (gcodes[i]->getId() == id) {
            found = true;
            if (gcodes[i]->callback(str)) { // returns true if data was extracted correctly (returns true for gcodes that dont need data extracted)
                data.id = gcodes[i]->getId();
                if (xQueueSendToBack(queue, &data, portMAX_DELAY) != pdTRUE) {
                	ITM_print("Error: Couldnt send data to queue even though waited for portMAX_DELAY\n");
                }

                // if not M10 or M11 (these will send ok from plotter with values)
            	if (data.id != M10.getId() && data.id != M11.getId()) {
                    USB_send((uint8_t *) OK_MESSAGE, strlen(OK_MESSAGE));
                    ITM_print("Send OK\n");
                }
                else{
                    ITM_print("Dont send OK\n");
                }

            }

            else {
                ITM_print("couldn't extract the data\n");
            }
            break;
        }
    }

    if (!found) {
        ITM_print("Error: ");
        ITM_print("%s is unknown Gcode\n", gcode);
    }
    ITM_print("\n");
}



/*FUNCTIONS*/
// Each gcode has its own extract function that we attach to the corresponding gcode variable,
// by a callback function.

/* Set penpos */
bool m1ExtractData(const char *str) {
    ITM_print("M1: ");
    if (sscanf( str,
                M1.toFormat(),
                &data.data.m1.penPos) == 1) {
        ITM_print("pen position: %u\n", data.data.m1.penPos);
        return true;
    }
    return false;
}

/* Save pen up and down */
bool m2ExtractData (const char *str) {
    ITM_print("M2: ");
    if (sscanf( str,
                M2.toFormat(),
                &data.data.m2.savePenUp,
                &data.data.m2.savePenDown) == 2) {
        ITM_print("up: %u  down: %u\n",data.data.m2.savePenUp, data.data.m2.savePenDown);
        return true;
    }
    return false;
}

/*M4: Set laser power*/
bool m4ExtractData (const char *str) {
    ITM_print("M4: ");
    if (sscanf(
                str,
                M4.toFormat(),
                &data.data.m4.laserPower) == 1) {
        ITM_print("Power level of laser: %u\n", data.data.m4.laserPower);
        return true;
    }
    return false;
}

/*M5: Save stepper directions, plot area, and plotting speed*/
bool m5ExtractData(const char *str) {
    ITM_print("M5: ");
    if (sscanf(
                str,
                M5.toFormat(),
                &data.data.m5.dirX,
                &data.data.m5.dirY,
                &data.data.m5.height,
                &data.data.m5.width,
                &data.data.m5.speed
            ) == 5) {
        ITM_print("X direction: %d, Y direction: %d, canvas dimensions: %d x %d, plotting speed: %d\n",
                  data.data.m5.dirX,
                  data.data.m5.dirY,
                  data.data.m5.width,
                  data.data.m5.height,
                  data.data.m5.speed);
        return true;
    }
    return false;
}

/* Reply to mdraw with all values? */
bool m10ExtractData (const char *str) {
    ITM_print("M10\n");
    return true;
}

/*M11: Limit switch status query*/
bool m11ExtractData (const char *str) {
    ITM_print("M11\n");
    return true;
}

/*g1: Move to coordinate*/
bool g1ExtractData (const char *str) {
    if (sscanf(
                str,
                G1.toFormat(),
                &data.data.g1.moveX,
                &data.data.g1.moveY,
                &data.data.g1.relative
            ) == 3)
    {
        ITM_print("G1: ");
        ITM_print("Moving to %s coordinates X %.2f and Y %.2f\n",
                data.data.g1.relative ? "relative" : "absolute",
                  data.data.g1.moveX,
                  data.data.g1.moveY
                 );
        return true;
    }
    return false;
}

/*G28: Move to origin*/
bool g28ExtractData (const char *str) {
    ITM_print("G28: Moving to origin\n");
    data.data.g1.moveX = 0;
    data.data.g1.moveY = 0;
    data.data.g1.relative = false;
    return true;
}
