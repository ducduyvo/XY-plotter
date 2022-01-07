#ifndef PLOTTER_H_
#define PLOTTER_H_
#include "motor.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "Gcode.h"

// RIT timer specific variables and functions
using RIT_void_t = void (*)();
extern volatile uint32_t RIT_Count; // Counter just if some callback would want to use a global counter
extern RIT_void_t RIT_Callback;
extern SemaphoreHandle_t RIT_Semaphore;

extern void PlotterIsrFunction();
extern void RIT_Start_polling(uint32_t count, int pps, RIT_void_t);
extern void RIT_Start_polling(int pps, RIT_void_t);
extern void RIT_Start_polling(int pps);
extern void RIT_Stop_polling();

// Defines and macros
#define USE_ACCEL 1
#define ACCEL_THRESHOLD_PERCENT 10 // How much accel and deaccel 0-50%
#define DEFAULT_PPS 1500
#define CALIBRATION_PPS 1000 // Should be small number in real hardware maybe 500
#define BOOL_TO_NUM(boolean) (boolean ? 1 : 0)
#define MOTORS_NULL(xMotor, yMotor) ((xMotor == nullptr || yMotor == nullptr) ?\
        (ITM_print("Atleast one motor not initalised! exiting %s\n", __FUNCTION__) || true) :\
        false)

#define PEN_INITIALISED   (1 << 0)
#define LASER_INITIALISED (1 << 1)
#define CALIBRATED        (1 << 2)
#define CALIBRATE_RUNS 1

#define TICKS_PER_SECOND  (1'000'000)
#define LASER_FREQ 1000
#define LASER_CYCLE 255
#define PEN_FREQ 50
#define minDuty (TICKS_PER_SECOND / 1000) // 1ms
#define maxDuty (TICKS_PER_SECOND / 500 ) // 2ms

class Plotter {
public:
    Plotter(Motor* xMotor_, Motor* yMotor_);
    static Plotter* activePlotter;
    void setMotors(Motor* xMotor_, Motor* yMotor_);
    void calibrate();
    void goToOrigin();
    void initPen();
    void setPenValue(uint8_t value);
    void initLaser();
    void setLaserPower(uint8_t pw);

    int calculatePps();
    void start_polling(int pps_);
    void stop_polling();
    void isrFunction();

    void moveIfInArea(bool xStep, bool yStep);
    void bresenham();
    void initBresenhamValues (int x1_,int y1_, int x2_,int y2_);
    void plotLine        (float x1,float y1, float x2,float y2);
    void plotLineAbsolute(float x1,float y1, float x2,float y2);
    void plotLineRelative(                   float x2,float y2);

    void handleGcodeData(const Gcode::Data& data);

    inline int getTotalStepX() { return totalStepX; };
    inline int getTotalStepY() { return totalStepY; };
    inline void setTotalStepX(int count) { totalStepX = count; };
    inline void setTotalStepY(int count) { totalStepY = count; };
    void setXStepInMM(int width)  { xStepMM = (float) totalStepX / width; }
    void setYStepInMM(int height) { yStepMM = (float) totalStepY / height; }
    int getBresenhamSteps() { return m_steps; }
    int getBresenhamCount() { return m_count; }
    int getCurrentX() { return currentX; }
    int getCurrentY() { return currentY; }

private:
    uint32_t status = 0;
    Motor* xMotor = nullptr;
    Motor* yMotor = nullptr;

    volatile int currentX;
    volatile int currentY;
    int totalStepX = 0;
    int totalStepY = 0;
    float xStepMM;
    float yStepMM;

    // M5 reply
    bool saveDirX;
    bool saveDirY;
    int savePlottingWidth     = 380;
    int savePlottingHeight    = 300;
    uint8_t savePlottingSpeed = 100; // in percent

    //Pen and Laser
    uint8_t savePenUp   = 160;
    uint8_t savePenDown = 90;
    uint8_t m_power;


    // m_ prefix for  Bresenham values only to make less confusing
    int  m_dx;
    int  m_dy;
    bool m_xGreater;
    int  m_D;
    int  m_steps;
    int  m_count;
    int  m_x;
    int  m_y;
    int  m_prevX;
    int  m_prevY;
    int  m_pps = DEFAULT_PPS;
    int  m_threshold;
};


#endif /* PLOTTER_H_ */

