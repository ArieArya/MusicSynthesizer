/*General note: in many instances a value of a configuration parameter e.g. the volume setting, is read atomically into a local variable which is then used to take the appropriate action, this means that any changes to that configuration parameter will not
  take effect until the next execution of the task, this is imperceptible to the user given the high frequency of initiation).
  Moreover, this is not an issue if the task using the local value is interrupted and the global value is changed by another task(i.e. if the local value becomes invalid) because none of the reading tasks write to the global variables
  based on the value they read. Moreover, making a local copy of a shared variable is desirable as it reduces the risk of long-term priori*/

#include <U8g2lib.h>
#include <Wire.h>
#include <STM32FreeRTOS.h>

// -----------------------------------------------
// -------------- Timing definitions -------------
// -----------------------------------------------
//Timing tests for a given task Y can be carried out by uncommenting the PROFILE_Y #define and all the NO_X #define's where X is all tasks but Y 
//#define NO_SAMPLE_ISR
//#define NO_DISPLAY_UPDATE
//#define NO_MSG_IN
//#define NO_PREV_KEY_ARRAY_UPDATE
//#define NO_SAMPLE_WRITE_TASK
//#define NO_MSG_OUT //changes the queue size to allow 32 iterations without it filling up
//#define NO_SCAN_KEYS
//#define PROFILE_SCAN_KEYS // set scanKeysTask to only do 32 iterations
//#define PROFILE_SAMPLE_WRITE_TASK
//#define PROFILE_MSG_IN_TASK
//#define PROFILE_ISR
//#define PROFILE_DISPLAY_UPDATE_TASK
//#define PROFILE_MSG_OUT_TASK

const TickType_t SEMAPHORE_MAX_WAIT_TIME = 10 / portTICK_PERIOD_MS;// the worst-case scenario waiting time for the samplewritetask to obtain
// access to the double buffer it intends to write to, ideally the wait time will be near 0 as the sample ISR gives one of the two buffer semaphores
// whenever it finishes reading 220 samples which - given its frequency of 22 KHz - matches the 10ms initiation interval of the samplewrite task
// since the ISR first reads from bufferOne and the write task first writes to bufferTwo, there should be negligible delay for the semaphore to be acquired
// by the write task. The max

HardwareTimer *sampleTimer;


// -----------------------------------------------
// --------------- Pin definitions ---------------
// -----------------------------------------------

//Row select and enable
const int RA0_PIN = D3;
const int RA1_PIN = D6;
const int RA2_PIN = D12;
const int REN_PIN = A5;

//Matrix input and output
const int C0_PIN = A2;
const int C1_PIN = D9;
const int C2_PIN = A6;
const int C3_PIN = D1;
const int OUT_PIN = D11;

//Audio analogue out
const int OUTL_PIN = A4;
const int OUTR_PIN = A3;

//Joystick analogue in
const int JOYY_PIN = A0;
const int JOYX_PIN = A1;

//Output multiplexer bits
const int DEN_BIT = 3;
const int DRST_BIT = 4;
const int HKOW_BIT = 5;
const int HKOE_BIT = 6;

// -----------------------------------------------
// -------------- Global Constants ---------------
// -----------------------------------------------

const uint8_t SAWTOOTH = 0;
const uint8_t SINE = 1;

const bool BUFFER_ONE = false;
const bool BUFFER_TWO = true;

const uint32_t stepSizes [] = {51076063, 54113191, 57330941, 60740013, 64351807, 68178349 , 72232447 , 76527610 , 81078186 , 85899345 , 91007194 , 96418755};//step sizes for sawtooth wave
const uint32_t sineStepSizes[] = {60, 63, 67, 71, 75, 79, 84, 89, 94, 100, 106, 112};//step sizes for sine wave

const char intToHex[] = "0123456789ABCDEF";

// -----------------------------------------------
// --------- Volatile Glaobal Variables ----------
// -----------------------------------------------

// polyphony variables
volatile uint32_t polyFrequencies [] = {NULL, NULL, NULL};
volatile uint32_t currentStepSize_0 = 0;
volatile uint32_t currentStepSize_1 = 0;
volatile uint32_t currentStepSize_2 = 0;
volatile uint32_t sineCurrentStepSize_0 = 0;
volatile uint32_t sineCurrentStepSize_1 = 0;
volatile uint32_t sineCurrentStepSize_2 = 0;

volatile uint32_t periodLength = 58;
volatile uint32_t highLimit = 29;
volatile uint32_t lowLimit = 29;

volatile bool joyStickMode = false;
volatile uint8_t waveForm = SAWTOOTH;
volatile uint8_t buf[17000];
volatile uint8_t keyArray[7];
volatile uint8_t bufferOne[220];
volatile uint8_t bufferTwo[220];
volatile bool reverbSet = false;

//chose a minimal number of samples for the sine lookup table that still allows for reasonable approximations of the equal temperament frequencies
volatile uint8_t sineTable[5000] = {127, 128, 128, 128, 128, 128, 128, 129, 129, 129, 129, 129, 129, 130, 130, 130, 130, 130, 130, 131, 131, 131, 131, 131, 131, 131, 132, 132, 132, 132, 132, 132, 133, 133, 133, 133, 133, 133, 134, 134, 134, 134, 134, 134, 135, 135, 135, 135, 135, 135, 135, 136, 136, 136, 136, 136, 136, 137, 137, 137, 137, 137, 137, 138, 138, 138, 138, 138, 138, 138, 139, 139, 139, 139, 139, 139, 140, 140, 140, 140, 140, 140, 141, 141, 141, 141, 141, 141, 142, 142, 142, 142, 142, 142, 142, 143, 143, 143, 143, 143, 143, 144, 144, 144, 144, 144, 144, 145, 145, 145, 145, 145, 145, 145, 146, 146, 146, 146, 146, 146, 147, 147, 147, 147, 147, 147, 148, 148, 148, 148, 148, 148, 148, 149, 149, 149, 149, 149, 149, 150, 150, 150, 150, 150, 150, 151, 151, 151, 151, 151, 151, 151, 152, 152, 152, 152, 152, 152, 153, 153, 153, 153, 153, 153, 153, 154, 154, 154, 154, 154, 154, 155, 155, 155, 155, 155, 155, 156, 156, 156, 156, 156, 156, 156, 157, 157, 157, 157, 157, 157, 158, 158, 158, 158, 158, 158, 158, 159, 159, 159, 159, 159, 159, 160, 160, 160, 160, 160, 160, 160, 161, 161, 161, 161, 161, 161, 162, 162, 162, 162, 162, 162, 162, 163, 163, 163, 163, 163, 163, 164, 164, 164, 164, 164, 164, 164, 165, 165, 165, 165, 165, 165, 166, 166, 166, 166, 166, 166, 166, 167, 167, 167, 167, 167, 167, 168, 168, 168, 168, 168, 168, 168, 169, 169, 169, 169, 169, 169, 169, 170, 170, 170, 170, 170, 170, 171, 171, 171, 171, 171, 171, 171, 172, 172, 172, 172, 172, 172, 172, 173, 173, 173, 173, 173, 173, 174, 174, 174, 174, 174, 174, 174, 175, 175, 175, 175, 175, 175, 175, 176, 176, 176, 176, 176, 176, 176, 177, 177, 177, 177, 177, 177, 177, 178, 178, 178, 178, 178, 178, 179, 179, 179, 179, 179, 179, 179, 180, 180, 180, 180, 180, 180, 180, 181, 181, 181, 181, 181, 181, 181, 182, 182, 182, 182, 182, 182, 182, 183, 183, 183, 183, 183, 183, 183, 184, 184, 184, 184, 184, 184, 184, 185, 185, 185, 185, 185, 185, 185, 186, 186, 186, 186, 186, 186, 186, 187, 187, 187, 187, 187, 187, 187, 188, 188, 188, 188, 188, 188, 188, 189, 189, 189, 189, 189, 189, 189, 190, 190, 190, 190, 190, 190, 190, 190, 191, 191, 191, 191, 191, 191, 191, 192, 192, 192, 192, 192, 192, 192, 193, 193, 193, 193, 193, 193, 193, 194, 194, 194, 194, 194, 194, 194, 194, 195, 195, 195, 195, 195, 195, 195, 196, 196, 196, 196, 196, 196, 196, 196, 197, 197, 197, 197, 197, 197, 197, 198, 198, 198, 198, 198, 198, 198, 198, 199, 199, 199, 199, 199, 199, 199, 200, 200, 200, 200, 200, 200, 200, 200, 201, 201, 201, 201, 201, 201, 201, 202, 202, 202, 202, 202, 202, 202, 202, 203, 203, 203, 203, 203, 203, 203, 203, 204, 204, 204, 204, 204, 204, 204, 204, 205, 205, 205, 205, 205, 205, 205, 205, 206, 206, 206, 206, 206, 206, 206, 206, 207, 207, 207, 207, 207, 207, 207, 207, 208, 208, 208, 208, 208, 208, 208, 208, 209, 209, 209, 209, 209, 209, 209, 209, 210, 210, 210, 210, 210, 210, 210, 210, 211, 211, 211, 211, 211, 211, 211, 211, 211, 212, 212, 212, 212, 212, 212, 212, 212, 213, 213, 213, 213, 213, 213, 213, 213, 214, 214, 214, 214, 214, 214, 214, 214, 214, 215, 215, 215, 215, 215, 215, 215, 215, 215, 216, 216, 216, 216, 216, 216, 216, 216, 217, 217, 217, 217, 217, 217, 217, 217, 217, 218, 218, 218, 218, 218, 218, 218, 218, 218, 219, 219, 219, 219, 219, 219, 219, 219, 219, 220, 220, 220, 220, 220, 220, 220, 220, 220, 221, 221, 221, 221, 221, 221, 221, 221, 221, 221, 222, 222, 222, 222, 222, 222, 222, 222, 222, 223, 223, 223, 223, 223, 223, 223, 223, 223, 223, 224, 224, 224, 224, 224, 224, 224, 224, 224, 225, 225, 225, 225, 225, 225, 225, 225, 225, 225, 226, 226, 226, 226, 226, 226, 226, 226, 226, 226, 227, 227, 227, 227, 227, 227, 227, 227, 227, 227, 228, 228, 228, 228, 228, 228, 228, 228, 228, 228, 229, 229, 229, 229, 229, 229, 229, 229, 229, 229, 229, 230, 230, 230, 230, 230, 230, 230, 230, 230, 230, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 232, 232, 232, 232, 232, 232, 232, 232, 232, 232, 232, 233, 233, 233, 233, 233, 233, 233, 233, 233, 233, 233, 234, 234, 234, 234, 234, 234, 234, 234, 234, 234, 234, 234, 235, 235, 235, 235, 235, 235, 235, 235, 235, 235, 235, 235, 236, 236, 236, 236, 236, 236, 236, 236, 236, 236, 236, 236, 237, 237, 237, 237, 237, 237, 237, 237, 237, 237, 237, 237, 238, 238, 238, 238, 238, 238, 238, 238, 238, 238, 238, 238, 238, 239, 239, 239, 239, 239, 239, 239, 239, 239, 239, 239, 239, 239, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 241, 241, 241, 241, 241, 241, 241, 241, 241, 241, 241, 241, 241, 241, 242, 242, 242, 242, 242, 242, 242, 242, 242, 242, 242, 242, 242, 242, 242, 243, 243, 243, 243, 243, 243, 243, 243, 243, 243, 243, 243, 243, 243, 243, 244, 244, 244, 244, 244, 244, 244, 244, 244, 244, 244, 244, 244, 244, 244, 244, 245, 245, 245, 245, 245, 245, 245, 245, 245, 245, 245, 245, 245, 245, 245, 245, 246, 246, 246, 246, 246, 246, 246, 246, 246, 246, 246, 246, 246, 246, 246, 246, 246, 246, 247, 247, 247, 247, 247, 247, 247, 247, 247, 247, 247, 247, 247, 247, 247, 247, 247, 247, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 249, 249, 249, 249, 249, 249, 249, 249, 249, 249, 249, 249, 249, 249, 249, 249, 249, 249, 249, 249, 249, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 251, 251, 251, 251, 251, 251, 251, 251, 251, 251, 251, 251, 251, 251, 251, 251, 251, 251, 251, 251, 251, 251, 251, 251, 251, 251, 251, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 254, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 253, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 252, 251, 251, 251, 251, 251, 251, 251, 251, 251, 251, 251, 251, 251, 251, 251, 251, 251, 251, 251, 251, 251, 251, 251, 251, 251, 251, 251, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 249, 249, 249, 249, 249, 249, 249, 249, 249, 249, 249, 249, 249, 249, 249, 249, 249, 249, 249, 249, 249, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 248, 247, 247, 247, 247, 247, 247, 247, 247, 247, 247, 247, 247, 247, 247, 247, 247, 247, 247, 246, 246, 246, 246, 246, 246, 246, 246, 246, 246, 246, 246, 246, 246, 246, 246, 246, 246, 245, 245, 245, 245, 245, 245, 245, 245, 245, 245, 245, 245, 245, 245, 245, 245, 244, 244, 244, 244, 244, 244, 244, 244, 244, 244, 244, 244, 244, 244, 244, 244, 243, 243, 243, 243, 243, 243, 243, 243, 243, 243, 243, 243, 243, 243, 243, 242, 242, 242, 242, 242, 242, 242, 242, 242, 242, 242, 242, 242, 242, 242, 241, 241, 241, 241, 241, 241, 241, 241, 241, 241, 241, 241, 241, 241, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 239, 239, 239, 239, 239, 239, 239, 239, 239, 239, 239, 239, 239, 238, 238, 238, 238, 238, 238, 238, 238, 238, 238, 238, 238, 238, 237, 237, 237, 237, 237, 237, 237, 237, 237, 237, 237, 237, 236, 236, 236, 236, 236, 236, 236, 236, 236, 236, 236, 236, 235, 235, 235, 235, 235, 235, 235, 235, 235, 235, 235, 235, 234, 234, 234, 234, 234, 234, 234, 234, 234, 234, 234, 234, 233, 233, 233, 233, 233, 233, 233, 233, 233, 233, 233, 232, 232, 232, 232, 232, 232, 232, 232, 232, 232, 232, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 231, 230, 230, 230, 230, 230, 230, 230, 230, 230, 230, 229, 229, 229, 229, 229, 229, 229, 229, 229, 229, 229, 228, 228, 228, 228, 228, 228, 228, 228, 228, 228, 227, 227, 227, 227, 227, 227, 227, 227, 227, 227, 226, 226, 226, 226, 226, 226, 226, 226, 226, 226, 225, 225, 225, 225, 225, 225, 225, 225, 225, 225, 224, 224, 224, 224, 224, 224, 224, 224, 224, 223, 223, 223, 223, 223, 223, 223, 223, 223, 223, 222, 222, 222, 222, 222, 222, 222, 222, 222, 221, 221, 221, 221, 221, 221, 221, 221, 221, 221, 220, 220, 220, 220, 220, 220, 220, 220, 220, 219, 219, 219, 219, 219, 219, 219, 219, 219, 218, 218, 218, 218, 218, 218, 218, 218, 218, 217, 217, 217, 217, 217, 217, 217, 217, 217, 216, 216, 216, 216, 216, 216, 216, 216, 215, 215, 215, 215, 215, 215, 215, 215, 215, 214, 214, 214, 214, 214, 214, 214, 214, 214, 213, 213, 213, 213, 213, 213, 213, 213, 212, 212, 212, 212, 212, 212, 212, 212, 211, 211, 211, 211, 211, 211, 211, 211, 211, 210, 210, 210, 210, 210, 210, 210, 210, 209, 209, 209, 209, 209, 209, 209, 209, 208, 208, 208, 208, 208, 208, 208, 208, 207, 207, 207, 207, 207, 207, 207, 207, 206, 206, 206, 206, 206, 206, 206, 206, 205, 205, 205, 205, 205, 205, 205, 205, 204, 204, 204, 204, 204, 204, 204, 204, 203, 203, 203, 203, 203, 203, 203, 203, 202, 202, 202, 202, 202, 202, 202, 202, 201, 201, 201, 201, 201, 201, 201, 200, 200, 200, 200, 200, 200, 200, 200, 199, 199, 199, 199, 199, 199, 199, 198, 198, 198, 198, 198, 198, 198, 198, 197, 197, 197, 197, 197, 197, 197, 196, 196, 196, 196, 196, 196, 196, 196, 195, 195, 195, 195, 195, 195, 195, 194, 194, 194, 194, 194, 194, 194, 194, 193, 193, 193, 193, 193, 193, 193, 192, 192, 192, 192, 192, 192, 192, 191, 191, 191, 191, 191, 191, 191, 190, 190, 190, 190, 190, 190, 190, 190, 189, 189, 189, 189, 189, 189, 189, 188, 188, 188, 188, 188, 188, 188, 187, 187, 187, 187, 187, 187, 187, 186, 186, 186, 186, 186, 186, 186, 185, 185, 185, 185, 185, 185, 185, 184, 184, 184, 184, 184, 184, 184, 183, 183, 183, 183, 183, 183, 183, 182, 182, 182, 182, 182, 182, 182, 181, 181, 181, 181, 181, 181, 181, 180, 180, 180, 180, 180, 180, 180, 179, 179, 179, 179, 179, 179, 179, 178, 178, 178, 178, 178, 178, 177, 177, 177, 177, 177, 177, 177, 176, 176, 176, 176, 176, 176, 176, 175, 175, 175, 175, 175, 175, 175, 174, 174, 174, 174, 174, 174, 174, 173, 173, 173, 173, 173, 173, 172, 172, 172, 172, 172, 172, 172, 171, 171, 171, 171, 171, 171, 171, 170, 170, 170, 170, 170, 170, 169, 169, 169, 169, 169, 169, 169, 168, 168, 168, 168, 168, 168, 168, 167, 167, 167, 167, 167, 167, 166, 166, 166, 166, 166, 166, 166, 165, 165, 165, 165, 165, 165, 164, 164, 164, 164, 164, 164, 164, 163, 163, 163, 163, 163, 163, 162, 162, 162, 162, 162, 162, 162, 161, 161, 161, 161, 161, 161, 160, 160, 160, 160, 160, 160, 160, 159, 159, 159, 159, 159, 159, 158, 158, 158, 158, 158, 158, 158, 157, 157, 157, 157, 157, 157, 156, 156, 156, 156, 156, 156, 156, 155, 155, 155, 155, 155, 155, 154, 154, 154, 154, 154, 154, 153, 153, 153, 153, 153, 153, 153, 152, 152, 152, 152, 152, 152, 151, 151, 151, 151, 151, 151, 151, 150, 150, 150, 150, 150, 150, 149, 149, 149, 149, 149, 149, 148, 148, 148, 148, 148, 148, 148, 147, 147, 147, 147, 147, 147, 146, 146, 146, 146, 146, 146, 145, 145, 145, 145, 145, 145, 145, 144, 144, 144, 144, 144, 144, 143, 143, 143, 143, 143, 143, 142, 142, 142, 142, 142, 142, 142, 141, 141, 141, 141, 141, 141, 140, 140, 140, 140, 140, 140, 139, 139, 139, 139, 139, 139, 138, 138, 138, 138, 138, 138, 138, 137, 137, 137, 137, 137, 137, 136, 136, 136, 136, 136, 136, 135, 135, 135, 135, 135, 135, 135, 134, 134, 134, 134, 134, 134, 133, 133, 133, 133, 133, 133, 132, 132, 132, 132, 132, 132, 131, 131, 131, 131, 131, 131, 131, 130, 130, 130, 130, 130, 130, 129, 129, 129, 129, 129, 129, 128, 128, 128, 128, 128, 128, 128, 127, 127, 127, 127, 127, 127, 126, 126, 126, 126, 126, 126, 125, 125, 125, 125, 125, 125, 124, 124, 124, 124, 124, 124, 124, 123, 123, 123, 123, 123, 123, 122, 122, 122, 122, 122, 122, 121, 121, 121, 121, 121, 121, 120, 120, 120, 120, 120, 120, 120, 119, 119, 119, 119, 119, 119, 118, 118, 118, 118, 118, 118, 117, 117, 117, 117, 117, 117, 117, 116, 116, 116, 116, 116, 116, 115, 115, 115, 115, 115, 115, 114, 114, 114, 114, 114, 114, 113, 113, 113, 113, 113, 113, 113, 112, 112, 112, 112, 112, 112, 111, 111, 111, 111, 111, 111, 110, 110, 110, 110, 110, 110, 110, 109, 109, 109, 109, 109, 109, 108, 108, 108, 108, 108, 108, 107, 107, 107, 107, 107, 107, 107, 106, 106, 106, 106, 106, 106, 105, 105, 105, 105, 105, 105, 104, 104, 104, 104, 104, 104, 104, 103, 103, 103, 103, 103, 103, 102, 102, 102, 102, 102, 102, 102, 101, 101, 101, 101, 101, 101, 100, 100, 100, 100, 100, 100, 99, 99, 99, 99, 99, 99, 99, 98, 98, 98, 98, 98, 98, 97, 97, 97, 97, 97, 97, 97, 96, 96, 96, 96, 96, 96, 95, 95, 95, 95, 95, 95, 95, 94, 94, 94, 94, 94, 94, 93, 93, 93, 93, 93, 93, 93, 92, 92, 92, 92, 92, 92, 91, 91, 91, 91, 91, 91, 91, 90, 90, 90, 90, 90, 90, 89, 89, 89, 89, 89, 89, 89, 88, 88, 88, 88, 88, 88, 87, 87, 87, 87, 87, 87, 87, 86, 86, 86, 86, 86, 86, 86, 85, 85, 85, 85, 85, 85, 84, 84, 84, 84, 84, 84, 84, 83, 83, 83, 83, 83, 83, 83, 82, 82, 82, 82, 82, 82, 81, 81, 81, 81, 81, 81, 81, 80, 80, 80, 80, 80, 80, 80, 79, 79, 79, 79, 79, 79, 79, 78, 78, 78, 78, 78, 78, 78, 77, 77, 77, 77, 77, 77, 76, 76, 76, 76, 76, 76, 76, 75, 75, 75, 75, 75, 75, 75, 74, 74, 74, 74, 74, 74, 74, 73, 73, 73, 73, 73, 73, 73, 72, 72, 72, 72, 72, 72, 72, 71, 71, 71, 71, 71, 71, 71, 70, 70, 70, 70, 70, 70, 70, 69, 69, 69, 69, 69, 69, 69, 68, 68, 68, 68, 68, 68, 68, 67, 67, 67, 67, 67, 67, 67, 66, 66, 66, 66, 66, 66, 66, 65, 65, 65, 65, 65, 65, 65, 65, 64, 64, 64, 64, 64, 64, 64, 63, 63, 63, 63, 63, 63, 63, 62, 62, 62, 62, 62, 62, 62, 61, 61, 61, 61, 61, 61, 61, 61, 60, 60, 60, 60, 60, 60, 60, 59, 59, 59, 59, 59, 59, 59, 59, 58, 58, 58, 58, 58, 58, 58, 57, 57, 57, 57, 57, 57, 57, 57, 56, 56, 56, 56, 56, 56, 56, 55, 55, 55, 55, 55, 55, 55, 55, 54, 54, 54, 54, 54, 54, 54, 53, 53, 53, 53, 53, 53, 53, 53, 52, 52, 52, 52, 52, 52, 52, 52, 51, 51, 51, 51, 51, 51, 51, 51, 50, 50, 50, 50, 50, 50, 50, 50, 49, 49, 49, 49, 49, 49, 49, 49, 48, 48, 48, 48, 48, 48, 48, 48, 47, 47, 47, 47, 47, 47, 47, 47, 46, 46, 46, 46, 46, 46, 46, 46, 45, 45, 45, 45, 45, 45, 45, 45, 44, 44, 44, 44, 44, 44, 44, 44, 44, 43, 43, 43, 43, 43, 43, 43, 43, 42, 42, 42, 42, 42, 42, 42, 42, 41, 41, 41, 41, 41, 41, 41, 41, 41, 40, 40, 40, 40, 40, 40, 40, 40, 40, 39, 39, 39, 39, 39, 39, 39, 39, 38, 38, 38, 38, 38, 38, 38, 38, 38, 37, 37, 37, 37, 37, 37, 37, 37, 37, 36, 36, 36, 36, 36, 36, 36, 36, 36, 35, 35, 35, 35, 35, 35, 35, 35, 35, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 33, 33, 33, 33, 33, 33, 33, 33, 33, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 31, 31, 31, 31, 31, 31, 31, 31, 31, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 6, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 7, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 8, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 11, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 14, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 15, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 17, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 18, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 19, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 21, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 22, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 23, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 24, 25, 25, 25, 25, 25, 25, 25, 25, 25, 25, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 26, 27, 27, 27, 27, 27, 27, 27, 27, 27, 27, 28, 28, 28, 28, 28, 28, 28, 28, 28, 28, 29, 29, 29, 29, 29, 29, 29, 29, 29, 29, 30, 30, 30, 30, 30, 30, 30, 30, 30, 30, 31, 31, 31, 31, 31, 31, 31, 31, 31, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 33, 33, 33, 33, 33, 33, 33, 33, 33, 34, 34, 34, 34, 34, 34, 34, 34, 34, 34, 35, 35, 35, 35, 35, 35, 35, 35, 35, 36, 36, 36, 36, 36, 36, 36, 36, 36, 37, 37, 37, 37, 37, 37, 37, 37, 37, 38, 38, 38, 38, 38, 38, 38, 38, 38, 39, 39, 39, 39, 39, 39, 39, 39, 40, 40, 40, 40, 40, 40, 40, 40, 40, 41, 41, 41, 41, 41, 41, 41, 41, 41, 42, 42, 42, 42, 42, 42, 42, 42, 43, 43, 43, 43, 43, 43, 43, 43, 44, 44, 44, 44, 44, 44, 44, 44, 44, 45, 45, 45, 45, 45, 45, 45, 45, 46, 46, 46, 46, 46, 46, 46, 46, 47, 47, 47, 47, 47, 47, 47, 47, 48, 48, 48, 48, 48, 48, 48, 48, 49, 49, 49, 49, 49, 49, 49, 49, 50, 50, 50, 50, 50, 50, 50, 50, 51, 51, 51, 51, 51, 51, 51, 51, 52, 52, 52, 52, 52, 52, 52, 52, 53, 53, 53, 53, 53, 53, 53, 53, 54, 54, 54, 54, 54, 54, 54, 55, 55, 55, 55, 55, 55, 55, 55, 56, 56, 56, 56, 56, 56, 56, 57, 57, 57, 57, 57, 57, 57, 57, 58, 58, 58, 58, 58, 58, 58, 59, 59, 59, 59, 59, 59, 59, 59, 60, 60, 60, 60, 60, 60, 60, 61, 61, 61, 61, 61, 61, 61, 61, 62, 62, 62, 62, 62, 62, 62, 63, 63, 63, 63, 63, 63, 63, 64, 64, 64, 64, 64, 64, 64, 65, 65, 65, 65, 65, 65, 65, 65, 66, 66, 66, 66, 66, 66, 66, 67, 67, 67, 67, 67, 67, 67, 68, 68, 68, 68, 68, 68, 68, 69, 69, 69, 69, 69, 69, 69, 70, 70, 70, 70, 70, 70, 70, 71, 71, 71, 71, 71, 71, 71, 72, 72, 72, 72, 72, 72, 72, 73, 73, 73, 73, 73, 73, 73, 74, 74, 74, 74, 74, 74, 74, 75, 75, 75, 75, 75, 75, 75, 76, 76, 76, 76, 76, 76, 76, 77, 77, 77, 77, 77, 77, 78, 78, 78, 78, 78, 78, 78, 79, 79, 79, 79, 79, 79, 79, 80, 80, 80, 80, 80, 80, 80, 81, 81, 81, 81, 81, 81, 81, 82, 82, 82, 82, 82, 82, 83, 83, 83, 83, 83, 83, 83, 84, 84, 84, 84, 84, 84, 84, 85, 85, 85, 85, 85, 85, 86, 86, 86, 86, 86, 86, 86, 87, 87, 87, 87, 87, 87, 87, 88, 88, 88, 88, 88, 88, 89, 89, 89, 89, 89, 89, 89, 90, 90, 90, 90, 90, 90, 91, 91, 91, 91, 91, 91, 91, 92, 92, 92, 92, 92, 92, 93, 93, 93, 93, 93, 93, 93, 94, 94, 94, 94, 94, 94, 95, 95, 95, 95, 95, 95, 95, 96, 96, 96, 96, 96, 96, 97, 97, 97, 97, 97, 97, 97, 98, 98, 98, 98, 98, 98, 99, 99, 99, 99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 101, 101, 101, 101, 101, 101, 102, 102, 102, 102, 102, 102, 102, 103, 103, 103, 103, 103, 103, 104, 104, 104, 104, 104, 104, 104, 105, 105, 105, 105, 105, 105, 106, 106, 106, 106, 106, 106, 107, 107, 107, 107, 107, 107, 107, 108, 108, 108, 108, 108, 108, 109, 109, 109, 109, 109, 109, 110, 110, 110, 110, 110, 110, 110, 111, 111, 111, 111, 111, 111, 112, 112, 112, 112, 112, 112, 113, 113, 113, 113, 113, 113, 113, 114, 114, 114, 114, 114, 114, 115, 115, 115, 115, 115, 115, 116, 116, 116, 116, 116, 116, 117, 117, 117, 117, 117, 117, 117, 118, 118, 118, 118, 118, 118, 119, 119, 119, 119, 119, 119, 120, 120, 120, 120, 120, 120, 120, 121, 121, 121, 121, 121, 121, 122, 122, 122, 122, 122, 122, 123, 123, 123, 123, 123, 123, 124, 124, 124, 124, 124, 124, 124, 125, 125, 125, 125, 125, 125, 126, 126, 126, 126, 126, 126, 127, 127, 127, 127, 127, 127};
volatile uint32_t knob0Rotation = 16;
volatile uint32_t knob1Rotation = 16;
volatile uint32_t knob2Rotation = 16;
volatile uint32_t knob3Rotation = 16;

volatile char noteMessage[] = "xxx";


// -----------------------------------------------
// ------------------ Mutexes --------------------
// -----------------------------------------------

QueueHandle_t msgOutQ;
SemaphoreHandle_t polyArrayMutex;
SemaphoreHandle_t keyArrayMutex;
SemaphoreHandle_t wetMutex;//cannot atomically access wet as it is a float so decided to mutex it instead, not used in ISR so this works
SemaphoreHandle_t KnobThreadSafeMutex;
SemaphoreHandle_t doubleBufferOneSemaphore;
SemaphoreHandle_t doubleBufferTwoSemaphore;

//Display driver object
U8G2_SSD1305_128X32_NONAME_F_HW_I2C u8g2(U8G2_R0);

//Function to set outputs via matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, bitIdx & 0x01);
  digitalWrite(RA1_PIN, bitIdx & 0x02);
  digitalWrite(RA2_PIN, bitIdx & 0x04);
  digitalWrite(OUT_PIN, value);
  digitalWrite(REN_PIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(REN_PIN, LOW);
}


// -----------------------------------------------
// ----------- Schroeder Reverberator ------------
// -----------------------------------------------
// start of Schroeder reverberator implementation
// Source: https://github.com/YetAnotherElectronicsChannel/STM32_DSP_Reverb/blob/master/code/Src/main.c
//Filter buffer sizes limited by amount of RAM
#define l_CB0 3460/2
#define l_CB1 2988/2
#define l_CB2 3882/2
#define l_CB3 4312/2
#define l_AP0 480/2
#define l_AP1 161/2
#define l_AP2 46/2

//wet and tim are the only two variables in the Schroeder reverb that will be accessed by multiple threads
//as they are user-configurable, everything else is a black box to all but the caller of Do_Reverb
//define wet 0.0 <-> 1.0
volatile float wet = 1.0f;
//define time delay 0.0 <-> 1.0 (max)
volatile float tim = 1.0f;

//define pointer limits = delay time
int cf0_lim;
int cf1_lim;
int cf2_lim;
int cf3_lim;
int ap0_lim;
int ap1_lim;
int ap2_lim;

//define buffer for comb- and allpassfilters
float cfbuf0[l_CB0];
float cfbuf1[l_CB1];
float cfbuf2[l_CB2];
float cfbuf3[l_CB3];
float apbuf0[l_AP0];
float apbuf1[l_AP1];
float apbuf2[l_AP2];

//feedback defines as of Schroeder
float cf0_g = 0.805f, cf1_g = 0.827f, cf2_g = 0.783f, cf3_g = 0.764f;
float ap0_g = 0.7f, ap1_g = 0.7f, ap2_g = 0.7f;

//buffer-pointer
volatile int cf0_p = 0;
int cf1_p = 0;
int cf2_p = 0;
int cf3_p = 0;
int ap0_p = 0;
int ap1_p = 0;
int ap2_p = 0;
uint16_t rxBuf[8];
uint16_t txBuf[8];

inline float Do_Comb0(float inSample) {
  //Serial.println(cfbuf0[cf0_p]);
  float readback = cfbuf0[cf0_p];
  //Serial.println(cfbuf0[cf0_p]);
  float newSample = readback * cf0_g + inSample;
  cfbuf0[cf0_p] = newSample;
  cf0_p++;
  if (cf0_p == cf0_lim) cf0_p = 0;
  return readback;
}
inline float Do_Comb1(float inSample) {
  float readback = cfbuf1[cf1_p];
  float newSample = readback * cf1_g + inSample;
  cfbuf1[cf1_p] = newSample;
  cf1_p++;
  if (cf1_p == cf1_lim) cf1_p = 0;
  return readback;
}
inline float Do_Comb2(float inSample) {
  float readback = cfbuf2[cf2_p];
  float newSample = readback * cf2_g + inSample;
  cfbuf2[cf2_p] = newSample;
  cf2_p++;
  if (cf2_p == cf2_lim) cf2_p = 0;
  return readback;
}
inline float Do_Comb3(float inSample) {
  float readback = cfbuf3[cf3_p];
  float newSample = readback * cf3_g + inSample;
  cfbuf3[cf3_p] = newSample;
  cf3_p++;
  if (cf3_p == cf3_lim) cf3_p = 0;
  return readback;
}
inline float Do_Allpass0(float inSample) {
  float readback = apbuf0[ap0_p];
  readback += (-ap0_g) * inSample;
  float newSample = readback * ap0_g + inSample;
  apbuf0[ap0_p] = newSample;
  ap0_p++;
  if (ap0_p == ap0_lim) ap0_p = 0;
  return readback;
}
inline float Do_Allpass1(float inSample) {
  float readback = apbuf1[ap1_p];
  readback += (-ap1_g) * inSample;
  float newSample = readback * ap1_g + inSample;
  apbuf1[ap1_p] = newSample;
  ap1_p++;
  if (ap1_p == ap1_lim) ap1_p = 0;
  return readback;
}
inline float Do_Allpass2(float inSample) {
  float readback = apbuf2[ap2_p];
  readback += (-ap2_g) * inSample;
  float newSample = readback * ap2_g + inSample;
  apbuf2[ap2_p] = newSample;
  ap2_p++;
  if (ap2_p == ap2_lim) ap2_p = 0;
  return readback;
}
inline float Do_Reverb(float inSample) {
  float newsample = (Do_Comb0(inSample) + Do_Comb1(inSample) + Do_Comb2(inSample) + Do_Comb3(inSample)) / 4.0f;
  newsample = Do_Allpass0(newsample);
  newsample = Do_Allpass1(newsample);
  newsample = Do_Allpass2(newsample);
  return newsample;
}


// -----------------------------------------------
// ------------- Knob Decoder Class --------------
// -----------------------------------------------
class Knob
{
    //In the current state of the code, Knob instances are only accessed by one thread, however, the class was made thread-safe
    //as this may be changed in the future as new features are added.
  private:
    uint8_t knobNumber;
    uint8_t amount;
    bool add;
    bool prevA;
    bool prevB;
    bool prevWasIncrement;//not sure if these need to be mutexed to make function reentrant
  public:
    Knob(uint8_t knobNumber_) {
      xSemaphoreTake(KnobThreadSafeMutex, portMAX_DELAY);
      knobNumber = knobNumber_;
      prevA = false;
      prevB = false;
      prevWasIncrement = false;
      xSemaphoreGive(KnobThreadSafeMutex);
    }

    inline void setKnobRotationAtomic(uint8_t* keyArrayLocal) {
      xSemaphoreTake(KnobThreadSafeMutex, portMAX_DELAY);//the mutex is taken at the beginning of the function and released at the end,
      //for now, this is more efficient than repeatedly locking and unlocking whenever a member vriable is accessed since we currently
      //do not use an instance of this class from different threads so will not benefit from allowing concurrency when it can be done safely.
      bool curA;
      bool curB;
      switch (knobNumber) {
        case 0:
          curA = (keyArrayLocal[4] >> 2 & 0b1);
          curB = (keyArrayLocal[4] >> 3 & 0b1);
          break;
        case 1:
          curA = (keyArrayLocal[4]  & 0b1);
          curB = (keyArrayLocal[4] >> 1 & 0b1);
          break;
        case 2:
          curA = (keyArrayLocal[3] >> 2 & 0b1);
          curB = (keyArrayLocal[3] >> 3 & 0b1);
          break;
        case 3:
          //Serial.println("HERE");
          curA = (keyArrayLocal[3] & 0b1);
          curB = (keyArrayLocal[3] >> 1 & 0b1);
          break;
      }
      amount = 0;//essential so no modifications are made when the knobs aren't moved
      if (prevA == 0 && prevB == 0) {
        if (curA == 1 && curB == 0) {
          amount = 1;
          add = true;
          prevWasIncrement = true;
        }
        if (curA == 0 && curB == 1) {
          amount = 1;
          add = false;
          prevWasIncrement = false;
        }
        if (curA == 1 && curB == 1) {
          if (prevWasIncrement) {
            amount = 2;
            add = true;
          } else {
            amount = 2;
            add = false;
          }
        }
      } else if (prevA == 0 && prevB == 1) {
        if (curA == 0 && curB == 0) {
          amount = 1;
          add = true;
          prevWasIncrement = true;
        }
        if (curA == 1 && curB == 1) {
          amount = 1;
          add = false;
          prevWasIncrement = false;
        }
        if (curA == 1 && curB == 0) {
          if (prevWasIncrement) {
            amount = 2;
            add = true;
          } else {
            amount = 2;
            add = false;
          }
        }
      } else if (prevA == 1 && prevB == 0) {
        if (curA == 0 && curB == 0) {
          amount = 1;
          add = false;
          prevWasIncrement = false;
        }
        if (curA == 1 && curB == 1) {
          amount = 1;
          add = true;
          prevWasIncrement = true;
        }
        if (curA == 0 && curB == 1) {
          if (prevWasIncrement) {
            amount = 2;
            add = true;
          } else {
            amount = 2;
            add = false;
          }
        }
      } else if (prevA == 1 && prevB == 1) {
        if (curA == 1 && curB == 0) {
          amount = 1;
          add = false;
          prevWasIncrement = false;
        }
        if (curA == 0 && curB == 1) {
          amount = 1;
          add = true;
          prevWasIncrement = true;
        }
        if (curA == 0 && curB == 0) {
          if (prevWasIncrement) {
            amount = 2;
            add = true;
          } else {
            amount = 2;
            add = false;
          }
        }
      }
      if (amount != 0) {
        switch (knobNumber) {
          case 0:
            {
              uint32_t knobRotationLocal = __atomic_load_n(&knob0Rotation, __ATOMIC_RELAXED);
              if (add) {
                knobRotationLocal = (knobRotationLocal + amount) % 17;
              } else {
                knobRotationLocal = (knobRotationLocal - amount) % 17;
              }
              __atomic_store_n(&knob0Rotation, knobRotationLocal, __ATOMIC_RELAXED);
              break;
            }
          case 1:
            {
              uint32_t knobRotationLocal = __atomic_load_n(&knob1Rotation, __ATOMIC_RELAXED);
              if (add) {
                knobRotationLocal = (knobRotationLocal + amount) % 17;
              } else {
                knobRotationLocal = (knobRotationLocal - amount) % 17;
              }
              __atomic_store_n(&knob1Rotation, knobRotationLocal, __ATOMIC_RELAXED);
              break;
            }
          case 2:
            {
              uint32_t knobRotationLocal = __atomic_load_n(&knob2Rotation, __ATOMIC_RELAXED);
              if (add) {
                knobRotationLocal = (knobRotationLocal + amount) % 17;
              } else {
                knobRotationLocal = (knobRotationLocal - amount) % 17;
              }
              __atomic_store_n(&knob2Rotation, knobRotationLocal, __ATOMIC_RELAXED);
              break;
            }
          case 3:
            {
              uint32_t knobRotationLocal = __atomic_load_n(&knob3Rotation, __ATOMIC_RELAXED);
              if (add) {
                knobRotationLocal = (knobRotationLocal + amount) % 17;
              } else {
                knobRotationLocal = (knobRotationLocal - amount) % 17;
              }
              __atomic_store_n(&knob3Rotation, knobRotationLocal, __ATOMIC_RELAXED);
              break;
            }

        }
      }
      prevA = curA;
      prevB = curB;
      xSemaphoreGive(KnobThreadSafeMutex);
    }
};


// -----------------------------------------------
// ---------- Polyphony Implementation -----------
// -----------------------------------------------
inline void polyphonise(uint8_t& currentSample, uint8_t waveFormLocal) {
  static uint32_t currentSineSample = 0;
  static uint32_t shiftCounter = 0;
  static uint8_t polyTurn = 0;
  static uint32_t phaseAcc = 0;

  // counter to switch between keys
  shiftCounter++;
  if (shiftCounter % 750 == 0) {
    polyTurn++;
  }

  // Sine wave Polyphony
  if (waveFormLocal == SINE) {
    uint32_t sineCurrentStepSize_0Local = __atomic_load_n(&sineCurrentStepSize_0, __ATOMIC_RELAXED);
    uint32_t sineCurrentStepSize_1Local = __atomic_load_n(&sineCurrentStepSize_1, __ATOMIC_RELAXED);
    uint32_t sineCurrentStepSize_2Local = __atomic_load_n(&sineCurrentStepSize_2, __ATOMIC_RELAXED);

    // find number of keys in play
    int numKeysInPlay = 0;
    if (sineCurrentStepSize_2Local != 0) {
      numKeysInPlay = 3;
    }
    else if (sineCurrentStepSize_1Local != 0) {
      numKeysInPlay = 2;
    }
    else if (sineCurrentStepSize_0Local != 0) {
      numKeysInPlay = 1;
    }

    currentSample = 0;

    // increment sine sample by polyphony key
    if (numKeysInPlay == 1) {
      currentSineSample += sineCurrentStepSize_0Local;
    }
    if (numKeysInPlay == 2) {
      if (polyTurn % 2 == 0) {
        // synchronize all phases
        currentSineSample += sineCurrentStepSize_0Local;
      }
      else if (polyTurn % 2 == 1 && sineCurrentStepSize_1Local != 0) {
        // synchronize all phases
        currentSineSample += sineCurrentStepSize_1Local;
      }
    }
    else if (numKeysInPlay == 3) {
      if (polyTurn % 3 == 0) {
        // synchronize all phases
        currentSineSample += sineCurrentStepSize_0Local;
      }
      else if (polyTurn % 3 == 1 && sineCurrentStepSize_1Local != 0) {
        // synchronize all phases
        currentSineSample += sineCurrentStepSize_1Local;
      }
      else if (polyTurn % 3 == 2 && sineCurrentStepSize_2Local != 0) {
        // synchronize all phases
        currentSineSample += sineCurrentStepSize_2Local;
      }
    }
    if (currentSineSample >= 5000) {
      currentSineSample = 0;
    }
    //write sine sample to currentSample
    currentSample = sineTable[currentSineSample];
  }

  // Sawtooth wave Polyphony
  else if (waveFormLocal == SAWTOOTH) {
    uint32_t currentStepSize_0Local = __atomic_load_n(&currentStepSize_0, __ATOMIC_RELAXED);
    uint32_t currentStepSize_1Local = __atomic_load_n(&currentStepSize_1, __ATOMIC_RELAXED);
    uint32_t currentStepSize_2Local = __atomic_load_n(&currentStepSize_2, __ATOMIC_RELAXED);

    // find number of keys in play
    int numKeysInPlay = 0;
    if (currentStepSize_2Local != 0) {
      numKeysInPlay = 3;
    }
    else if (currentStepSize_1Local != 0) {
      numKeysInPlay = 2;
    }
    else if (currentStepSize_0Local != 0) {
      numKeysInPlay = 1;
    }

    currentSample = 0; //current sample is the name of outValue within this task

    // increment phase accumulator by polyphony key
    if (numKeysInPlay == 1) {
      phaseAcc += currentStepSize_0Local;
    }
    else if (numKeysInPlay == 2) {
      if (polyTurn % 2 == 0) {
        // synchronize all phases
        phaseAcc += currentStepSize_0Local;
      }
      else if (polyTurn % 2 == 1 && currentStepSize_1Local != 0) {
        // synchronize all phases
        phaseAcc += currentStepSize_1Local;
      }
    }
    else if (numKeysInPlay == 3) {
      if (polyTurn % 3 == 0) {
        // synchronize all phases
        phaseAcc += currentStepSize_0Local;
      }
      else if (polyTurn % 3 == 1 && currentStepSize_1Local != 0) {
        // synchronize all phases
        phaseAcc += currentStepSize_1Local;
      }
      else if (polyTurn % 3 == 2 && currentStepSize_2Local != 0) {
        // synchronize all phases
        phaseAcc += currentStepSize_2Local;
      }
    }
    currentSample = phaseAcc >> 24;//write sawtooth sample to currentSample
  }
}


// -----------------------------------------------
// ---------------- Apply Reverb -----------------
// -----------------------------------------------
inline void reverbise(uint8_t& currentSample) {
  currentSample = currentSample >> 2;
  float currentSampleFloat = (float)currentSample;
  xSemaphoreTake(wetMutex, portMAX_DELAY);
  currentSampleFloat = (1.0f - wet) * currentSampleFloat + wet * Do_Reverb(currentSampleFloat);
  xSemaphoreGive(wetMutex);
  currentSample = (uint8_t)currentSampleFloat;
}


// -----------------------------------------------
// --------------- JoyStick Sample ---------------
// -----------------------------------------------
inline void getJoystickSample(uint8_t& currentSample) {
  static uint32_t joystickModeCounter = 0;
  static uint8_t squareWave = 0b11111111;
  static bool squareWaveHigh = true;

  joystickModeCounter++;
  uint32_t compareValue;
  if (squareWaveHigh) {
    compareValue = __atomic_load_n(&highLimit, __ATOMIC_RELAXED);
  } else {
    compareValue = __atomic_load_n(&lowLimit, __ATOMIC_RELAXED);
  }
  if (joystickModeCounter >= compareValue) {
    joystickModeCounter = 0;
    squareWaveHigh = !squareWaveHigh;
    squareWave = (squareWave ^ 0b11111111);
  }
  currentSample = squareWave;
}


// -----------------------------------------------
// ----------- Write Sample to Buffer ------------
// -----------------------------------------------
inline void writeSampleToBuffer(bool bufferToWrite) {
  uint8_t currentSample;
  bool joyStickModeLocal = __atomic_load_n(&joyStickMode, __ATOMIC_RELAXED);
  bool reverbSetLocal = __atomic_load_n(&reverbSet, __ATOMIC_RELAXED);
  if (joyStickModeLocal) {
    for (int i = 0; i < 220; i++) {
      getJoystickSample(currentSample);
      if (reverbSetLocal) {
        reverbise(currentSample);
      }
      if (bufferToWrite == BUFFER_ONE) {
        bufferOne[i] = currentSample;//no need to protect accesses to buffers one and two since the semaphores in the calling function (with the ISR giving the semaphores) ensure that this thread is the only one accessing whatever buffer we write to (the ISR would be acessing/reading from the other)
      } else {
        bufferTwo[i] = currentSample;
      }
    }
  } else {
    uint8_t waveFormLocal = __atomic_load_n(&waveForm, __ATOMIC_RELAXED);
    if (waveFormLocal == SINE) {
      for (int i = 0; i < 220; i++) {
        polyphonise(currentSample, SINE);
        if (reverbSetLocal) {
          reverbise(currentSample);
        }
        if (bufferToWrite == BUFFER_ONE) {
          bufferOne[i] = currentSample;
        } else {
          bufferTwo[i] = currentSample;
        }
      }
    } else if (waveFormLocal == SAWTOOTH) {
      for (int i = 0; i < 220; i++) {
        polyphonise(currentSample, SAWTOOTH);

        if (reverbSetLocal) {
          reverbise(currentSample);
        }
        if (bufferToWrite == BUFFER_ONE) {
          bufferOne[i] = currentSample;
        } else {
          bufferTwo[i] = currentSample;
        }
      }
    }
  }
}

void sampleWriteTask(void * pvParameters) {
#ifndef PROFILE_SAMPLE_WRITE_TASK
  while (1) {//not running on a timer but effectively is timed by the ISR releasing the semaphore for it to run
    if ( xSemaphoreTake( doubleBufferTwoSemaphore, SEMAPHORE_MAX_WAIT_TIME ) == pdTRUE ) {
      writeSampleToBuffer(BUFFER_TWO);

    }
    if ( xSemaphoreTake( doubleBufferOneSemaphore, SEMAPHORE_MAX_WAIT_TIME ) == pdTRUE ) {
      writeSampleToBuffer(BUFFER_ONE);
    }
  }
#endif
#ifdef PROFILE_SAMPLE_WRITE_TASK
  // Simulation: start of worst-case configuration
  // simulate first three keys pressed to force worst-case execution time in polyphonise
  __atomic_store_n(&sineCurrentStepSize_0, sineStepSizes[0], __ATOMIC_RELAXED);
  __atomic_store_n(&sineCurrentStepSize_1, sineStepSizes[1], __ATOMIC_RELAXED);
  __atomic_store_n(&sineCurrentStepSize_2, sineStepSizes[2], __ATOMIC_RELAXED);
  __atomic_store_n(&joyStickMode, false, __ATOMIC_RELAXED);
  __atomic_store_n(&reverbSet, true, __ATOMIC_RELAXED);//setting this to false will reduce execution time by more than 10 time indicating how resource intensive the large number of floating point operations are.
  // end of worst-case configuration

  uint32_t startTime = micros();
  for (int i = 0; i < 32; i++) {
    if ( /*xSemaphoreTake( doubleBufferTwoSemaphore, SEMAPHORE_MAX_WAIT_TIME ) == pdTRUE */true) {
      writeSampleToBuffer(BUFFER_TWO);
    }
    if ( /*xSemaphoreTake( doubleBufferOneSemaphore, SEMAPHORE_MAX_WAIT_TIME ) == pdTRUE*/ false) {//the worst-case won't involve writing to two buffers and both paths here are identical in execution time
      writeSampleToBuffer(BUFFER_ONE);
    }
  }
  uint32_t timeTaken = micros() - startTime;
  Serial.println(timeTaken);
  while (1) {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
#endif
}


// -----------------------------------------------
// ----------------- Sample ISR ------------------
// -----------------------------------------------
void sampleISR() {
  /*reads values from one of two buffers following a pingpong buffer/double buffer scheme where the
    ISR reads from one buffer while the sample writing task writes to the other. Each buffer has an
    associated semaphore which is released by the ISR after it has finished to indicate to sampleWriteTask
    that it can write to it.*/
#ifndef PROFILE_ISR
  static int bufferIndex = 0;
  static bool bufferToRead = BUFFER_TWO;
  uint8_t outValue;
  if (bufferToRead == BUFFER_ONE) {
    outValue = bufferOne[bufferIndex];
    bufferIndex++;
    if (bufferIndex == 219) {
      bufferIndex = 0;
      bufferToRead = BUFFER_TWO;
      xSemaphoreGiveFromISR( doubleBufferOneSemaphore, NULL );
    }
  } else {
    outValue = bufferTwo[bufferIndex];
    bufferIndex++;
    if (bufferIndex == 219) {
      bufferIndex = 0;
      bufferToRead = BUFFER_ONE;
      xSemaphoreGiveFromISR( doubleBufferTwoSemaphore, NULL );
    }
  }
  outValue = outValue >> (8 - knob3Rotation / 2);
  analogWrite(OUTR_PIN, outValue);
#endif
#ifdef PROFILE_ISR
  static int bufferIndex = 0;
  static bool bufferToRead = BUFFER_TWO;
  long startTime = micros();
  for (int i = 0; i < 32; i++) {
    uint8_t outValue;
    if (bufferToRead == BUFFER_ONE) {
      outValue = bufferOne[bufferIndex];
      bufferIndex++;
      if (true) {
        bufferIndex = 0;
        bufferToRead = BUFFER_TWO;
        xSemaphoreGiveFromISR( doubleBufferOneSemaphore, NULL );
      }
    } else {
      outValue = bufferTwo[bufferIndex];
      bufferIndex++;
      if (true) {
        bufferIndex = 0;
        bufferToRead = BUFFER_ONE;
        xSemaphoreGiveFromISR( doubleBufferTwoSemaphore, NULL );
      }
    }
    outValue = outValue >> (8 - knob3Rotation / 2);
    analogWrite(OUTR_PIN, outValue);
  }
  Serial.println(micros() - startTime);
  sampleTimer->pause();
#endif
}

// Read Columns of Switch Matrix
inline uint8_t readCols() {
  digitalWrite(REN_PIN, HIGH);
  uint8_t cols = digitalRead(C3_PIN) << 3 | digitalRead(C2_PIN) << 2 | digitalRead(C1_PIN) << 1 | digitalRead(C0_PIN);
  return cols;
}

// Select Row of Switch Matrix
inline void setRow(uint8_t rowIdx) {
  digitalWrite(REN_PIN, LOW);
  uint8_t RA0_ASSERTION = LOW;
  uint8_t RA1_ASSERTION = LOW;
  uint8_t RA2_ASSERTION = LOW;
  if (rowIdx & 0b001)
    RA0_ASSERTION = HIGH;
  if (rowIdx & 0b010)
    RA1_ASSERTION = HIGH;
  if (rowIdx & 0b100)
    RA2_ASSERTION = HIGH;
  digitalWrite(RA0_PIN, RA0_ASSERTION);
  digitalWrite(RA1_PIN, RA1_ASSERTION);
  digitalWrite(RA2_PIN, RA2_ASSERTION);
  digitalWrite(REN_PIN, HIGH);
}


// -----------------------------------------------
// ---------------- MsgOut Task ------------------
// -----------------------------------------------
void msgOutTask(void *pvParameters) {
  char outMsg[4];
#ifndef PROFILE_MSG_OUT_TASK
  while (1) {
    xQueueReceive(msgOutQ, outMsg, portMAX_DELAY);
    Serial.println(outMsg);
  }
#endif
#ifdef PROFILE_MSG_OUT_TASK
  uint32_t timeTakenTotal = 0;
  for (int iter = 0; iter < 32; iter++) {
    //start of worst-case configuration code
    for (int i = 0; i < 8; i++) {
      xQueueSend( msgOutQ, (char*) noteMessage, portMAX_DELAY);
    }
    //end of worst-case configuration code, note that this must be within the loop because the queue capacity is 8 (hence the worst-case) so only 8 at a time can be written to the queue
    uint32_t startTime = micros();
    for (int i = 0; i < 8; i ++) {
      xQueueReceive(msgOutQ, outMsg, portMAX_DELAY);
      Serial.println(outMsg);
    }
    uint32_t timeTaken = micros() - startTime;
    timeTakenTotal = timeTakenTotal + timeTaken;
  }
  Serial.println(timeTakenTotal);
#endif
}

// Helper function for octave shifting from base 4 octave
inline int8_t getOctaveShift(char octaveAsChar) {
  int8_t octaveShift;
  switch (octaveAsChar) {
    case '0':
      octaveShift = -4;
      break;
    case '1':
      octaveShift = -3;
      break;
    case '2':
      octaveShift = -2;
      break;
    case '3':
      octaveShift = -1;
      break;
    case '4':
      octaveShift = 0;
      break;
    case '5':
      octaveShift = 1;
      break;
    case '6':
      octaveShift = 2;
      break;
    case '7':
      octaveShift = 3;
      break;
    case '8':
      octaveShift = 4;
      break;

  }
  return octaveShift;
}

// Set polyphony step sizes into volatile global variables
inline void setStepSizeAtomic(uint8_t stepIndex, int8_t octaveShift, int8_t polyIdx) {
  if (octaveShift >= 0) {
    // search for correct polyphony step size
    if (polyIdx == 0) {
      __atomic_store_n(&currentStepSize_0, stepSizes[stepIndex] << octaveShift, __ATOMIC_RELAXED);
    }
    else if (polyIdx == 1) {
      __atomic_store_n(&currentStepSize_1, stepSizes[stepIndex] << octaveShift, __ATOMIC_RELAXED);
    }
    else if (polyIdx == 2) {
      __atomic_store_n(&currentStepSize_2, stepSizes[stepIndex] << octaveShift, __ATOMIC_RELAXED);
    }
  } else {
    // search for correct polyphony step size
    if (polyIdx == 0) {
      __atomic_store_n(&currentStepSize_0, stepSizes[stepIndex] >> (-octaveShift), __ATOMIC_RELAXED);
    }
    else if (polyIdx == 1) {
      __atomic_store_n(&currentStepSize_1, stepSizes[stepIndex] >> (-octaveShift), __ATOMIC_RELAXED);
    }
    else if (polyIdx == 2) {
      __atomic_store_n(&currentStepSize_2, stepSizes[stepIndex] >> (-octaveShift), __ATOMIC_RELAXED);
    }
  }
}

// Helper function to convert hexadecimal digits to int
inline int hexToInt(char hexInput) {
  switch (hexInput) {
    case '0':
      return 0;
    case '1':
      return 1;
    case '2':
      return 2;
    case '3':
      return 3;
    case '4':
      return 4;
    case '5':
      return 5;
    case '6':
      return 6;
    case '7':
      return 7;
    case '8':
      return 8;
    case '9':
      return 9;
    case 'A':
      return 10;
    case 'B':
      return 11;
  }
}

// -----------------------------------------------
// ----------------- MsgIn Task ------------------
// -----------------------------------------------
void msgInTask(void *pvParameters) {
  const TickType_t xFrequency = 5 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  char inMsg[] = "xxx";

#ifndef PROFILE_MSG_IN_TASK
  while (1) {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    while ( Serial.available() > 0) {
      uint8_t i = 0;
      while (1) {//loop will be broken out of once a newline character is encountered
        char curChar = Serial.read();
        if (curChar == '\n') {
          break;
        } else {
          inMsg[i] = curChar;
        }
        i++;
      }
      // Handle Remove Key Message
      if (inMsg[0] == 'R') {
        xSemaphoreTake(polyArrayMutex, portMAX_DELAY);
        // remove note from polyphony list
        int removeNoteIdx = 100; // set to arbitrary index above 2
        for (int i = 0; i < 3; i++) {
          if (polyFrequencies[i] == hexToInt(inMsg[2]) + 1) {
            // check which polyphony slot this belongs to and reset to 0
            removeNoteIdx = i;
          }
        }
        // shift step sizes & polyFrequencies
        if (removeNoteIdx == 0) {
          __atomic_store_n(&currentStepSize_0, currentStepSize_1, __ATOMIC_RELAXED);
          __atomic_store_n(&currentStepSize_1, currentStepSize_2, __ATOMIC_RELAXED);
          __atomic_store_n(&currentStepSize_2, 0, __ATOMIC_RELAXED);
          polyFrequencies[0] = polyFrequencies[1];
          polyFrequencies[1] = polyFrequencies[2];
          polyFrequencies[2] = NULL;
        }
        else if (removeNoteIdx == 1) {
          __atomic_store_n(&currentStepSize_1, currentStepSize_2, __ATOMIC_RELAXED);
          __atomic_store_n(&currentStepSize_2, 0, __ATOMIC_RELAXED);
          polyFrequencies[1] = polyFrequencies[2];
          polyFrequencies[2] = NULL;
        }
        else if (removeNoteIdx == 2) {
          __atomic_store_n(&currentStepSize_2, 0, __ATOMIC_RELAXED);
          polyFrequencies[2] = NULL;
        }
        xSemaphoreGive(polyArrayMutex);
      }
      // Handle Press Key Message
      else if (inMsg[0] == 'P') {
        int8_t octaveShift = getOctaveShift(inMsg[1]);
        xSemaphoreTake(polyArrayMutex, portMAX_DELAY);
        // Search for empty polyphony slot
        bool foundPolySlot = false;
        int polySlot = 0;
        for (int i = 0; i < 3; i++) {
          if (polyFrequencies[i] == hexToInt(inMsg[2]) + 1) {
            // don't add key if it already exists
            break;
          }
          if (polyFrequencies[i] == NULL) {
            polyFrequencies[i] = hexToInt(inMsg[2]) + 1;
            foundPolySlot = true;
            polySlot = i;
            break;
          }
        }
        xSemaphoreGive(polyArrayMutex);
        // only set new step size if slot found
        if (foundPolySlot) {
          int noteNumber = hexToInt(inMsg[2]);
          setStepSizeAtomic(noteNumber, octaveShift, polySlot);
        }
      }
    }
  }
#endif
#ifdef PROFILE_MSG_IN_TASK
  uint32_t val;
  for (int i = 0; i < 32; i++) {
    //insert full buffer
    while(!Serial.available()){
    //do nothing, here the user provides a long string of 64 characters to simulate worst-case execution  
    }
    long startTime = micros();
    while ( Serial.available()) {
      uint8_t i = 0;
      while (1) {//loop will be broken out of once a newline character is encountered
        char curChar = Serial.read();
        if (curChar == '\n') {
          break;
        } else {
          inMsg[i] = curChar;
        }
        i++;
      }
      // Handle Remove Key Message
      if (inMsg[0] == 'R') {
        xSemaphoreTake(polyArrayMutex, portMAX_DELAY);
        // remove note from polyphony list
        int removeNoteIdx = 100; // set to arbitrary index above 2
        for (int i = 0; i < 3; i++) {
          if (polyFrequencies[i] == hexToInt(inMsg[2]) + 1) {
            // check which polyphony slot this belongs to and reset to 0
            removeNoteIdx = i;
          }
        }
        // shift step sizes & polyFrequencies
        if (removeNoteIdx == 0) {
          __atomic_store_n(&currentStepSize_0, currentStepSize_1, __ATOMIC_RELAXED);
          __atomic_store_n(&currentStepSize_1, currentStepSize_2, __ATOMIC_RELAXED);
          __atomic_store_n(&currentStepSize_2, 0, __ATOMIC_RELAXED);
          polyFrequencies[0] = polyFrequencies[1];
          polyFrequencies[1] = polyFrequencies[2];
          polyFrequencies[2] = NULL;
        }
        else if (removeNoteIdx == 1) {
          __atomic_store_n(&currentStepSize_1, currentStepSize_2, __ATOMIC_RELAXED);
          __atomic_store_n(&currentStepSize_2, 0, __ATOMIC_RELAXED);
          polyFrequencies[1] = polyFrequencies[2];
          polyFrequencies[2] = NULL;
        }
        else if (removeNoteIdx == 2) {
          __atomic_store_n(&currentStepSize_2, 0, __ATOMIC_RELAXED);
          polyFrequencies[2] = NULL;
        }
        xSemaphoreGive(polyArrayMutex);
      }
      // Handle Press Key Message
      else if (inMsg[0] == 'P') {
        int8_t octaveShift = getOctaveShift(inMsg[1]);
        xSemaphoreTake(polyArrayMutex, portMAX_DELAY);
        // Search for empty polyphony slot
        bool foundPolySlot = false;
        int polySlot = 0;
        for (int i = 0; i < 3; i++) {
          if (polyFrequencies[i] == hexToInt(inMsg[2]) + 1) {
            // don't add key if it already exists
            break;
          }
          if (polyFrequencies[i] == NULL) {
            polyFrequencies[i] = hexToInt(inMsg[2]) + 1;
            foundPolySlot = true;
            polySlot = i;
            break;
          }
        }
        xSemaphoreGive(polyArrayMutex);
        // only set new step size if slot found
        if (foundPolySlot) {
          int noteNumber = hexToInt(inMsg[2]);
          setStepSizeAtomic(noteNumber, octaveShift, polySlot);
        }
      }
    }
    long endTime = micros() - startTime;
    Serial.println(endTime);//this is different from the others in that this is just for one run, need to repeat 32 times

  }
  uint32_t timeTaken = micros() - startTime;
  Serial.println(timeTaken);

#endif
}


// -----------------------------------------------
// ------------- Fill Message Queue --------------
// -----------------------------------------------
inline bool fillMessageQueue(uint8_t* keyArrayLocal) { //returns true if any messages were sent/there was a change in the keyarray

  static uint8_t prevKeyArray[7] = {0b11110000, 0b11110000, 0b11111111, 1, 1, 1, 1};//this says that 8 keys are pressed which fills up the queue, useful for testing worst case execution a
  bool changeInKeys = false;
  for (int rowIdx = 0; rowIdx < 3; rowIdx++) {
    uint32_t curRowIdx = rowIdx << 2;

    uint8_t keyOne = keyArrayLocal[rowIdx] & 0b1;
    uint8_t keyTwo = keyArrayLocal[rowIdx] >> 1 & 0b1;
    uint8_t keyThree = keyArrayLocal[rowIdx] >> 2 & 0b1;
    uint8_t keyFour = keyArrayLocal[rowIdx] >> 3 & 0b1;

    uint8_t prevKeyOne = prevKeyArray[rowIdx] & 0b1;
    uint8_t prevKeyTwo = prevKeyArray[rowIdx] >> 1 & 0b1;
    uint8_t prevKeyThree = prevKeyArray[rowIdx] >> 2 & 0b1;
    uint8_t prevKeyFour = prevKeyArray[rowIdx] >> 3 & 0b1;

    if (keyOne != prevKeyOne) {
      changeInKeys = true;
      if (prevKeyOne == 0b1) {
        noteMessage[0] = 'P';//accesses to noteMessage are protected by the key array mutex
      } else {
        noteMessage[0] = 'R';
      }
      noteMessage[1] = '4';
      noteMessage[2] = intToHex[curRowIdx];
      xQueueSend( msgOutQ, (char*) noteMessage, portMAX_DELAY);
    }
    if (keyTwo != prevKeyTwo) {
      changeInKeys = true;
      if (prevKeyTwo == 0b1) {
        noteMessage[0] = 'P';
      } else {
        noteMessage[0] = 'R';
      }
      noteMessage[1] = '4';
      noteMessage[2] = intToHex[curRowIdx + 1];
      xQueueSend( msgOutQ, (char*) noteMessage, portMAX_DELAY);
    }
    if (keyThree != prevKeyThree) {
      changeInKeys = true;
      if (prevKeyThree == 0b1) {
        noteMessage[0] = 'P';
      } else {
        noteMessage[0] = 'R';
      }
      noteMessage[1] = '4';
      noteMessage[2] = intToHex[curRowIdx + 2];
      xQueueSend( msgOutQ, (char*) noteMessage, portMAX_DELAY);
    }
    if (keyFour != prevKeyFour) {
      changeInKeys = true;
      if (prevKeyFour == 0b1) {
        noteMessage[0] = 'P';
      } else {
        noteMessage[0] = 'R';
      }
      noteMessage[1] = '4';
      noteMessage[2] = intToHex[curRowIdx + 3];
      xQueueSend( msgOutQ, (char*) noteMessage, portMAX_DELAY);
    }
  }
#ifndef NO_PREV_KEY_ARRAY_UPDATE
  for (int rowIdx = 0; rowIdx < 3; rowIdx++) {
    prevKeyArray[rowIdx] = keyArrayLocal[rowIdx];
  }
#endif
  return changeInKeys;
}

inline uint8_t* readIntoKeyArray() {//check if you can do this
  static uint8_t keyArrayLocal[7];
  for (int rowIdx = 0; rowIdx < 7; rowIdx++) {
    setRow(rowIdx);
    delayMicroseconds(10);
    uint8_t keys = readCols();
    keyArrayLocal[rowIdx] = keys;
  }
  return keyArrayLocal;
}


// Change current step sizes for polyphony keys
inline void setCurrentStepSizeAtomic(bool changeInKeys, uint8_t* keyArrayLocal) {
  // Look up the phase step size for the key that is pressed and update currentStepSize
  uint8_t waveFormLocal = __atomic_load_n(&waveForm, __ATOMIC_RELAXED);
  uint32_t currentStepSizeLocalList [] = {0, 0, 0};
  int counter = 0;

  // Sawtooth Waveform
  if (waveFormLocal == SAWTOOTH) {
    xSemaphoreTake(polyArrayMutex, portMAX_DELAY);
    for (uint32_t rowIdx = 0; rowIdx < 3; rowIdx++) { //change data types
      uint32_t curRowIdx = rowIdx << 2;
      if ((keyArrayLocal[rowIdx] >> 0 & 0b1) == 0 && counter < 3) {
        currentStepSizeLocalList[counter] = stepSizes[curRowIdx + 0];
        polyFrequencies[counter] = 1 + curRowIdx + 0;
        counter++;
      }
      if ((keyArrayLocal[rowIdx] >> 1 & 0b1) == 0 && counter < 3) {
        currentStepSizeLocalList[counter] = stepSizes[curRowIdx + 1];
        polyFrequencies[counter] = 1 + curRowIdx + 1;
        counter++;
      }
      if ((keyArrayLocal[rowIdx] >> 2 & 0b1) == 0 && counter < 3) {
        currentStepSizeLocalList[counter] = stepSizes[curRowIdx + 2];
        polyFrequencies[counter] = 1 + curRowIdx + 2;
        counter++;
      }
      if ((keyArrayLocal[rowIdx] >> 3 & 0b1) == 0 && counter < 3) {
        currentStepSizeLocalList[counter] = stepSizes[curRowIdx + 3];
        polyFrequencies[counter] = 1 + curRowIdx + 3;
        counter++;
      }
    }
    if (changeInKeys) {
      // set new step sizes
      for (int i = 0; i < counter; i++) {
        if (i == 0) {
          __atomic_store_n(&currentStepSize_0, currentStepSizeLocalList[0], __ATOMIC_RELAXED);
        }
        else if (i == 1) {
          __atomic_store_n(&currentStepSize_1, currentStepSizeLocalList[1], __ATOMIC_RELAXED);
        }
        else if (i == 2) {
          __atomic_store_n(&currentStepSize_2, currentStepSizeLocalList[2], __ATOMIC_RELAXED);
        }
      }
      // remove step sizes when no key pressed
      for (int i = 0; i < 3 - counter; i++) {
        int idx = 3 - (i + 1);
        if (idx == 0) {
          __atomic_store_n(&currentStepSize_0, 0, __ATOMIC_RELAXED);
          polyFrequencies[0] = NULL;
        }
        if (idx == 1) {
          __atomic_store_n(&currentStepSize_1, 0, __ATOMIC_RELAXED);
          polyFrequencies[1] = NULL;
        }
        if (idx == 2) {
          __atomic_store_n(&currentStepSize_2, 0, __ATOMIC_RELAXED);
          polyFrequencies[2] = NULL;
        }
      }
    }
    xSemaphoreGive(polyArrayMutex);
  }

  // Sine Waveform
  else if (waveFormLocal == SINE) {
    xSemaphoreTake(polyArrayMutex, portMAX_DELAY);
    for (uint32_t rowIdx = 0; rowIdx < 3; rowIdx++) { //change data types
      uint32_t curRowIdx = rowIdx << 2;
      if ((keyArrayLocal[rowIdx] >> 0 & 0b1) == 0 && counter < 3) {
        currentStepSizeLocalList[counter] = sineStepSizes[curRowIdx + 0];
        polyFrequencies[counter] = 1 + curRowIdx + 0;
        counter++;
      }
      if ((keyArrayLocal[rowIdx] >> 1 & 0b1) == 0 && counter < 3) {
        currentStepSizeLocalList[counter] = sineStepSizes[curRowIdx + 1];
        polyFrequencies[counter] = 1 + curRowIdx + 1;
        counter++;
      }
      if ((keyArrayLocal[rowIdx] >> 2 & 0b1) == 0 && counter < 3) {
        currentStepSizeLocalList[counter] = sineStepSizes[curRowIdx + 2];
        polyFrequencies[counter] = 1 + curRowIdx + 2;
        counter++;
      }
      if ((keyArrayLocal[rowIdx] >> 3 & 0b1) == 0 && counter < 3) {
        currentStepSizeLocalList[counter] = sineStepSizes[curRowIdx + 3];
        polyFrequencies[counter] = 1 + curRowIdx + 3;
        counter++;
      }
    }
    if (changeInKeys) {
      // set new step sizes
      for (int i = 0; i < counter; i++) {
        if (i == 0) {
          __atomic_store_n(&sineCurrentStepSize_0, currentStepSizeLocalList[0], __ATOMIC_RELAXED);
        }
        else if (i == 1) {
          __atomic_store_n(&sineCurrentStepSize_1, currentStepSizeLocalList[1], __ATOMIC_RELAXED);
        }
        else if (i == 2) {
          __atomic_store_n(&sineCurrentStepSize_2, currentStepSizeLocalList[2], __ATOMIC_RELAXED);
        }
      }
      // remove step sizes when no key pressed
      for (int i = 0; i < 3 - counter; i++) {
        int idx = 3 - (i + 1);
        if (idx == 0) {
          __atomic_store_n(&sineCurrentStepSize_0, 0, __ATOMIC_RELAXED);
          polyFrequencies[0] = NULL;
        }
        if (idx == 1) {
          __atomic_store_n(&sineCurrentStepSize_1, 0, __ATOMIC_RELAXED);
          polyFrequencies[1] = NULL;
        }
        if (idx == 2) {
          __atomic_store_n(&sineCurrentStepSize_2, 0, __ATOMIC_RELAXED);
          polyFrequencies[2] = NULL;
        }
      }
    }
    xSemaphoreGive(polyArrayMutex);
  }
}


// -----------------------------------------------
// ------------- Set JoyStick Mode ---------------
// -----------------------------------------------
inline void setJoystickMode(uint8_t* keyArrayLocal) {
  static bool prevJoystickMode = false;
  static bool pressedBefore = false;
  if (!(keyArrayLocal[5] >> 2 & 0b1) && !pressedBefore) {
    pressedBefore = true;
    prevJoystickMode = !prevJoystickMode;
    __atomic_store_n(&joyStickMode, prevJoystickMode, __ATOMIC_RELAXED);
  } else {
    if (pressedBefore && !(keyArrayLocal[5] >> 2 & 0b1)) {

    } else {
      if (pressedBefore) {
        pressedBefore = false;
      }
    }
  }
}


// -----------------------------------------------
// ------------- Set Waveform Mode ---------------
// -----------------------------------------------
inline void setWaveForm(uint8_t* keyArrayLocal) {
  static bool prevWaveForm = false;
  static bool pressedBefore = false;
  uint8_t waveFormLocal = SAWTOOTH;
  if (!(keyArrayLocal[5] >> 1 & 0b1) && !pressedBefore) {
    pressedBefore = true;
    prevWaveForm = !prevWaveForm;
    if (prevWaveForm == true) {
      waveFormLocal = SINE;
    } else {
      waveFormLocal = SAWTOOTH;
    }
    __atomic_store_n(&waveForm, waveFormLocal, __ATOMIC_RELAXED);
  } else {
    if (pressedBefore && !(keyArrayLocal[5] >> 1 & 0b1)) {

    } else {
      if (pressedBefore) {
        pressedBefore = false;
      }
    }
  }
}


// -----------------------------------------------
// -------------- Set Reverb Mode ----------------
// -----------------------------------------------
inline void setReverb(uint8_t* keyArrayLocal) {
  static bool reverbSetLocal = false;
  static bool pressedBefore = false;
  float wetLocal = 0.0f;
  float timLocal = 0.0f;
  uint32_t knob0RotationLocal = __atomic_load_n(&knob0Rotation, __ATOMIC_RELAXED);
  uint32_t knob1RotationLocal = __atomic_load_n(&knob1Rotation, __ATOMIC_RELAXED);
  wetLocal = ((float)knob0RotationLocal) / 16.0f;
  timLocal = ((float)knob1RotationLocal) / 16.0f;
  xSemaphoreTake(wetMutex, portMAX_DELAY);
  wet = wetLocal;
  tim = timLocal;
  xSemaphoreGive(wetMutex);
  if (!(keyArrayLocal[6] & 0b1) && !pressedBefore) {
    pressedBefore = true;
    reverbSetLocal = !reverbSetLocal;
    __atomic_store_n(&reverbSet, reverbSetLocal, __ATOMIC_RELAXED);
  } else {
    if (pressedBefore && !(keyArrayLocal[6] & 0b1)) {

    } else {
      if (pressedBefore) {
        pressedBefore = false;
      }
    }
  }
}


// -----------------------------------------------
// -------------- Scan Keys Task -----------------
// -----------------------------------------------
void scanKeysTask(void * pvParameters) {
  const TickType_t xFrequency = 50 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // set class for all 4 knobs
  Knob knob0(0);
  Knob knob1(1);
  Knob knob2(2);
  Knob knob3(3);

  bool changeInKeys = false;
  uint8_t* keyArrayLocal;// a local copy of the key array is made as there are many accesses to it needed here during which it would need to be mutexed which has a big effect on task prioritisation, better make a copy used locally and only mutex when writing back
#ifndef PROFILE_SCAN_KEYS
  while (1) {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    keyArrayLocal = readIntoKeyArray();//scan rows of the key matrix and store the result in local version of key array, this is after the semaphore is taken so safe
    uint32_t periodLengthLocal = 56 + (analogRead(JOYX_PIN) / 50);
    float dutyCycle = ((float)analogRead(JOYY_PIN)) / 1024.0;
    //Serial.println(dutyCycle);
    uint32_t highLimitLocal = (uint32_t)(dutyCycle * periodLengthLocal);
    uint32_t lowLimitLocal = periodLength - highLimitLocal;
    __atomic_store_n(&periodLength, periodLengthLocal, __ATOMIC_RELAXED);
    __atomic_store_n(&highLimit, highLimitLocal, __ATOMIC_RELAXED);
    __atomic_store_n(&lowLimit, lowLimitLocal, __ATOMIC_RELAXED);
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);//check if there's other places where you can do the memcpy
    memcpy((uint8_t*)keyArray, keyArrayLocal, 7);//only 7 bytes to copy should not take too much memory and CPU time so worth the improvements to task prioritisation.
    xSemaphoreGive(keyArrayMutex);

    changeInKeys = fillMessageQueue(keyArrayLocal);//returns true if any changes were seen as this is also means there are messages to send

    setJoystickMode(keyArrayLocal);
    setWaveForm(keyArrayLocal);
    setCurrentStepSizeAtomic(changeInKeys, keyArrayLocal);
    setReverb(keyArrayLocal);

    // knob1 and knob2 unused
    knob0.setKnobRotationAtomic(keyArrayLocal);
    //knob1.setKnobRotationAtomic(keyArrayLocal);
    //knob2.setKnobRotationAtomic(keyArrayLocal);
    knob3.setKnobRotationAtomic(keyArrayLocal);
  }
#endif
#ifdef PROFILE_SCAN_KEYS
  uint32_t startTime = micros();
  for (int i = 0; i < 32; i++) {
    keyArrayLocal = readIntoKeyArray();//scan rows of the key matrix and store the result in local version of key array, this is after the semaphore is taken so safe
    uint32_t periodLengthLocal = 56 + (analogRead(JOYX_PIN) / 50);
    float dutyCycle = ((float)analogRead(JOYY_PIN)) / 1024.0;
    //Serial.println(dutyCycle);
    uint32_t highLimitLocal = (uint32_t)(dutyCycle * periodLengthLocal);
    uint32_t lowLimitLocal = periodLength - highLimitLocal;
    __atomic_store_n(&periodLength, periodLengthLocal, __ATOMIC_RELAXED);
    __atomic_store_n(&highLimit, highLimitLocal, __ATOMIC_RELAXED);
    __atomic_store_n(&lowLimit, lowLimitLocal, __ATOMIC_RELAXED);
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);//check if there's other places where you can do the memcpy
    memcpy((uint8_t*)keyArray, keyArrayLocal, 7);//only 7 bytes to copy should not take too much memory and CPU time so worth the improvements to task prioritisation.
    xSemaphoreGive(keyArrayMutex);

    changeInKeys = fillMessageQueue(keyArrayLocal);//returns true if any changes were seen as this is also means there are messages to send

    setJoystickMode(keyArrayLocal);
    setWaveForm(keyArrayLocal);
    setCurrentStepSizeAtomic(changeInKeys, keyArrayLocal);
    setReverb(keyArrayLocal);

    // knob1 and knob2 unused
    knob0.setKnobRotationAtomic(keyArrayLocal);
    //knob1.setKnobRotationAtomic(keyArrayLocal);
    //knob2.setKnobRotationAtomic(keyArrayLocal);
    knob3.setKnobRotationAtomic(keyArrayLocal);
  }
  uint32_t timeTaken = micros() - startTime;
  delayMicroseconds(1000000);
  Serial.println(timeTaken);
  while (1) {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
#endif
}


// -----------------------------------------------
// ------------ Display Update Task --------------
// -----------------------------------------------
void displayUpdateTask(void * pvParameters) {
  const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  char keyDisplay[6];
  char volumeLevel[2];
  char reverbLevel[2];
  char dispNote[13];
  char dispWave[4];
  char dispMode[8];
  char dispVol[8];
  char dispReverb[8];
  uint8_t keyArrayLocal[7];
#ifndef PROFILE_DISPLAY_UPDATE_TASK
  while (1) {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
    keyDisplay[0] = '-';
    keyDisplay[1] = '-';
    keyDisplay[2] = ' ';
    keyDisplay[3] = ' ';
    keyDisplay[4] = ' ';
    keyDisplay[5] = ' ';

    uint32_t volumeLevelNumber = __atomic_load_n(&knob3Rotation, __ATOMIC_RELAXED);

    uint32_t polyFrequenciesLocal[3];
    xSemaphoreTake(polyArrayMutex, portMAX_DELAY);
    polyFrequenciesLocal[0] = polyFrequencies[0];
    polyFrequenciesLocal[1] = polyFrequencies[1];
    polyFrequenciesLocal[2] = polyFrequencies[2];
    xSemaphoreGive(polyArrayMutex);

    itoa (volumeLevelNumber, volumeLevel, 10);
    if (volumeLevelNumber < 10) {
      volumeLevel[1] = ' ';
    }

    bool reverbSetLocal = __atomic_load_n(&reverbSet, __ATOMIC_RELAXED);
    if (reverbSetLocal) {
      uint32_t reverbLevelNumber;
      reverbLevelNumber = __atomic_load_n(&knob0Rotation, __ATOMIC_RELAXED);
      itoa (reverbLevelNumber, reverbLevel, 10);
      if (reverbLevelNumber < 10) {
        reverbLevel[1] = ' ';
      }
    } else {
      reverbLevel[0] = '-';
      reverbLevel[1] = '-';
    }

    uint8_t waveFormLocal = __atomic_load_n(&waveForm, __ATOMIC_RELAXED);
    if (waveFormLocal == SINE) {
      dispWave[0] = 'S';
      dispWave[1] = 'I';
      dispWave[2] = 'N';
      dispWave[3] = '\n';

    } else if (waveFormLocal == SAWTOOTH) {
      dispWave[0] = 'S';
      dispWave[1] = 'A';
      dispWave[2] = 'W';
      dispWave[3] = '\n';
    }
    dispMode[0] = 'M';
    dispMode[1] = 'o';
    dispMode[2] = 'd';
    dispMode[3] = 'e';
    dispMode[4] = ':';
    dispMode[5] = ' ';
    dispMode[7] = '\n';

    dispVol[0] = 'V';
    dispVol[1] = 'o';
    dispVol[2] = 'l';
    dispVol[3] = ':';
    dispVol[4] = ' ';
    dispVol[5] = volumeLevel[0];
    dispVol[6] =  volumeLevel[1];
    dispVol[7] = '\n';

    dispReverb[0] = 'R';
    dispReverb[1] = 'e';
    dispReverb[2] = 'v';
    dispReverb[3] = ':';
    dispReverb[4] = ' ';
    dispReverb[5] = reverbLevel[0];
    dispReverb[6] =  reverbLevel[1];
    dispReverb[7] = '\n';

    for (int i = 0; i < 3; i++) {
      if ((polyFrequenciesLocal[i] == 1)) {
        keyDisplay[i * 2] = 'C';
        keyDisplay[i * 2 + 1] = ' ';
      }
      if ((polyFrequenciesLocal[i] == 2)) {
        keyDisplay[i * 2] = 'C';
        keyDisplay[i * 2 + 1] = '#';
      }
      if ((polyFrequenciesLocal[i] == 3)) {
        keyDisplay[i * 2] = 'D';
        keyDisplay[i * 2 + 1] = ' ';
      }
      if ((polyFrequenciesLocal[i] == 4)) {
        keyDisplay[i * 2] = 'D';
        keyDisplay[i * 2 + 1] = '#';
      }
      if ((polyFrequenciesLocal[i] == 5)) {
        keyDisplay[i * 2] = 'E';
        keyDisplay[i * 2 + 1] = ' ';
      }
      if ((polyFrequenciesLocal[i] == 6)) {
        keyDisplay[i * 2] = 'F';
        keyDisplay[i * 2 + 1] = ' ';
      }
      if ((polyFrequenciesLocal[i] == 7)) {
        keyDisplay[i * 2] = 'F';
        keyDisplay[i * 2 + 1] = '#';
      }
      if ((polyFrequenciesLocal[i] == 8)) {
        keyDisplay[i * 2] = 'G';
        keyDisplay[i * 2 + 1] = ' ';
      }
      if ((polyFrequenciesLocal[i] == 9)) {
        keyDisplay[i * 2] = 'G';
        keyDisplay[i * 2 + 1] = '#';
      }
      if ((polyFrequenciesLocal[i] == 10)) {
        keyDisplay[i * 2] = 'A';
        keyDisplay[i * 2 + 1] = ' ';
      }
      if ((polyFrequenciesLocal[i] == 11)) {
        keyDisplay[i * 2] = 'A';
        keyDisplay[i * 2 + 1] = '#';
      }
      if ((polyFrequenciesLocal[i] == 12)) {
        keyDisplay[i * 2] = 'B';
        keyDisplay[i * 2 + 1] = ' ';
      }
    }
    dispNote[0] = 'N';
    dispNote[1] = 'o';
    dispNote[2] = 't';
    dispNote[3] = 'e';
    dispNote[4] = ':';
    dispNote[5] = ' ';
    dispNote[6] = keyDisplay[0];
    dispNote[7] = keyDisplay[1];
    dispNote[8] = keyDisplay[2];
    dispNote[9] = keyDisplay[3];
    dispNote[10] = keyDisplay[4];
    dispNote[11] = keyDisplay[5];
    dispNote[12] = '\n';
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    if (joyStickMode) {
      dispMode[6] = 'J';
      dispWave[0] = 'S';
      dispWave[1] = 'Q';
      dispWave[2] = 'R';
      dispWave[3] = '\n';
      dispNote[6] = '-';
      dispNote[7] = '-';
    } else {
      dispMode[6] = 'K';
    }
    memcpy(keyArrayLocal, (uint8_t*) keyArray, 7); //only 7 bytes to copy should not take too much memory and CPU time so worth the improvements to task prioritisation.
    xSemaphoreGive(keyArrayMutex);
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.drawStr(2, 10, dispNote); // write something to the internal memory
    u8g2.drawStr(80, 10, dispMode); // write something to the internal memory
    u8g2.drawStr(90, 30, dispVol); // write something to the internal memory
    u8g2.drawStr(50, 30, dispWave); // write something to the internal memory
    u8g2.drawStr(2, 30, dispReverb); // write something to the internal memory
    u8g2.sendBuffer();          // transfer internal memory to the dispNotelay*/
  }
#endif
#ifdef PROFILE_DISPLAY_UPDATE_TASK
  //Start of configuration to force worst-case execution
  __atomic_store_n(&reverbSet, true, __ATOMIC_RELAXED);// more is written if reverb is set
  //End of configuration to force worst-case execution
  uint32_t startTime = micros();
  for (int i = 0; i < 32; i++) {
    keyDisplay[0] = '-';
    keyDisplay[1] = '-';
    keyDisplay[2] = ' ';
    keyDisplay[3] = ' ';
    keyDisplay[4] = ' ';
    keyDisplay[5] = ' ';

    uint32_t volumeLevelNumber = __atomic_load_n(&knob3Rotation, __ATOMIC_RELAXED);

    uint32_t polyFrequenciesLocal[3];
    xSemaphoreTake(polyArrayMutex, portMAX_DELAY);
    polyFrequenciesLocal[0] = polyFrequencies[0];
    polyFrequenciesLocal[1] = polyFrequencies[1];
    polyFrequenciesLocal[2] = polyFrequencies[2];
    xSemaphoreGive(polyArrayMutex);

    itoa (volumeLevelNumber, volumeLevel, 10);
    if (volumeLevelNumber < 10) {
      volumeLevel[1] = ' ';
    }

    bool reverbSetLocal = __atomic_load_n(&reverbSet, __ATOMIC_RELAXED);
    if (reverbSetLocal) {
      uint32_t reverbLevelNumber;
      reverbLevelNumber = __atomic_load_n(&knob0Rotation, __ATOMIC_RELAXED);
      itoa (reverbLevelNumber, reverbLevel, 10);
      if (reverbLevelNumber < 10) {
        reverbLevel[1] = ' ';
      }
    } else {
      reverbLevel[0] = '-';
      reverbLevel[1] = '-';
    }

    uint8_t waveFormLocal = __atomic_load_n(&waveForm, __ATOMIC_RELAXED);
    if (waveFormLocal == SINE) {
      dispWave[0] = 'S';
      dispWave[1] = 'I';
      dispWave[2] = 'N';
      dispWave[3] = '\n';

    } else if (waveFormLocal == SAWTOOTH) {
      dispWave[0] = 'S';
      dispWave[1] = 'A';
      dispWave[2] = 'W';
      dispWave[3] = '\n';
    }
    dispMode[0] = 'M';
    dispMode[1] = 'o';
    dispMode[2] = 'd';
    dispMode[3] = 'e';
    dispMode[4] = ':';
    dispMode[5] = ' ';
    dispMode[7] = '\n';

    dispVol[0] = 'V';
    dispVol[1] = 'o';
    dispVol[2] = 'l';
    dispVol[3] = ':';
    dispVol[4] = ' ';
    dispVol[5] = volumeLevel[0];
    dispVol[6] =  volumeLevel[1];
    dispVol[7] = '\n';

    dispReverb[0] = 'R';
    dispReverb[1] = 'e';
    dispReverb[2] = 'v';
    dispReverb[3] = ':';
    dispReverb[4] = ' ';
    dispReverb[5] = reverbLevel[0];
    dispReverb[6] =  reverbLevel[1];
    dispReverb[7] = '\n';

    for (int i = 0; i < 3; i++) {
      if ((polyFrequenciesLocal[i] == 1)) {
        keyDisplay[i * 2] = 'C';
        keyDisplay[i * 2 + 1] = ' ';
      }
      if ((polyFrequenciesLocal[i] == 2)) {
        keyDisplay[i * 2] = 'C';
        keyDisplay[i * 2 + 1] = '#';
      }
      if ((polyFrequenciesLocal[i] == 3)) {
        keyDisplay[i * 2] = 'D';
        keyDisplay[i * 2 + 1] = ' ';
      }
      if ((polyFrequenciesLocal[i] == 4)) {
        keyDisplay[i * 2] = 'D';
        keyDisplay[i * 2 + 1] = '#';
      }
      if ((polyFrequenciesLocal[i] == 5)) {
        keyDisplay[i * 2] = 'E';
        keyDisplay[i * 2 + 1] = ' ';
      }
      if ((polyFrequenciesLocal[i] == 6)) {
        keyDisplay[i * 2] = 'F';
        keyDisplay[i * 2 + 1] = ' ';
      }
      if ((polyFrequenciesLocal[i] == 7)) {
        keyDisplay[i * 2] = 'F';
        keyDisplay[i * 2 + 1] = '#';
      }
      if ((polyFrequenciesLocal[i] == 8)) {
        keyDisplay[i * 2] = 'G';
        keyDisplay[i * 2 + 1] = ' ';
      }
      if ((polyFrequenciesLocal[i] == 9)) {
        keyDisplay[i * 2] = 'G';
        keyDisplay[i * 2 + 1] = '#';
      }
      if ((polyFrequenciesLocal[i] == 10)) {
        keyDisplay[i * 2] = 'A';
        keyDisplay[i * 2 + 1] = ' ';
      }
      if ((polyFrequenciesLocal[i] == 11)) {
        keyDisplay[i * 2] = 'A';
        keyDisplay[i * 2 + 1] = '#';
      }
      if ((polyFrequenciesLocal[i] == 12)) {
        keyDisplay[i * 2] = 'B';
        keyDisplay[i * 2 + 1] = ' ';
      }
    }

    dispNote[0] = 'N';
    dispNote[1] = 'o';
    dispNote[2] = 't';
    dispNote[3] = 'e';
    dispNote[4] = ':';
    dispNote[5] = ' ';
    dispNote[6] = keyDisplay[0];
    dispNote[7] = keyDisplay[1];
    dispNote[8] = keyDisplay[2];
    dispNote[9] = keyDisplay[3];
    dispNote[10] = keyDisplay[4];
    dispNote[11] = keyDisplay[5];
    dispNote[12] = '\n';
    xSemaphoreTake(keyArrayMutex, portMAX_DELAY);
    if (joyStickMode) {
      dispMode[6] = 'J';
      dispWave[0] = 'S';
      dispWave[1] = 'Q';
      dispWave[2] = 'R';
      dispWave[3] = '\n';
      dispNote[6] = '-';
      dispNote[7] = '-';
    } else {
      dispMode[6] = 'K';
    }
    memcpy(keyArrayLocal, (uint8_t*) keyArray, 7); //only 7 bytes to copy should not take too much memory and CPU time so worth the improvements to task prioritisation.
    xSemaphoreGive(keyArrayMutex);
    u8g2.clearBuffer();         // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr); // choose a suitable font
    u8g2.drawStr(2, 10, dispNote); // write something to the internal memory
    u8g2.drawStr(80, 10, dispMode); // write something to the internal memory
    u8g2.drawStr(90, 30, dispVol); // write something to the internal memory
    u8g2.drawStr(50, 30, dispWave); // write something to the internal memory
    u8g2.drawStr(2, 30, dispReverb); // write something to the internal memory
    u8g2.sendBuffer();          // transfer internal memory to the dispNotelay*/
  }
  uint32_t timeTaken = micros() - startTime;
  Serial.println(timeTaken);
  while (1) {
    vTaskDelayUntil( &xLastWakeTime, xFrequency );
  }
#endif
}

// -----------------------------------------------
// ------------------- Setup ---------------------
// -----------------------------------------------
void setup() {
  // put your setup code here, to run once:
  //Initialise UART
  Serial.begin(115200);
  Serial.println("Hello World");
  //Set pin directions
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  cf0_lim = (int)(tim * l_CB0);
  cf1_lim = (int)(tim * l_CB1);
  cf2_lim = (int)(tim * l_CB2);
  cf3_lim = (int)(tim * l_CB3);
  ap0_lim = (int)(tim * l_AP0);
  ap1_lim = (int)(tim * l_AP1);
  ap2_lim = (int)(tim * l_AP2);

  TIM_TypeDef *Instance = TIM1;
  sampleTimer = new HardwareTimer(Instance);

  sampleTimer->setOverflow(22000, HERTZ_FORMAT);

#ifndef NO_SAMPLE_ISR
  sampleTimer->attachInterrupt(sampleISR);
#endif
  sampleTimer->resume();

  // Define task threads
#ifndef NO_SCAN_KEYS
  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
    scanKeysTask, /* Function that implements the task */
    "scanKeys", /* Text name for the task */
    256, /* Stack size in words, not bytes this was originally 32 not enough now that I broke it up into funcs*/
    NULL, /* Parameter passed into the task */
    2, /* Task priority 50 ms II, lower II chosen as prediction based on impossible states was judged to give acceptable enough accuracy*/
    &scanKeysHandle ); /* Pointer to store the task handle */
#endif
#ifndef NO_DISPLAY_UPDATE
  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(
    displayUpdateTask, /* Function that implements the task */
    "displayUpdate", /* Text name for the task */
    256, /* Stack size in words, not bytes */
    NULL, /* Parameter passed into the task */
    1, /* Lowert task priority due to 100ms II being the longest*/
    &displayUpdateHandle ); /* Pointer to store the task handle */
#endif
#ifndef NO_MSG_OUT
  TaskHandle_t msgOutHandle = NULL;
  xTaskCreate(
    msgOutTask, /* Function that implements the task */
    "msgOut", /* Text name for the task */
    32, /* Stack size in words, not bytes */
    NULL, /* Parameter passed into the task */
    2, /* Task priority, 50 ms II same as scanKeysTask so both have same priority of 2 which is less than samplewrite and msgin*/
    &msgOutHandle ); /* Pointer to store the task handle */
#endif
#ifndef NO_MSG_IN
  TaskHandle_t msgInHandle = NULL;
  xTaskCreate(
    msgInTask, /* Function that implements the task */
    "msgIn", /* Text name for the task */
    256, /* Stack size in words, not bytes */
    NULL, /* Parameter passed into the task */
    4, /* Highest task priority due to 5 ms II*/
    &msgInHandle ); /* Pointer to store the task handle */
#endif
  keyArrayMutex = xSemaphoreCreateMutex();//make sure this is always before scheduler is started as the threads need the mutex
  wetMutex = xSemaphoreCreateMutex();//wet is a float and atomic stores/load are incompatible with that
  doubleBufferOneSemaphore = xSemaphoreCreateBinary();
  doubleBufferTwoSemaphore = xSemaphoreCreateBinary();
  KnobThreadSafeMutex = xSemaphoreCreateMutex();
  polyArrayMutex = xSemaphoreCreateMutex();
#ifndef NO_MSG_OUT
  msgOutQ = xQueueCreate(8, 4);
#endif
#ifdef NO_MSG_OUT
  msgOutQ = xQueueCreate( 256, 4);
#endif
#ifndef NO_SAMPLE_WRITE_TASK
  TaskHandle_t sampleWriteHandle = NULL;
  xTaskCreate(
    sampleWriteTask, /* Function that implements the task */
    "sampleWrite", /* Text name for the task */
    256, /* Stack size in words, not bytes */
    NULL, /* Parameter passed into the task */
    3, /* Task priority 10 ms II*/
    &sampleWriteHandle ); /* Pointer to store the task handle */
#endif
  vTaskStartScheduler();
}

void loop() {
  // empty loop
}
