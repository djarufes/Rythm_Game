#include "stm32f0xx.h"
#include "stm32f0_discovery.h"
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#define ROWS 16 //32 divided by 2 (top and bottom sections)
#define COLS 64

//pins
#define OE 12
#define CLK 11
#define LAT 10
#define D 9
#define C 8
#define B 7
#define A 6
#define R2 5
#define G2 4
#define B2 3
#define R1 2
#define G1 1
#define B1 0

//gpio
#define GPIOB_ODR_R1  ((uint16_t)0x01)
#define GPIOB_ODR_B1  ((uint16_t)0x02)
#define GPIOB_ODR_G1  ((uint16_t)0x04)
#define GPIOB_ODR_R2  ((uint16_t)0x08)
#define GPIOB_ODR_B2  ((uint16_t)0x10)
#define GPIOB_ODR_G2  ((uint16_t)0x20)

//note related variables
int lane1 = 0;
int lane2 = 0;
int lane3 = 0;
int lane4 = 0;
int lane5 = 0;
int lane6 = 0;

//Count
int k = 0;

//Lane related variables
int active[6][3] = {0};
int shift[6][3] = {0};

//Score variables
int correctHits = 0;
int displayScore = 0;
int correctmark = 0;

//Hold detection variables (to prevent cheating)
int history1 = 0;
int history2 = 0;
int history3 = 0;
int history4 = 0;
int history5 = 0;
int history6 = 0;

#define RATE 100000
#define N 1000
short int wavetable[N];

int offset[6] = {0};
//Change the steps to tune the notes, 0-5 = PB6-PB4 THEN PB15-PB13 (Double them to move to higher scale)
// C4 = 2360 (261.63 Hz)
// D4 = 2640 (293.66 Hz)
// E4 = 2950 (329.63 Hz)
// F4 = 3150 (349.23 Hz)
// G4 = 3530 (392.00 Hz)
// A4 = 3960 (440.00 Hz)

int step[6] = {3960 * N / 100000.0 * (1 << 16), 3530 * N / 100000.0 * (1 << 16), 3150 * N / 100000.0 * (1 << 16), 2950 * N / 100000.0 * (1 << 16), 2640 * N / 100000.0 * (1 << 16), 2360 * N / 100000.0 * (1 << 16)};
int sample = 0;
void micro_wait(unsigned int);

//-------------- DISPLAY SECTIONS --------------------

uint8_t image[16][64] = {{0, 0, 0, 0, 0, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 63, 63, 63, 63, 63, 63, 63, 63},
                         {6, 6, 6, 6, 6, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 63},
                         {6, 48, 0, 48, 6, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 63},
                         {6, 6, 6, 6, 6, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 63},
                         {48, 48, 48, 0, 48, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 63},
                         {54,6, 54, 54, 6, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 63, 63, 63, 63, 63, 63, 63, 63},
                         {54, 48, 48, 48, 54, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 63},
                         {6, 6, 6, 6, 6, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 63},
                         {48, 0, 0, 0, 48, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 63},
                         {54, 6, 6, 6, 54, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 63},
                         {54, 48, 48, 48, 54, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 63, 63, 63, 63, 63, 63, 63, 63},
                         {6, 6, 6, 6, 6, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 63},
                         {48, 0, 48, 48, 48, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 63},
                         {54, 6, 54, 6, 54, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 63},
                         {54, 48, 48, 0, 54, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 63},
                         {6, 6, 6, 6, 6, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 9, 63, 63, 63, 63, 63, 63, 63, 63}};

const uint8_t startScreen[16][64] =  {{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8, 8, 8, 8, 8, 8, 0, 0, 8, 8, 8, 8, 8, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 8, 1, 0, 8, 9, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 48, 48, 48, 52, 52, 52, 52, 0},
                                     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 8, 1, 0, 8, 9, 0, 0, 9, 1, 8, 9, 9, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 48, 48, 48, 52, 48, 48, 52, 0},
                                     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 9, 9, 9, 9, 9, 0, 0, 9, 1, 8, 8, 9, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 52, 52, 4, 52, 4, 52, 52, 0},
                                     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 9, 9, 9, 9, 1, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 52, 52, 4, 52, 4, 52, 52, 0},
                                     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 8, 8, 8, 8, 8, 8, 0, 0, 9, 9, 9, 9, 1, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 8, 9, 9, 9, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 20, 16, 16, 20, 4, 4, 20, 0},
                                     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 9, 9, 1, 1, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 20, 16, 16, 20, 4, 4, 20, 0},
                                     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 9, 9, 9, 8, 9, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 20, 4, 4, 20, 16, 16, 20, 0},
                                     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 9, 9, 9, 8, 9, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 20, 4, 4, 20, 16, 16, 20, 0},
                                     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 9, 8, 8, 8, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 9, 1, 1, 9, 9, 0, 0, 9, 9, 9, 9, 9, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 6, 6, 6, 6, 22, 22, 0},
                                     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 9, 0, 0, 9, 9, 0, 0, 8, 0, 8, 8, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 22, 22, 22, 22, 22, 22, 0},
                                     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 9, 8, 8, 9, 9, 0, 0, 9, 9, 9, 9, 9, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 16, 16, 22, 22, 16, 16, 16, 0},
                                     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 9, 9, 9, 9, 9, 9, 0, 0, 1, 9, 8, 8, 9, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 22, 22, 6, 6, 0, 16, 16, 0},
                                     {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 6, 6, 6, 6, 0, 0, 0, 0}};
                                                                                                                             //mid ^
void setup_gpio() {
    //Using GPIOC Pins 0-12
    //bit:     12  11  10  9   8   7   6   5   4   3   2   1   0
    //purpose: OE  CLK LAT D   C   B   A   R2  G2  B2  R1  G1  B1

    RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOCEN;
    // set pins 0-12 for output
    GPIOC->MODER &= ~(  GPIO_MODER_MODER0   |
                        GPIO_MODER_MODER1   |
                        GPIO_MODER_MODER2   |
                        GPIO_MODER_MODER3   |
                        GPIO_MODER_MODER4   |
                        GPIO_MODER_MODER5   |
                        GPIO_MODER_MODER6   |
                        GPIO_MODER_MODER7   |
                        GPIO_MODER_MODER8   |
                        GPIO_MODER_MODER9   |
                        GPIO_MODER_MODER10  |
                        GPIO_MODER_MODER11  |
                        GPIO_MODER_MODER12  );

    GPIOC->MODER |=  (  GPIO_MODER_MODER0_0   |
                        GPIO_MODER_MODER1_0   |
                        GPIO_MODER_MODER2_0   |
                        GPIO_MODER_MODER3_0   |
                        GPIO_MODER_MODER4_0   |
                        GPIO_MODER_MODER5_0   |
                        GPIO_MODER_MODER6_0   |
                        GPIO_MODER_MODER7_0   |
                        GPIO_MODER_MODER8_0   |
                        GPIO_MODER_MODER9_0   |
                        GPIO_MODER_MODER10_0  |
                        GPIO_MODER_MODER11_0  |
                        GPIO_MODER_MODER12_0  );

    GPIOC->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0   |
                        GPIO_OSPEEDER_OSPEEDR1   |
                        GPIO_OSPEEDER_OSPEEDR2   |
                        GPIO_OSPEEDER_OSPEEDR3   |
                        GPIO_OSPEEDER_OSPEEDR4   |
                        GPIO_OSPEEDER_OSPEEDR5   |
                        GPIO_OSPEEDER_OSPEEDR6   |
                        GPIO_OSPEEDER_OSPEEDR7   |
                        GPIO_OSPEEDER_OSPEEDR8   |
                        GPIO_OSPEEDER_OSPEEDR9   |
                        GPIO_OSPEEDER_OSPEEDR10  |
                        GPIO_OSPEEDER_OSPEEDR11  |
                        GPIO_OSPEEDER_OSPEEDR12  );

    GPIOC->OSPEEDR |= ( GPIO_OSPEEDER_OSPEEDR0   |
                        GPIO_OSPEEDER_OSPEEDR1   |
                        GPIO_OSPEEDER_OSPEEDR2   |
                        GPIO_OSPEEDER_OSPEEDR3   |
                        GPIO_OSPEEDER_OSPEEDR4   |
                        GPIO_OSPEEDER_OSPEEDR5   |
                        GPIO_OSPEEDER_OSPEEDR6   |
                        GPIO_OSPEEDER_OSPEEDR7   |
                        GPIO_OSPEEDER_OSPEEDR8   |
                        GPIO_OSPEEDER_OSPEEDR9   |
                        GPIO_OSPEEDER_OSPEEDR10  |
                        GPIO_OSPEEDER_OSPEEDR11  |
                        GPIO_OSPEEDER_OSPEEDR12  );
}

//Function that shifts the notes downwards 
void shift_notes(void){

    for(int i = 0; i < 6; i++){
        for(int j = 0; j < 3; j++){
            if(active[i][j] == 1){shift[i][j]++;}
            if(shift[i][j] > 58){shift[i][j] = 0; active[i][j] = 0;}
        }
    }

    for(int i = 0; i < 16; i++){
        for(int j = 62; j > 6; j--){
            if(!(i == 0) & !(i == 5) & !(i == 10) & !(i == 15))	//Don't shift the separation segments
            {
                image[i][j] = image[i][j-1];
            }
        }
    }
}

//Functions that choose what lane a note appears and shifts down from
void lane1set(void){
    lane1 = 2;
    if(active[0][0] == 0){
        active[0][0] = 1;
    }
    else if(active[0][1] == 0){
        active[0][1] = 1;
    }
    else{
        active[0][2] = 1;
    }
}
void lane2set(void){
    lane2 = 2;
    if(active[1][0] == 0){
        active[1][0] = 1;
    }
    else if(active[1][1] == 0){
        active[1][1] = 1;
    }
    else{
        active[1][2] = 1;
    }
}
void lane3set(void){
    lane3 = 2;
    if(active[2][0] == 0){
        active[2][0] = 1;
    }
    else if(active[2][1] == 0){
        active[2][1] = 1;
    }
    else{
        active[2][2] = 1;
    }
}
void lane4set(void){
    lane4 = 2;
    if(active[3][0] == 0){
        active[3][0] = 1;
    }
    else if(active[3][1] == 0){
        active[3][1] = 1;
    }
    else{
        active[3][2] = 1;
    }
}
void lane5set(void){
    lane5 = 2;
    if(active[4][0] == 0){
        active[4][0] = 1;
    }
    else if(active[4][1] == 0){
        active[4][1] = 1;
    }
    else{
        active[4][2] = 1;
    }
}
void lane6set(void){
    lane6 = 2;
    if(active[5][0] == 0){
        active[5][0] = 1;
    }
    else if(active[5][1] == 0){
        active[5][1] = 1;
    }
    else{
        active[5][2] = 1;
    }
}

//Function that sets and updates the game's score from 0000 to 1000 counting by 100's
void scoreNum(void)
{
    //Clear score
    //Ones
    image[9][0] = 48;
    image[9][1] = 0;
    image[9][2] = 0;
    image[9][3] = 0;
    image[9][4] = 48;

    image[10][0] = 48;
    image[10][1] = 48;
    image[10][2] = 48;
    image[10][3] = 48;
    image[10][4] = 48;

    image[11][0] = 0;
    image[11][1] = 0;
    image[11][2] = 0;
    image[11][3] = 0;
    image[11][4] = 0;

    //Tens
    if(correctHits > 9)
    {
        image[13][0] = 48;
        image[13][1] = 0;
        image[13][2] = 48;
        image[13][3] = 0;
        image[13][4] = 48;

        image[14][0] = 48;
        image[14][1] = 48;
        image[14][2] = 48;
        image[14][3] = 0;
        image[14][4] = 48;

        image[15][0] = 0;
        image[15][1] = 0;
        image[15][2] = 0;
        image[15][3] = 0;
        image[15][4] = 0;
    }

    //Set Ones place of score
    displayScore = correctHits;
    if(correctHits > 9)
    {
        displayScore = correctHits % 10;
    }

    switch(displayScore) //Ones place
    {
        case 1:
        {
            image[9][0] += 6;
            image[9][1] += 6;
            image[9][2] += 6;
            image[9][3] += 6;
            image[9][4] += 6;

            break;
        }
        case 2:
        {
            image[9][0] += 6;
            image[9][1] += 6;
            image[9][2] += 6;
            image[9][4] += 6;

            image[10][0] += 6;
            image[10][2] += 6;
            image[10][4] += 6;

            image[11][0] += 6;
            image[11][2] += 6;
            image[11][3] += 6;
            image[11][4] += 6;
            break;
        }
        case 3:
        {
            image[9][0] += 6;
            image[9][1] += 6;
            image[9][2] += 6;
            image[9][3] += 6;
            image[9][4] += 6;

            image[10][0] += 6;
            image[10][2] += 6;
            image[10][4] += 6;

            image[11][0] += 6;
            image[11][2] += 6;
            image[11][4] += 6;
            break;
        }
        case 4:
        {
            image[9][0] += 6;
            image[9][1] += 6;
            image[9][2] += 6;
            image[9][3] += 6;
            image[9][4] += 6;

            image[10][2] += 6;

            image[11][0] += 6;
            image[11][1] += 6;
            image[11][2] += 6;
            break;
        }
        case 5:
        {
            image[9][0] += 6;
            image[9][2] += 6;
            image[9][3] += 6;
            image[9][4] += 6;

            image[10][0] += 6;
            image[10][2] += 6;
            image[10][4] += 6;

            image[11][0] += 6;
            image[11][1] += 6;
            image[11][2] += 6;
            image[11][4] += 6;
            break;
        }
        case 6:
        {
            image[9][2] += 6;
            image[9][3] += 6;
            image[9][4] += 6;

            image[10][2] += 6;
            image[10][4] += 6;

            image[11][0] += 6;
            image[11][1] += 6;
            image[11][2] += 6;
            image[11][3] += 6;
            image[11][4] += 6;
            break;
        }
        case 7:
        {
            image[9][0] += 6;
            image[9][1] += 6;
            image[9][2] += 6;
            image[9][3] += 6;
            image[9][4] += 6;

            image[10][0] += 6;

            image[11][0] += 6;
            break;
        }
        case 8:
        {
            image[9][0] += 6;
            image[9][1] += 6;
            image[9][2] += 6;
            image[9][3] += 6;
            image[9][4] += 6;

            image[10][0] += 6;
            image[10][2] += 6;
            image[10][4] += 6;

            image[11][0] += 6;
            image[11][1] += 6;
            image[11][2] += 6;
            image[11][3] += 6;
            image[11][4] += 6;
            break;
        }
        case 9:
        {
            image[9][0] += 6;
            image[9][1] += 6;
            image[9][2] += 6;
            image[9][3] += 6;
            image[9][4] += 6;

            image[10][0] += 6;
            image[10][2] += 6;

            image[11][0] += 6;
            image[11][1] += 6;
            image[11][2] += 6;
            break;
        }
        default:    //0000
        {
            image[9][0] += 6;
            image[9][1] += 6;
            image[9][2] += 6;
            image[9][3] += 6;
            image[9][4] += 6;

            image[10][0] += 6;
            image[10][4] += 6;

            image[11][0] += 6;
            image[11][1] += 6;
            image[11][2] += 6;
            image[11][3] += 6;
            image[11][4] += 6;
            break;
        }
    }

    //Set tens place of score
    if(correctHits > 9)
    {
        displayScore = correctHits / 10;
        switch(displayScore)
        {
            case 1:
            {
                image[13][0] += 6;
                image[13][1] += 6;
                image[13][2] += 6;
                image[13][3] += 6;
                image[13][4] += 6;
                break;
            }

            case 2:
            {
                image[13][0] += 6;
                image[13][1] += 6;
                image[13][2] += 6;
                image[13][4] += 6;

                image[14][0] += 6;
                image[14][2] += 6;
                image[14][4] += 6;

                image[15][0] += 6;
                image[15][2] += 6;
                image[15][3] += 6;
                image[15][4] += 6;
                break;
            }
            case 3:
            {
                image[13][0] += 6;
                image[13][1] += 6;
                image[13][2] += 6;
                image[13][3] += 6;
                image[13][4] += 6;

                image[14][0] += 6;
                image[14][2] += 6;
                image[14][4] += 6;

                image[15][0] += 6;
                image[15][2] += 6;
                image[15][4] += 6;
                break;
            }
            case 4:
            {
                image[13][0] += 6;
                image[13][1] += 6;
                image[13][2] += 6;
                image[13][3] += 6;
                image[13][4] += 6;

                image[14][2] += 6;

                image[15][0] += 6;
                image[15][1] += 6;
                image[15][2] += 6;
                break;
            }
            case 5:
            {
                image[13][0] += 6;
                image[13][2] += 6;
                image[13][3] += 6;
                image[13][4] += 6;

                image[14][0] += 6;
                image[14][2] += 6;
                image[14][4] += 6;

                image[15][0] += 6;
                image[15][1] += 6;
                image[15][2] += 6;
                image[15][4] += 6;
                break;
            }
            case 6:
            {
                image[13][2] += 6;
                image[13][3] += 6;
                image[13][4] += 6;

                image[14][2] += 6;
                image[14][4] += 6;

                image[15][0] += 6;
                image[15][1] += 6;
                image[15][2] += 6;
                image[15][3] += 6;
                image[15][4] += 6;
                break;
            }
            case 7:
            {
                image[13][0] += 6;
                image[13][1] += 6;
                image[13][2] += 6;
                image[13][3] += 6;
                image[13][4] += 6;

                image[14][0] += 6;

                image[15][0] += 6;
                break;
            }
            case 8:
            {
                image[13][0] += 6;
                image[13][1] += 6;
                image[13][2] += 6;
                image[13][3] += 6;
                image[13][4] += 6;

                image[14][0] += 6;
                image[14][2] += 6;
                image[14][4] += 6;

                image[15][0] += 6;
                image[15][1] += 6;
                image[15][2] += 6;
                image[15][3] += 6;
                image[15][4] += 6;
                break;
            }
            case 9:
            {
                image[13][0] += 6;
                image[13][1] += 6;
                image[13][2] += 6;
                image[13][3] += 6;
                image[13][4] += 6;

                image[14][0] += 6;
                image[14][2] += 6;

                image[15][0] += 6;
                image[15][1] += 6;
                image[15][2] += 6;
                break;
            }
            default:    //0000
            {
                image[13][0] += 6;
                image[13][1] += 6;
                image[13][2] += 6;
                image[13][3] += 6;
                image[13][4] += 6;

                image[14][0] += 6;
                image[14][4] += 6;

                image[15][0] += 6;
                image[15][1] += 6;
                image[15][2] += 6;
                image[15][3] += 6;
                image[15][4] += 6;
                break;
            }
        }
    }
    else
    {
        image[13][0] = 54;
        image[13][1] = 6;
        image[13][2] = 54;
        image[13][3] = 6;
        image[13][4] = 54;

        image[14][0] = 54;
        image[14][1] = 48;
        image[14][2] = 48;
        image[14][3] = 0;
        image[14][4] = 54;

        image[15][0] = 6;
        image[15][1] = 6;
        image[15][2] = 6;
        image[15][3] = 6;
        image[15][4] = 6;
    }
}

//Function that drives the LED Matrix 
void draw(int ind)
{
    for(int rows = 0; rows <16; rows++)
    {
        for(int cols = 0; cols < 64; cols++)
        {
			GPIOC->ODR &= ~(GPIOB_ODR_R1) & ~(GPIOB_ODR_B1) & ~(GPIOB_ODR_G1) & ~(GPIOB_ODR_R2) & ~(GPIOB_ODR_B2) & ~(GPIOB_ODR_G2);  //Clear ODR RGB bits
			
			//If no button has been pressed, display startup screen (choose song)
			//Else, display the game, playin the song that was chosen.
			if(ind == 0)
			{
				GPIOC->ODR |= startScreen[rows][cols];  //Screen pixel value (RGB)
			}
			else
			{
				GPIOC->ODR |= image[rows][cols];  //Screen pixel value (RGB)
			}

			GPIOC->BSRR |= 1<<CLK;
			GPIOC->BRR |= 1<<CLK;
        }

        //Latch values from buffer to screen
        GPIOC->BSRR |= 1<<OE;
        GPIOC->BSRR |= 1<<LAT;
        GPIOC->BRR |= 1<<LAT;

        //New row address = row variable. row bits are 6-10
        GPIOC->BRR |= 0b1111 << A;
        GPIOC->BSRR |= (rows << A);

        //Clear output enable
        //micro_wait(200);     //Dims display (but some flickering is visible)
        GPIOC->BRR |= 1<<OE;
    }
}
//------------------- BUTTONS SECTION -------------------------

void setUpButtons(void){
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB->MODER &= ~(GPIO_MODER_MODER4 | GPIO_MODER_MODER5 | GPIO_MODER_MODER6 | GPIO_MODER_MODER13 | GPIO_MODER_MODER14 | GPIO_MODER_MODER15);
    GPIOB->MODER |= (GPIO_MODER_MODER4_0 | GPIO_MODER_MODER5_0 | GPIO_MODER_MODER6_0 | GPIO_MODER_MODER13_0 | GPIO_MODER_MODER14_0 | GPIO_MODER_MODER15_0);
    GPIOB->ODR = 0;
}

int buttonPressed(void){
    if((GPIOB->IDR & GPIO_IDR_13) || (GPIOB->IDR & GPIO_IDR_5) || (GPIOB->IDR & GPIO_IDR_4) || (GPIOB->IDR & GPIO_IDR_15) || (GPIOB->IDR & GPIO_IDR_14) || (GPIOB->IDR & GPIO_IDR_6) ){
        return 1;
    }
    return 0;
}
void setupDAC(void) {
    RCC->APB1ENR |= RCC_APB1ENR_DACEN;
    //  - its output is routed to its output pin.
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA->MODER &= ~(GPIO_MODER_MODER4);
    GPIOA->MODER |= GPIO_MODER_MODER4;
    //^^^^FOUND IN DATA SHEET

    //  - it is triggered by software.VVV
    DAC->CR &= ~(DAC_CR_TSEL1);
    DAC->CR |= (DAC_CR_TSEL1);

    //  - the trigger is enabled.
    //  - it is enabled.
    DAC->CR |= (DAC_CR_TEN1);
    DAC->CR |= (DAC_CR_EN1);
}

//Play sound only when a button is pressed (6 buttons)
void playSound(void){
    if((GPIOB->IDR & GPIO_IDR_6) == GPIO_IDR_6){		//Button 1
        DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;

        offset[0] += step[0];
        if ((offset[0]>>16) >= N) offset[0] -= N<<16;
        sample = wavetable[offset[0]>>16];
        sample = sample / 16 + 2048;
        if (sample > 4095) sample = 4095;
        else if(sample < 0) sample = 0;
        DAC->DHR12R1 = sample;
    }
    else if((GPIOB->IDR & GPIO_IDR_5) == GPIO_IDR_5){	//Button 2
        DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;

        offset[1] += step[1];
        if ((offset[1]>>16) >= N) offset[1] -= N<<16;
        sample = wavetable[offset[1]>>16];
        sample = sample / 16 + 2048;
        if (sample > 4095) sample = 4095;
        else if(sample < 0) sample = 0;
        DAC->DHR12R1 = sample;
    }
    else if((GPIOB->IDR & GPIO_IDR_4) == GPIO_IDR_4){	//Button 3
        DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;

        offset[2] += step[2];
        if ((offset[2]>>16) >= N) offset[2] -= N<<16;
        sample = wavetable[offset[2]>>16];
        sample = sample / 16 + 2048;
        if (sample > 4095) sample = 4095;
        else if(sample < 0) sample = 0;
        DAC->DHR12R1 = sample;
    }
    else if((GPIOB->IDR & GPIO_IDR_15) == GPIO_IDR_15){	//Button 4
        DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;

        offset[3] += step[3];
        if ((offset[3]>>16) >= N) offset[3] -= N<<16;
        sample = wavetable[offset[3]>>16];
        sample = sample / 16 + 2048;
        if (sample > 4095) sample = 4095;
        else if(sample < 0) sample = 0;
        DAC->DHR12R1 = sample;
    }
    else if((GPIOB->IDR & GPIO_IDR_14) == GPIO_IDR_14){	//Button 5
        DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;

        offset[4] += step[4];
        if ((offset[4]>>16) >= N) offset[4] -= N<<16;
        sample = wavetable[offset[4]>>16];
        sample = sample / 16 + 2048;
        if (sample > 4095) sample = 4095;
        else if(sample < 0) sample = 0;
        DAC->DHR12R1 = sample;
    }
    else if((GPIOB->IDR & GPIO_IDR_13) == GPIO_IDR_13){	//Button 6
        DAC->SWTRIGR |= DAC_SWTRIGR_SWTRIG1;

        offset[5] += step[5];
        if ((offset[5]>>16) >= N) offset[5] -= N<<16;
        sample = wavetable[offset[5]>>16];
        sample = sample / 16 + 2048;
        if (sample > 4095) sample = 4095;
        else if(sample < 0) sample = 0;
        DAC->DHR12R1 = sample;
    }
}

//Wave table that produces the sine wave values
void initWavetable(void) {
    int x;
    for(x=0; x < N; x += 1){
         wavetable[x] = 32767 * sin(x * 2 * M_PI / N);
    }
}

//Setup DMA which allows to display and play music at the same time with no interference / lag
void setupDMA1(void) {
    RCC->AHBENR |= RCC_AHBENR_DMA1EN; //set up a DMA
    DMA1_Channel5->CCR &= ~DMA_CCR_EN;
    DMA1_Channel5->CPAR = (uint32_t) (&(DAC->DHR12R1));					//  - The peripheral output is the DAC DHR12R1 register.
    DMA1_Channel5->CMAR = (uint32_t) wavetable; 						//  - The input is the memory region for wavetable
    DMA1_Channel5->CNDTR = sizeof wavetable / sizeof wavetable[0];		//  - The count is the number of elements in wavetable.
    DMA1_Channel5->CCR |=  DMA_CCR_CIRC| DMA_CCR_DIR | DMA_CCR_MINC; 	//  - Ensure it is copying from memory to the peripheral. DIR 1.. CIR CIRC 1... DMA TRIGGER TEIE
    DMA1_Channel5->CCR &= ~(DMA_CCR_MSIZE | DMA_CCR_PSIZE); 
    DMA1_Channel5->CCR |= (DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0);
    DMA1_Channel5->CCR |= DMA_CCR_EN;
}

//Setup timer 15, which drives the DMA
void setupTIM15(void) {
    RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;
    //  - It issues an update event at a desired rate
    TIM15->ARR = 9 - 1;		//HAVE TO BE EXACTLY THE SAME FROM TIM6
    TIM15->PSC = 480 - 1;	//HAVE TO BE EXACTLY THE SAME FROM TIM6
    //  - It triggers a DMA channel on an update event
    TIM15->DIER |= TIM_DIER_UDE;
    //  - Set the MMS field so that the update event is selected for TRGO
    TIM15->CR2 &= ~(TIM_CR2_MMS);
    TIM15->CR2 |= (TIM_CR2_MMS_1);
    //  - Set the auto-reload preload enable flag.
    TIM15->CR1 |= TIM_CR1_ARPE;
    //  - Reconfigure the DAC so it is triggered by Timer 15 TRGO.
    DAC->CR &= ~(DAC_CR_TSEL1);
    DAC->CR |= (DAC_CR_TSEL1_0 | DAC_CR_TSEL1_1);
    //  - Turn off Timer 2.
    RCC->APB1ENR &= ~(RCC_APB1ENR_TIM2EN);
    //  - Enable the timer.
    TIM15->CR1 |= TIM_CR1_CEN;
}

//--------------------------------------------
//Timer that updates the screen
void setup_timer6() {
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;
    TIM6->PSC = 480 - 1;	//HAVE TO BE EXACTLY THE SAME FROM TIM15
    TIM6->ARR = 9 - 1;		//HAVE TO BE EXACTLY THE SAME FROM TIM15
    TIM6->DIER |= TIM_DIER_UIE;
    TIM6->CR1 |= TIM_CR1_CEN;
    NVIC->ISER[0] = 1<<TIM6_DAC_IRQn;
}

//Interrupt 
void TIM6_DAC_IRQHandler() {
    TIM6->SR &= ~TIM_SR_UIF; //SET TO 1 BY HARDWARE WHEN UPDATE OCCURS, MUST CLEAR BY SOFTWARE TO UPDATE AGAIN

    //Lane 1
    if((GPIOB->IDR & GPIO_IDR_6) == GPIO_IDR_6){
        history1++;
        if((shift[0][0] > 48 || shift[0][1] > 48 || shift[0][2] > 48) && (history1 < 100)){
            for(int i = 1; i < 5; i++){
                image[i][62] &= ~(2);
                image[i][61] &= ~(2);
                image[i][60] &= ~(2);
                image[i][59] &= ~(2);
                image[i][58] &= ~(2);
                image[i][57] &= ~(2);
                image[i][56] &= ~(2);
                image[i][55] &= ~(2);
            }
            correctmark = 8;
            correctHits += 1;
            scoreNum();
            if(active[0][0] == 1){
                active[0][0] = 0;
                shift[0][0] = 0;
            }
            else if(active[0][1] == 1){
                active[0][1] = 0;
                shift[0][1] = 0;
            }
            else{
                active[0][2] = 0;
                shift[0][2] = 0;
            }
        }
    }
    else{
        history1 = 0;
    }

    //Lane 2
    if((GPIOB->IDR & GPIO_IDR_5) == GPIO_IDR_5){
        history2++;
        if((shift[1][0] > 48 || shift[1][1] > 48 || shift[1][2] > 48) && (history2 < 100)){
            for(int i = 6; i < 10; i++){
                image[i][62] &= ~(4);
                image[i][61] &= ~(4);
                image[i][60] &= ~(4);
                image[i][59] &= ~(4);
                image[i][58] &= ~(4);
                image[i][57] &= ~(4);
                image[i][56] &= ~(4);
                image[i][55] &= ~(4);
            }
            correctmark = 8;
            correctHits += 1;
            scoreNum();
            if(active[1][0] == 1){
                active[1][0] = 0;
                shift[1][0] = 0;
            }
            else if(active[1][1] == 1){
                active[1][1] = 0;
                shift[1][1] = 0;
            }
            else{
                active[1][2] = 0;
                shift[1][2] = 0;
            }
        }
    }
    else{
        history2 = 0;
    }

    //Lane 3
    if((GPIOB->IDR & GPIO_IDR_4) == GPIO_IDR_4){
        history3++;
        if((shift[2][0] > 48 || shift[2][1] > 48 || shift[2][2] > 48) && (history3 < 100)){
            for(int i = 11; i < 15; i++){
                image[i][62] &= ~(6);
                image[i][61] &= ~(6);
                image[i][60] &= ~(6);
                image[i][59] &= ~(6);
                image[i][58] &= ~(6);
                image[i][57] &= ~(6);
                image[i][56] &= ~(6);
                image[i][55] &= ~(6);
            }
            correctmark = 8;
            correctHits += 1;
            scoreNum();
            if(active[2][0] == 1){
                active[2][0] = 0;
                shift[2][0] = 0;
            }
            else if(active[2][1] == 1){
                active[2][1] = 0;
                shift[2][1] = 0;
            }
            else{
                active[2][2] = 0;
                shift[2][2] = 0;
            }
        }
    }
    else{
        history3 = 0;
    }

    //Lane 4
    if((GPIOB->IDR & GPIO_IDR_15) == GPIO_IDR_15){
        history4++;
        if((shift[3][0] > 48 || shift[3][1] > 48 || shift[3][2] > 48) && (history4 < 100)){
            for(int i = 1; i < 5; i++){
                image[i][62] &= ~(48);
                image[i][61] &= ~(48);
                image[i][60] &= ~(48);
                image[i][59] &= ~(48);
                image[i][58] &= ~(48);
                image[i][57] &= ~(48);
                image[i][56] &= ~(48);
                image[i][55] &= ~(48);
            }
            correctmark = 8;
            correctHits += 1;
            scoreNum();
            if(active[3][0] == 1){
                active[3][0] = 0;
                shift[3][0] = 0;
            }
            else if(active[3][1] == 1){
                active[3][1] = 0;
                shift[3][1] = 0;
            }
            else{
                active[3][2] = 0;
                shift[3][2] = 0;
            }
        }
    }
    else{
        history4 = 0;
    }

    //Lane 5
    if((GPIOB->IDR & GPIO_IDR_14) == GPIO_IDR_14){
        history5++;
        if((shift[4][0] > 48 || shift[4][1] > 48 || shift[4][2] > 48) && (history5 < 100)){
            for(int i = 6; i < 10; i++){
                image[i][62] &= ~(32);
                image[i][61] &= ~(32);
                image[i][60] &= ~(32);
                image[i][59] &= ~(32);
                image[i][58] &= ~(32);
                image[i][57] &= ~(32);
                image[i][56] &= ~(32);
                image[i][55] &= ~(32);
            }
            correctmark = 8;
            correctHits += 1;
            scoreNum();
            if(active[4][0] == 1){
                active[4][0] = 0;
                shift[4][0] = 0;
            }
            else if(active[4][1] == 1){
                active[4][1] = 0;
                shift[4][1] = 0;
            }
            else{
                active[4][2] = 0;
                shift[4][2] = 0;
            }
        }
    }
    else{
        history5 = 0;
    }

    //Lane 6
    if((GPIOB->IDR & GPIO_IDR_13) == GPIO_IDR_13){
        history6++;
        if((shift[5][0] > 44 || shift[5][1] > 44 || shift[5][2] > 44) && (history6 < 100)){
            for(int i = 11; i < 15; i++){
                image[i][62] &= ~(16);
                image[i][61] &= ~(16);
                image[i][60] &= ~(16);
                image[i][59] &= ~(16);
                image[i][58] &= ~(16);
                image[i][57] &= ~(16);
                image[i][56] &= ~(16);
                image[i][55] &= ~(16);
            }
            correctmark = 8;
            correctHits += 1;
            scoreNum();
            if(active[5][0] == 1){
                active[5][0] = 0;
                shift[5][0] = 0;
            }
            else if(active[5][1] == 1){
                active[5][1] = 0;
                shift[5][1] = 0;
            }
            else{
                active[5][2] = 0;
                shift[5][2] = 0;
            }
        }
    }
    else{
        history6 = 0;
    }


    if(correctmark > 0 && buttonPressed()){
        DAC->CR |= (DAC_CR_EN1);
        playSound();
    }
    else{
        DAC->CR &= ~(DAC_CR_EN1);
    }
}

//------------------ SONGS SECTION 	--------------------------

void song1(void)
{
    if(k==500){lane6set();}
    if(k==750){lane6set();}
    if(k==1000){lane2set();}
    if(k==1250){lane2set();}
    if(k==1500){lane1set();}
    if(k==1750){lane1set();}
    if(k==2000){lane2set();}

    if(k==2500){lane3set();}
    if(k==2750){lane3set();}
    if(k==3000){lane4set();}
    if(k==3250){lane4set();}
    if(k==3500){lane5set();}
    if(k==3750){lane5set();}
    if(k==4000){lane6set();}

    if(k==4500){lane2set();}
    if(k==4750){lane2set();}
    if(k==5000){lane3set();}
    if(k==5250){lane3set();}
    if(k==5500){lane4set();}
    if(k==5750){lane4set();}
    if(k==6000){lane5set();}

    if(k==6500){lane2set();}
    if(k==6750){lane2set();}
    if(k==7000){lane3set();}
    if(k==7250){lane3set();}
    if(k==7500){lane4set();}
    if(k==7750){lane4set();}
    if(k==8000){lane5set();}

    if(k==8500){lane6set();}
    if(k==8750){lane6set();}
    if(k==9000){lane2set();}
    if(k==9250){lane2set();}
    if(k==9500){lane1set();}
    if(k==9750){lane1set();}
    if(k==10000){lane2set();}

    if(k==10500){lane3set();}
    if(k==10750){lane3set();}
    if(k==11000){lane4set();}
    if(k==11250){lane4set();}
    if(k==11500){lane5set();}
    if(k==11750){lane5set();}
    if(k==12000){lane6set();}
    if(k==14000){k = 49999;}

}

void song2(void)
{
    if(k==500){lane1set();}
    if(k==750){lane2set();}
    if(k==1000){lane1set();}
    if(k==1250){lane2set();}
    if(k==1500){lane1set();}
    if(k==1750){lane4set();}
    if(k==2000){lane2set();}
    if(k==2250){lane3set();}
    if(k==2500){lane5set();}

    if(k==3200){lane6set();}
    if(k==3450){lane6set();}
    if(k==3700){lane3set();}
    if(k==3950){lane4set();}

    if(k==4650){lane6set();}
    if(k==4900){lane6set();}
    if(k==5150){lane2set();}
    if(k==5400){lane3set();}

    if(k==6100){lane6set();}
    if(k==6350){lane1set();}
    if(k==6600){lane2set();}
    if(k==6850){lane1set();}
    if(k==7100){lane2set();}
    if(k==7350){lane1set();}
    if(k==7600){lane4set();}
    if(k==7850){lane2set();}
    if(k==8100){lane3set();}
    if(k==8350){lane5set();}

    if(k==9050){lane6set();}
    if(k==9300){lane6set();}
    if(k==9550){lane3set();}
    if(k==9800){lane4set();}

    if(k==10500){lane6set();}
    if(k==10750){lane2set();}
    if(k==11000){lane3set();}
    if(k==11250){lane5set();}

    if(k==13250){k = 49999;}

}
void song3(void)
{
    if(k==500){lane6set();}
    if(k==700){lane6set();}
    if(k==950){lane5set();}
    if(k==1250){lane5set();}

    if(k==1750){lane5set();}
    if(k==1950){lane6set();}
    if(k==2050){lane5set();}
    if(k==2300){lane3set();}

    if(k==2700){lane2set();}
    if(k==2850){lane2set();}
    if(k==3100){lane1set();}
    if(k==3350){lane2set();}

    if(k==3650){lane5set();}
    if(k==3850){lane5set();}
    if(k==4050){lane6set();}
    if(k==4250){lane6set();}
    if(k==4450){lane5set();}
    if(k==4600){lane3set();}
    if(k==4800){lane3set();}

    if(k==5300){lane6set();}
    if(k==5500){lane6set();}
    if(k==5700){lane5set();}
    if(k==5950){lane6set();}

    if(k==6300){lane5set();}
    if(k==6500){lane3set();}
    if(k==6700){lane3set();}
    if(k==6900){lane2set();}
    if(k==7100){lane2set();}

    if(k==7500){lane3set();}
    if(k==7700){lane3set();}
    if(k==7900){lane3set();}
    if(k==8100){lane4set();}
    if(k==8300){lane2set();}
    if(k==8550){lane3set();}

    if(k==8900){lane5set();}
    if(k==9100){lane3set();}
    if(k==9300){lane3set();}
    if(k==9500){lane2set();}
    if(k==9650){lane2set();}
    if(k==10000){lane4set();}
    if(k==10200){lane2set();}

    if(k==10400){lane1set();}
    if(k==10900){lane1set();}
    if(k==11100){lane2set();}
    if(k==11300){lane3set();}
    if(k==11500){lane2set();}

    if(k==12000){lane1set();}
    if(k==12200){lane2set();}
    if(k==12400){lane4set();}
    if(k==12900){lane3set();}
    if(k==13000){lane2set();}
    if(k==13200){lane1set();}

    if(k==13600){lane1set();}
    if(k==13800){lane2set();}
    if(k==14000){lane2set();}
    if(k==14200){lane3set();}

    if(k==14600){lane3set();}
    if(k==14750){lane4set();}
    if(k==14900){lane5set();}
    if(k==15100){lane4set();}
    if(k==15300){lane3set();}

    if(k==15600){lane3set();}
    if(k==15750){lane4set();}
    if(k==15900){lane5set();}
    if(k==16100){lane6set();}
    if(k==16500){lane5set();}
    if(k==16700){lane4set();}
    if(k==16900){lane4set();}

    if(k==18900){k = 49999;}
}
void song4(void)
{

    if(k==500){lane2set();}
    if(k==700){lane3set();}
    if(k==900){lane2set();}

    if(k==1300){lane5set();}
    if(k==1500){lane2set();}
    if(k==1700){lane3set();}
    if(k==1900){lane2set();}

    if(k==2300){lane5set();}
    if(k==2500){lane2set();}
    if(k==2700){lane3set();}
    if(k==2900){lane2set();}
    if(k==3300){lane1set();}
    if(k==3700){lane5set();}
    if(k==4100){lane1set();}

    if(k==4600){lane5set();}
    if(k==4800){lane1set();}
    if(k==5000){lane2set();}
    if(k==5200){lane1set();}

    if(k==5600){lane5set();}
    if(k==5800){lane1set();}
    if(k==6000){lane5set();}
    if(k==6200){lane4set();}

    if(k==6600){lane6set();}
    if(k==6800){lane2set();}
    if(k==7000){lane3set();}
    if(k==7200){lane3set();}
    if(k==7600){lane4set();}
    if(k==8000){lane6set();}
    if(k==8400){lane5set();}
 
 //--fast part

    if(k==9200){lane2set();}
    if(k==9350){lane3set();}
    if(k==9500){lane2set();}
    if(k==9650){lane5set();}
    if(k==9800){lane3set();}
    if(k==9950){lane5set();}

    if(k==10100){lane2set();}
    if(k==10250){lane3set();}
    if(k==10400){lane2set();}
    if(k==10550){lane5set();}
    if(k==10700){lane3set();}
    if(k==10850){lane5set();}

    if(k==11000){lane2set();}
    if(k==11150){lane3set();}
    if(k==11300){lane2set();}
    if(k==11450){lane6set();}
    if(k==11600){lane4set();}
    if(k==11750){lane6set();}
    if(k==11900){lane4set();}
    if(k==12050){lane5set();}
    if(k==12200){lane3set();}

    if(k==12600){lane5set();}
    if(k==12750){lane3set();}
    if(k==12900){lane4set();}
    if(k==13050){lane5set();}

    if(k==15050){k = 49999;}

}
void song5(void)
{
    if(k==500){lane4set();}
    if(k==600){lane4set();}
    if(k==700){lane3set();}
    if(k==800){lane2set();}
    if(k==900){lane2set();}
    if(k==1000){lane3set();}
    if(k==1100){lane4set();}
    if(k==1200){lane5set();}
    if(k==1300){lane6set();}
    if(k==1400){lane6set();}
    if(k==1500){lane5set();}
    if(k==1600){lane4set();}
    if(k==1700){lane4set();}
    if(k==2000){lane5set();}
    if(k==2100){lane5set();}

    if(k==2300){lane4set();}
    if(k==2400){lane4set();}
    if(k==2500){lane3set();}
    if(k==2600){lane2set();}
    if(k==2700){lane2set();}
    if(k==2800){lane3set();}
    if(k==2900){lane4set();}
    if(k==3000){lane5set();}
    if(k==3100){lane6set();}
    if(k==3200){lane6set();}
    if(k==3300){lane5set();}
    if(k==3400){lane4set();}
    if(k==3500){lane1set();}
    if(k==3800){lane6set();}
    if(k==3900){lane6set();}

    if(k==4100){lane5set();}
    if(k==4200){lane5set();}
    if(k==4300){lane4set();}
    if(k==4400){lane6set();}
    if(k==4500){lane5set();}
    if(k==4600){lane4set();}
    if(k==4800){lane3set();}
    if(k==4900){lane2set();}
    if(k==5000){lane6set();}
    if(k==5100){lane5set();}
    if(k==5200){lane4set();}
    if(k==5300){lane3set();}
    if(k==5500){lane4set();}
    if(k==5600){lane5set();}
    if(k==5700){lane6set();}
    if(k==5800){lane5set();}
    if(k==5900){lane6set();}

    if(k==6100){lane4set();}
    if(k==6200){lane4set();}
    if(k==6300){lane3set();}
    if(k==6400){lane2set();}
    if(k==6500){lane2set();}
    if(k==6600){lane3set();}
    if(k==6700){lane4set();}
    if(k==6800){lane5set();}
    if(k==6900){lane6set();}
    if(k==7000){lane6set();}
    if(k==7100){lane5set();}
    if(k==7200){lane4set();}
    if(k==7300){lane4set();}
    if(k==7500){lane6set();}
    if(k==7600){lane6set();}

    if(k==9600){k = 49999;}

}
void song6(void)
{
    //my god

    if(k==600){lane1set();}
    if(k==700){lane3set();}
    if(k==800){lane5set();}
    if(k==900){lane4set();}
    if(k==980){lane6set();}

    if(k==1130){lane1set();}
    if(k==1230){lane3set();}
    if(k==1330){lane5set();}
    if(k==1430){lane4set();}
    if(k==1510){lane6set();}

    if(k==1660){lane1set();}
    if(k==1760){lane3set();}
    if(k==1860){lane5set();}
    if(k==1960){lane4set();}
    if(k==2060){lane6set();}

    if(k==2210){lane1set();}
    if(k==2310){lane2set();}
    if(k==2410){lane4set();}
    if(k==2510){lane5set();}

    if(k==2700){lane3set();}
    if(k==2850){lane1set();}
    if(k==3000){lane2set();}
    if(k==3150){lane4set();}
    if(k==3300){lane2set();}
    if(k==3450){lane3set();}

    if(k==3600){lane5set();}
    if(k==3750){lane3set();}
    if(k==3900){lane4set();}
    if(k==4050){lane6set();}
    if(k==4200){lane4set();}
    if(k==4350){lane5set();}

    if(k==4500){lane1set();}
    if(k==4600){lane3set();}
    if(k==4700){lane5set();}
    if(k==4800){lane4set();}
    if(k==4880){lane6set();}

    if(k==5030){lane1set();}
    if(k==5130){lane3set();}
    if(k==5230){lane5set();}
    if(k==5330){lane4set();}
    if(k==5410){lane6set();}

    if(k==5560){lane1set();}
    if(k==5660){lane3set();}
    if(k==5760){lane5set();}
    if(k==5860){lane4set();}
    if(k==5940){lane6set();}

    if(k==6090){lane1set();}
    if(k==6190){lane2set();}
    if(k==6290){lane4set();}
    if(k==6390){lane5set();}
    if(k==6590){lane6set();}
    if(k==6790){lane4set();}
    if(k==6990){lane2set();}
    if(k==7190){lane1set();}

    if(k==9190){k = 49999;}
}

int main(void)
{
    setUpButtons();
    setupDAC();
    initWavetable();
    setupDMA1();
    setupTIM15();

    DAC->CR &= ~(DAC_CR_EN1);

    setup_timer6();
    setup_gpio();
		
	while(1){
		int start = 1;
		int chosenSong = 0;

		while(start)	//Show strtup screen until a button is pressed to choose a song
		{
			draw(0);
			if((GPIOB->IDR & GPIO_IDR_6) == GPIO_IDR_6)    		//Song 6
			{
				start = 0;
				chosenSong = 6;
			}
			else if((GPIOB->IDR & GPIO_IDR_5) == GPIO_IDR_5)  	//Song 5
			{
				start = 0;
				chosenSong = 5;
			}
			else if((GPIOB->IDR & GPIO_IDR_4) == GPIO_IDR_4)    //Song 4
			{
				start = 0;
				chosenSong = 4;
			}
			else if((GPIOB->IDR & GPIO_IDR_15) == GPIO_IDR_15)  //Song 3
			{
				start = 0;
				chosenSong = 3;
			}
			else if((GPIOB->IDR & GPIO_IDR_14) == GPIO_IDR_14)  //Song 2
			{
				start = 0;
				chosenSong = 2;
			}
			else if((GPIOB->IDR & GPIO_IDR_13) == GPIO_IDR_13)  //Song 1
			{
				start = 0;
				chosenSong = 1;
			}
		}

		int play = 1;

		k = 0; //Increase to change when the song starts	
		int count = 0;
		while(play)
		{
			if(chosenSong == 1){song1();}
			else if(chosenSong == 2){song2();}
			else if(chosenSong == 3){song3();}
			else if(chosenSong == 4){song4();}
			else if(chosenSong == 5){song5();}
			else if(chosenSong == 6){song6();}

			if(count == 8){
				//note shift

				shift_notes();
				count = 0;

				//lane 1
				for(int i = 1; i < 5; i++){
						image[i][6] &= ~(2);
				}
				if(lane1 > 0){
					for(int i = 1; i < 5; i++){
							image[i][6] |= 2;
					}
					lane1--;
				}

				//lane 2
				for(int i = 6; i < 10; i++){
						image[i][6] &= ~(4);
				}
				if(lane2 > 0){
					for(int i = 6; i < 10; i++){
							image[i][6] |= 4;
					}
					lane2--;
				}

				//lane 3
				for(int i = 11; i < 15; i++){
						image[i][6] &= ~(6);
				}
				if(lane3 > 0){
					for(int i = 11; i < 15; i++){
							image[i][6] |= 6;
					}
					lane3--;
				}

				//lane 4
				for(int i = 1; i < 5; i++){
						image[i][6] &= ~(48);
				}
				if(lane4 > 0){
					for(int i = 1; i < 5; i++){
							image[i][6] |= 48;
					}
					lane4--;
				}

				//lane 5
				for(int i = 6; i < 10; i++){
						image[i][6] &= ~(32);
				}
				if(lane5 > 0){
					for(int i = 6; i < 10; i++){
							image[i][6] |= 32;
					}
					lane5--;
				}

				//lane 6
				for(int i = 11; i < 15; i++){
						image[i][6] &= ~(16);
				}
				if(lane6 > 0){
					for(int i = 11; i < 15; i++){
							image[i][6] |= 16;
					}
					lane6--;
				}

				if (correctmark > 0){
					for(int i = 0; i < 16; i++){
						image[i][63] = 18;
					}
					correctmark--;
				}
				else{
					for(int i = 0; i < 16; i++){
						image[i][63] = 63;
					}
				}
			}

			draw(1);

			k++;
			if(k > 50000)	//Reset after each game
			{
				correctHits = 0;
				scoreNum();
				k = 0;
				break;
			}

			count++;
		}
		//GPIOC->BSRR = 1<<OE;

	}
}
