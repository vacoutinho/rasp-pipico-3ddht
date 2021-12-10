/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <math.h>
#include <stdint.h>

#include "pico/stdlib.h"

#define    DHT_exact_ID 0
#define    DHT_8_ID     1
#define    DHT_11_ID    2
#define    DHT_12_ID    3
#define    DHT_16_ID    4
#define    DCT_exact_ID 5

#define CURRENT_USED_TRANSFORM (DCT_exact_ID)

#define COUNT_3D_BLOCKS        8192
#define EXPERIMENTE_ITERATIONS 20


uint32_t myGetTimeStamp(void);

void my3D_transform(int32_t *tensor3d_in, int32_t *tensor3d_out);
void my1DDCT_transform(int32_t *vector);

int main() {

    const uint LED_PIN = PICO_DEFAULT_LED_PIN;
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    stdio_init_all();

    int32_t tensor3d_out[8][8][8], tensor3d_in[8][8][8] = { 
        {
            {59,   22,   60,   88,   98,   87,   21,   80},
            {13,   68,   53,   27,   24,   66,   72,   48},
            {10,   26,   37,   27,   23,   58,    0,   95},
            {19,   12,   42,   13,   74,   80,   32,   46},
            {30,   84,   11,   38,   55,   96,   76,   69},
            { 0,   80,   86,   10,   33,   52,    4,   74},
            {52,   54,   10,   26,   16,   41,   93,   95},
            {43,   76,   75,   33,   90,   61,   96,   57}
        },
        {
            {15,   21,   24,   67,   30,   90,   90,   27},
            {61,   30,   19,   98,   52,    5,   97,   14},
            {36,   89,   89,   50,   97,   85,   35,   17},
            { 7,   55,   33,   59,   14,   90,   81,   87},
            {41,   87,   91,   23,   18,   39,   27,   11},
            { 2,   28,   88,   75,   57,   56,   50,   40},
            {53,   12,   90,   25,   35,   38,   74,   55},
            {53,   89,   69,   28,   12,   46,   19,   81}

        },
        {
           { 3,   15,   60,   82,   61,   38,   53,   40},
           {21,   59,   38,   63,   34,   39,   92,   22},
           {43,   22,   46,   56,   26,   12,   46,    2},
           {22,   83,   30,    6,   65,   50,   90,   69},
           {23,    8,   63,   34,   30,   43,    1,   57},
           { 1,   89,   83,   12,   20,   82,   59,   11},
           {70,   48,   52,   98,   67,   16,   34,   20},
           {96,   85,   72,   32,   23,    7,   67,   35}
        },
        {
            {45,    21,     0,    71,    16,    44,    52,    90},
            {85,    31,    28,    97,     9,    67,    85,    75},
            {92,    69,    29,    20,    82,    26,    10,    51},
            { 8,    25,    92,    84,    57,    61,    51,    89},
            {77,    77,    48,    86,    54,    89,    16,     2},
            {85,    13,    31,     8,    53,    68,   100,    35},
            {42,    79,    92,    72,    12,    16,    49,    75},
            {40,     5,    71,    39,    61,    42,    24,    87}
        },
        {
            {22,   35,   77,   16,   52,   78,   72,   57},
            {82,   61,   16,   76,    4,   99,   77,   52},
            {34,   74,   71,   53,   99,   56,    1,    5},
            {25,   89,   97,   31,   34,   40,   82,   69},
            {95,   81,   57,   22,   47,   30,   72,   36},
            {85,   66,   98,   61,   76,   74,   77,   40},
            {78,   54,   24,   25,   83,   44,   53,   22},
            { 9,    8,   64,   55,    0,   31,   71,   53}

        },
        {
            {81,    9,   58,   14,   19,   28,   54,   69},
            { 8,   34,   62,    5,   29,   38,   79,   25},
            {48,   19,   14,   97,   34,    5,    0,    5},
            {45,   61,   51,   89,   20,   90,   43,   45},
            { 4,   90,   70,   81,   47,   91,   62,   74},
            {47,   82,   23,    2,   84,   94,   55,   63},
            {49,   76,   18,   64,   67,   44,   35,   49},
            {34,   64,   89,   10,   39,   33,   80,   61}

        },
        {
            {83,   57,   84,   52,   76,  85,   86,   79},
            {99,   44,   45,   34,   30,  38,   47,   75},
            {13,   67,   62,   92,   41,  14,   99,   37},
            {46,   47,   99,   21,   80,   8,   40,   40},
            {16,   98,   87,   83,   22,  40,   36,    1},
            {22,   58,   14,   69,   37,  44,   50,   18},
            {38,   46,   53,   41,    8,   2,   85,   98},
            {45,    6,   35,   78,   74,  49,   62,   18}

        },
        {
            {40,   82,   66,   11,   45,   48,   82,   64},
            {50,   98,   10,   10,   19,   49,   32,   62},
            {55,   29,   43,   22,   42,   96,   41,    8},
            {87,   68,   38,   25,   69,   10,   80,   52},
            { 3,   24,   89,   22,   89,   82,   28,   71},
            {13,   34,   88,   69,   42,   72,   97,   42},
            {44,   50,   42,   93,    7,   82,   91,   44},
            {30,   59,   68,    9,   55,    7,    0,   82}

        },
    };

        
    uint8_t led_state = 1;
    uint32_t elapsed_time = 0, t1 = 0, t0 = 0;

    for (int k = 0; k < EXPERIMENTE_ITERATIONS; k++){
        gpio_put(LED_PIN, led_state);
        led_state = led_state ? 0 : 1;
        t0 = myGetTimeStamp();
        for(int i = 0; i<COUNT_3D_BLOCKS; i++) 
            my3D_transform(&tensor3d_in[0][0][0], &tensor3d_out[0][0][0]);
        t1 = myGetTimeStamp();
        printf("\n\rit%d:time = %d - %d = %d (ms)\n\r",k, t1, t0, t1-t0);
        elapsed_time += t1 - t0;
    }
    
    elapsed_time = elapsed_time / EXPERIMENTE_ITERATIONS;

    while (true) {
        printf("\n\rprocessing time = %d (ms)\n\r",elapsed_time);

        gpio_put(LED_PIN, 0);
        sleep_ms(250);
        gpio_put(LED_PIN, 1);
        sleep_ms(250);
    }
    return 0;
}

uint32_t myGetTimeStamp(void)
{
    uint32_t ts;
    ts = to_ms_since_boot(get_absolute_time());
    return ts;
}


// 3D transforms (for 3D DHT-like and 3D DCT) ################################################

void my3D_transform_rows(int32_t *tensor3d);
void my3D_transform_transposition(int32_t *tensor3d_in, int32_t *tensor3d_out);
void my1DDHT_transform(int32_t *vector);
void my1DDCT_transform(int32_t *vector);
void my3DDHT_from_3DSDHT(int32_t *tensor3d_in, int32_t *tensor3d_out);

void my3D_transform(int32_t *tensor3d_in, int32_t *tensor3d_out)
{
    #if CURRENT_USED_TRANSFORM == DCT_exact_ID
        my3D_transform_rows(tensor3d_in);
        my3D_transform_transposition(tensor3d_in, tensor3d_out);
        my3D_transform_rows(tensor3d_out);
        my3D_transform_transposition(tensor3d_out, tensor3d_in);
        my3D_transform_rows(tensor3d_in);
        my3D_transform_transposition(tensor3d_in, tensor3d_out);
    #else
        int32_t tensor_buffer[8][8][8];
    
        my3D_transform_rows(tensor3d_in);
        my3D_transform_transposition(tensor3d_in, &tensor_buffer[0][0][0]);
        my3D_transform_rows(&tensor_buffer[0][0][0]);
        my3D_transform_transposition(&tensor_buffer[0][0][0], tensor3d_in);
        my3D_transform_rows(tensor3d_in);
        my3D_transform_transposition(tensor3d_in, &tensor_buffer[0][0][0]);
    
        my3DDHT_from_3DSDHT(&tensor_buffer[0][0][0], tensor3d_out);
    #endif
 }

void my3D_transform_transposition(int32_t *tensor3d_in, int32_t *tensor3d_out)
{
    for (int n1 = 0; n1<8; n1++)
        for (int n2 = 0; n2<8; n2++)
            for (int n3 = 0; n3<8; n3++)
                *(tensor3d_out + n2 + 8*n3 + 64*n1) = *(tensor3d_in + n1 + 8*n2 + 64*n3);
}

void my3D_transform_rows(int32_t *tensor3d)
{
    for (int n1 = 0; n1<8; n1++)
        for (int n2 = 0; n2<8; n2++) {
            #if CURRENT_USED_TRANSFORM == DCT_exact_ID
                my1DDCT_transform(tensor3d + 8*n1 + 64*n2);
            #else
                my1DDHT_transform(tensor3d + 8*n1 + 64*n2);
            #endif
        }
            
}

void my3DDHT_from_3DSDHT(int32_t *tensor3d_in, int32_t *tensor3d_out)
{
    for (int n1 = 0; n1<8; n1++)
        for (int n2 = 0; n2<8; n2++)
            for (int n3 = 0; n3<8; n3++)
                *(tensor3d_out + n1 + 8*n2 + 64*n3) =   (
                                                            *(tensor3d_in + (8 - n1) % 8  + 8*n2 + 64*n3) + \
                                                            *(tensor3d_in + n1 + 8*((8 - n2) % 8) + 64*n3) + \
                                                            *(tensor3d_in + n1 + 8*n2 + 64*((8 - n3) % 8)) - \
                                                            *(tensor3d_in + (8 - n1) % 8  + 8*((8 - n2) % 8) + 64*((8 - n3) % 8)) \
                                                        ) >> 1;
}


// 1D DHT-like transforms ################################################


void my1DDHT_adder_stage1(int32_t *vector, int32_t *ret_vector);
void my1DDHT_multiplier_stage(int32_t *vector);
void my1DDHT_adder_stage2(int32_t *vector, int32_t *ret_vector);
void my1DDHT_adder_stage3(int32_t *vector, int32_t *ret_vector);

void my1DDHT_transform(int32_t *vector)
{
    int32_t tempVector[8], tempVector2[8];

    my1DDHT_adder_stage1(vector, tempVector);
    my1DDHT_multiplier_stage(tempVector);
    my1DDHT_adder_stage2(tempVector, tempVector2);
    my1DDHT_adder_stage3(tempVector2, vector);
}

void my1DDHT_adder_stage1(int32_t *vector, int32_t *ret_vector)
{
    ret_vector[0] = vector[0] + vector[4];
    ret_vector[1] = vector[0] - vector[4];

    ret_vector[2] = vector[2] + vector[6];
    ret_vector[3] = vector[2] - vector[6];

    ret_vector[4] = vector[1] + vector[5];
    ret_vector[5] = vector[1] - vector[5];

    ret_vector[6] = vector[3] + vector[7];
    ret_vector[7] = vector[3] - vector[7];
}

void my1DDHT_multiplier_stage(int32_t *vector)
{
    #if CURRENT_USED_TRANSFORM == DHT_exact_ID
        float float5, float7;
        float5 = sqrt(2) * ( (float) vector[5] );
        float7 = sqrt(2) * ( (float) vector[7] );
        vector[5] = (int32_t) float5;
        vector[7] = (int32_t) float7;
    #elif CURRENT_USED_TRANSFORM == DHT_8_ID
        //do nothing
    #elif CURRENT_USED_TRANSFORM == DHT_11_ID
        vector[5] = vector[5] + (vector[5] >> 2) + (vector[5] >> 3); //multiplication by (1 + 1/4 + 1/8)
        vector[7] = vector[7] + (vector[7] >> 2) + (vector[7] >> 3);

    #elif CURRENT_USED_TRANSFORM == DHT_12_ID
        vector[5] = vector[5] + (vector[5] >> 1); //multiplication by (1 + 1/2)
        vector[7] = vector[7] + (vector[7] >> 1);
    #elif CURRENT_USED_TRANSFORM == DHT_16_ID
        vector[5] = vector[5] << 1; //multiplication by (2)
        vector[7] = vector[7] << 1;
        //do nothing
    #endif
}

void my1DDHT_adder_stage2(int32_t *vector, int32_t *ret_vector)
{
    ret_vector[0] = vector[0] + vector[2];
    ret_vector[1] = vector[1] + vector[3];
    ret_vector[2] = vector[0] - vector[2];
    ret_vector[3] = vector[1] - vector[3];
    ret_vector[4] = vector[4] + vector[6];
    ret_vector[5] = vector[5];
    ret_vector[6] = vector[4] - vector[6];
    ret_vector[7] = vector[7];
}

void my1DDHT_adder_stage3(int32_t *vector, int32_t *ret_vector)
{
    ret_vector[0] = vector[0] + vector[4];
    ret_vector[1] = vector[1] + vector[5];
    ret_vector[2] = vector[2] + vector[6];
    ret_vector[3] = vector[3] + vector[7];
    ret_vector[4] = vector[0] - vector[4];
    ret_vector[5] = vector[1] - vector[5];
    ret_vector[6] = vector[2] - vector[6];
    ret_vector[7] = vector[3] - vector[7];
}

// 1D DCT transform -- Loeffler Algorithm ###############################################
#define pi 3.14159265f

void my1DDCT_multiplier_block(int32_t in0, int32_t in1, int32_t *out0, int32_t *out1, float k, float n);
void my1DDCT_stage1(int32_t *vector, int32_t *ret_vector);
void my1DDCT_stage2(int32_t *vector, int32_t *ret_vector);
void my1DDCT_stage3(int32_t *vector, int32_t *ret_vector);
void my1DDCT_stage4(int32_t *vector, int32_t *ret_vector);

void my1DDCT_transform(int32_t *vector)
{
    int32_t tempVector[8];

    my1DDCT_stage1(vector, tempVector);
    my1DDCT_stage2(tempVector, vector);
    my1DDCT_stage3(vector, tempVector);
    my1DDCT_stage4(tempVector, vector);
}

void my1DDCT_multiplier_block(int32_t in0, int32_t in1, int32_t *out0, int32_t *out1, float k, float n)
{
    float out_float0, out_float1, in0_k, in1_k, cos_x, sin_x;
    
    in0_k = ((float) in0 ) * k;
    in1_k = ((float) in1 ) * k;
    cos_x = cos(n*pi/16);
    sin_x = sin(n*pi/16);

    out_float0 = in0_k * cos_x + in1_k * sin_x;
    out_float1 = -in0_k * sin_x + in1_k * cos_x;

    *out0 = (int32_t) out_float0;
    *out1 = (int32_t) out_float1;
}

void my1DDCT_stage1(int32_t *vector, int32_t *ret_vector)
{
    ret_vector[0] = vector[7] + vector[0];
    ret_vector[1] = vector[6] + vector[1];
    ret_vector[2] = vector[5] + vector[2];
    ret_vector[3] = vector[4] + vector[3];
    ret_vector[4] = vector[3] - vector[4];
    ret_vector[5] = vector[2] - vector[5];
    ret_vector[6] = vector[1] - vector[6];
    ret_vector[7] = vector[0] - vector[7];
}

void my1DDCT_stage2(int32_t *vector, int32_t *ret_vector)
{
    ret_vector[0] = vector[3] + vector[0];
    ret_vector[1] = vector[2] + vector[1];
    ret_vector[2] = vector[1] - vector[2];
    ret_vector[3] = vector[0] - vector[3];

    my1DDCT_multiplier_block(vector[4], vector[7], &ret_vector[4], &ret_vector[7], 1.0f, 3.0f);
    my1DDCT_multiplier_block(vector[5], vector[6], &ret_vector[5], &ret_vector[6], 1.0f, 1.0f);
}

void my1DDCT_stage3(int32_t *vector, int32_t *ret_vector)
{
    ret_vector[0] = vector[0] + vector[1];
    ret_vector[1] = vector[0] - vector[1];
    
    my1DDCT_multiplier_block(vector[2], vector[3], &ret_vector[2], &ret_vector[3], sqrt(2), 1.0f);

    ret_vector[4] = vector[4] + vector[6];
    ret_vector[5] = vector[7] - vector[5];
    ret_vector[6] = vector[4] - vector[6];
    ret_vector[7] = vector[5] + vector[7];
}

void my1DDCT_stage4(int32_t *vector, int32_t *ret_vector)
{
    ret_vector[0] = vector[0];
    ret_vector[4] = vector[1];
    ret_vector[2] = vector[2];
    ret_vector[6] = vector[3];

    ret_vector[7] = vector[7] - vector[4];
    ret_vector[3] = (int32_t) (sqrt(2) * ( (float) vector[5]) );
    ret_vector[5] = (int32_t) (sqrt(2) * ( (float) vector[6]) );
    ret_vector[1] = vector[4] + vector[7];
}