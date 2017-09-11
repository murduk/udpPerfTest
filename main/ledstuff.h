
#ifndef __LEDSTUFF_H__
#define __LEDSTUFF_H__

typedef struct
{
    int ms;
    unsigned char strip;
    unsigned char mode;
    unsigned char red;
    unsigned char green;
    unsigned char blue;
    int delay;
} stripAction;

void testActions(stripAction actions[], int size);
void setupPWM();

#endif