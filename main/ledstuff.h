
#ifndef __LEDSTUFF_H__
#define __LEDSTUFF_H__

typedef struct
{
    int ms;
    int strip;
    int mode;
    int red;
    int green;
    int blue;
    int red2;
    int green2;
    int blue2;
    int delay;
} stripAction;

void testActions(stripAction actions[], int size);
void setupPWM();

#endif