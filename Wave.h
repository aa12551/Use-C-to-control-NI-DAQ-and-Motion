#pragma once
#ifndef WAVE_H
#define WAVE_H
#include "NIDAQmx.h"
const int PerChannelData = 4000;
const int Channel_num = 13;
typedef struct Wave_XY {
	float64 *X;
	float64 *Y;
}Wave_XY;
Wave_XY* HandMade_WaveForm(double amplitube, int DataPoint, double Wave_Packet_frequency, int cycles, double trigger_delay, double trigger_interval,int channel_num);
Wave_XY* AI_AO();
void Normalize(float64 * wave, int channel, int perchanneldata);

#endif // !WAVE_H