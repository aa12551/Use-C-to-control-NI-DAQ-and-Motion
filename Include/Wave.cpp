#include "Wave.h"
#include "math.h"
#include "complex"
#include "iostream"
#include <fstream>
#define DAQmxErrChk(functionCall) if( DAQmxFailed(error=(functionCall)) ) goto Error; else

int OutputChannel = 2;
int IntputChannel = 13;
constexpr auto pi = 3.1415926;
void CheckError(int32 error);
float64* linspace(float64 first, float64 last, int num,int channel_num) // 創建一個由first到last，且有num個的陣列，重複channel_num次
{
	float64 * Data = new float64[num*channel_num];
	float64 interval = (last - first) / (num - 1);
	for (int j = 0; j < channel_num; j++)
	{
		for (int i = 0; i < num; i++)
		{
			Data[i+j*num] = first + i * interval;
		}
	}
	return Data;
}

float64* hanning_win(float64 * Data,int DataPoint) // Hanning Window
{
	float64 * hanning = new float64[DataPoint];
	for (int i = 0; i < DataPoint; i++)
	{
		hanning[i] = Data[i] * 0.5*(1 - cos(2*pi*i / (DataPoint-1)));
	}
	return hanning;
}

float64* SineWave(int DataPoint, float64 cycle, float64 amplitube) //創建出具有cycle個sin波，且有DataPoint個取樣點
{
	float64 * Sine_Y = new float64[DataPoint];
	float64 * Sine_X = linspace(0, DataPoint - 1, DataPoint,1);
	for (int i = 0; i < DataPoint; i++)
	{
		Sine_Y[i] = amplitube * sin(2 * pi*Sine_X[i] * cycle / DataPoint);
	}
	return Sine_Y;
}

Wave_XY* HandMade_WaveForm(double amplitube, int DataPoint, double Wave_Packet_frequency, int cycles, double trigger_delay, double trigger_interval,int channel_num)
{
	// 計算各項參數
	float64 period_time = trigger_interval * 2;
	float64 total_frequency = DataPoint / period_time;
	float64 Wave_Packet_Point = total_frequency / Wave_Packet_frequency;
	float64 last_half_point = (period_time - trigger_delay)*total_frequency-Wave_Packet_Point;
	float64 first_half_point = trigger_delay * total_frequency;
	
	// 創建整個訊號的記憶體空間
	float64 * Wave = new float64[DataPoint*channel_num];

	// 做出中間的波型
	float64 * Sine = SineWave(Wave_Packet_Point, cycles, amplitube);
	float64 * hanning = hanning_win(Sine, Wave_Packet_Point);

	// 多重 channel output 需要把所有channel要輸出的data矩陣合併
	for (int k = 0; k < channel_num; k++)
	{
		//前半點與後半點補0
		for (int i = 0; i < first_half_point; i++)
		{
			Wave[i+k*DataPoint] = 0;
		}
		for (int i = first_half_point + Wave_Packet_Point; i < DataPoint; i++)
		{
			Wave[i+k*DataPoint] = 0;
		}
		// 中間補上Sine訊號
		int j = 0;
		for (int i = first_half_point; i < first_half_point + Wave_Packet_Point; i++)
		{
			Wave[i+k*DataPoint] = hanning[j];
			j++;
		}
	}
	
	// 補上x的資料
	float64 * x = linspace(0, period_time, DataPoint,channel_num);

	Wave_XY* output;
	output = new Wave_XY;
	output->X = x;
	output->Y = Wave;
	return output;
	delete[] Sine;
	delete[] hanning;
}

TaskHandle AO_set()
{
	// error code
	int32 error = 0;
	// create a new Task
	TaskHandle Task;
	// AOVoltage parameter
	const char physicalChannel[20] = "PXI1Slot5/ao0:1";
	float64 minVal = -10;
	float64 maxVal = 10;
	int32 terminalConfig = DAQmx_Val_RSE;
	int32 units = DAQmx_Val_Volts;
	// ClockTiming parameter
	float64 rate = 1000000;
	int32 sampleMode = DAQmx_Val_FiniteSamps;
	uInt64 sampsPerChanToAcquire = PerChannelData;
	int32 activeEdge = DAQmx_Val_Rising;
	// AnlgEdge parameter
	int32 triggerSlope = DAQmx_Val_Rising;
	const char triggerSource[15] = "APFI0";
	float64 triggerLevel = 0;
	

	error = DAQmxCreateTask("", &Task);
	CheckError(error);
	error = DAQmxCreateAOVoltageChan(Task, physicalChannel,"", minVal, maxVal, units, NULL); // 少一個參數terminal configuration
	CheckError(error);
	error = DAQmxCfgSampClkTiming(Task, "" , rate, activeEdge, sampleMode, sampsPerChanToAcquire);
	CheckError(error);
	//error = DAQmxCfgAnlgEdgeStartTrig(Task, triggerSource, triggerSlope,triggerLevel);
	//CheckError(error);
	return Task;
}

TaskHandle AI_set()
{
	// error code
	int32 error;
	// create a new task
	TaskHandle Task = 0;
	// AIVoltage parameter
	const char physicalChannel[20] = "PXI1Slot5/ai0:12";
	float64 minVal = -10;
	float64 maxVal = 10;
	int32 terminalConfig = DAQmx_Val_Diff;
	int32 units = DAQmx_Val_Volts;
	// ClockTiming parameter
	float64 rate = 1000000;
	int32 sampleMode = DAQmx_Val_FiniteSamps;
	uInt64 sampsPerChanToAcquire = PerChannelData;
	int32 activeEdge = DAQmx_Val_Rising;
	// AnlgEdge parameter
	int32 triggerSlope = DAQmx_Val_Rising;
	const char triggerSource[15] = "APFI0";
	float64 triggerLevel = 0.25;

	error = DAQmxCreateTask("", &Task);
	CheckError(error);
	error = DAQmxCreateAIVoltageChan(Task, physicalChannel,"",terminalConfig, minVal, maxVal, units, NULL);
	CheckError(error);
	error = DAQmxCfgSampClkTiming(Task, "", rate, activeEdge, sampleMode, sampsPerChanToAcquire);
	CheckError(error);
	//error = DAQmxCfgAnlgEdgeStartTrig(Task, triggerSource, triggerSlope, triggerLevel);
	//CheckError(error);
	return Task;
}

void Normalize(float64 * wave, int channel, int perchanneldata)
{
	for (int i = 0; i < channel; i++)
	{
		int DC = wave[i*perchanneldata];
		for (int j = 0; j < channel; j++)
		{
			wave[j] = wave[j] / DC;
		}
	}
}

Wave_XY* AI_AO()
{
	Wave_XY* wave;
	wave = new Wave_XY;
	int32 error;
	// create wave parameter
	double Amplitube = 10;
	int DataPoint = PerChannelData;
	double Wave_Packet_Frequency = 100;
	int Cycles = 5;
	int channel = Channel_num;
	double Trigger_Delay = 0.005;
	double Trigger_Interval = 0.02;
	// Write parameter
	bool32 dataLayout = DAQmx_Val_GroupByChannel;
	// make the waveform
	wave = HandMade_WaveForm(Amplitube, DataPoint, Wave_Packet_Frequency, Cycles, Trigger_Delay, Trigger_Interval,OutputChannel);
	// set the output and input channel parameter
	TaskHandle OutTask = AO_set();
	TaskHandle InTask = AI_set();
	// write into the output channel
	error = DAQmxWriteAnalogF64(OutTask, DataPoint, false, 10.0, dataLayout, wave->Y, NULL, NULL);
	CheckError(error);
	// start the Output task
	error = DAQmxStartTask(OutTask);
	CheckError(error);
	// Read the input channel
	// parameter
	bool32 fillMode = DAQmx_Val_GroupByChannel;
	// allocate memory and initialize
	float64 *readarray = new float64[DataPoint*IntputChannel];
	for (int i = 0; i < DataPoint*IntputChannel; i++)
	{
		readarray[i] = 0;
	}
	int32 totalRead = 0; // per channel read sample
	int32 numSampsPerChanRead = DAQmx_Val_Auto;
	error = DAQmxStartTask(InTask);
	CheckError(error);
	error = DAQmxReadAnalogF64(InTask, DataPoint, 10, fillMode, readarray, DataPoint*channel, &totalRead, NULL);
	CheckError(error);
	std::cout << totalRead << std::endl;
	error = DAQmxClearTask(OutTask);
	CheckError(error);
	error = DAQmxClearTask(InTask);
	CheckError(error);
	// 給圖輸出的x座標
	wave->X = linspace(0, DataPoint*13-1, DataPoint*13,1);
	wave->Y = readarray;
	return wave;
}

void CheckError(int32 err)
{
	if (err != 0)
	{
		char errorString[400];
		DAQmxGetErrorString(err, errorString, 400);
		std::cout << err << std::endl;
		std::cout << errorString << std::endl;
	}
}

//std::complex<float64>* WaveToComplex(float64*wave)
//{
//	const int DataPoint = PerChannelData;
//	std::complex<float64> * x = new (std::complex<float64>*)[DataPoint];
//	return x;
//}

void FFT(int N, std::complex<float64>* x)
{
	/* bit-reversal permutation */
	for (int i = 1, j = 0; i < N; ++i)
	{
		for (int k = N >> 1; !((j ^= k)&k); k >>= 1);
		if (i > j) swap(x[i], x[j]);
	}

	/* dynamic programming */
	for (int k = 2; k <= N; k <<= 1)
	{
		float theta = -2.0 * 3.14159 / k;
		std::complex<float64> delta_w(cos(theta), sin(theta));

		// 每k個做一次FFT
		for (int j = 0; j < N; j += k)

		{
			// 前k/2個與後k/2的三角函數值恰好對稱，
			// 因此兩兩對稱的一起做。
			std::complex<float64> w(1, 0);
			for (int i = j; i < j + k / 2; i++)
			{
				std::complex<float64> a = x[i];
				std::complex<float64> b = x[i + k / 2] * w;
				x[i] = a + b;
				x[i + k / 2] = a - b;
				w *= delta_w;
			}
		}
	}
	//scale
	for (int i = 0; i < N; i++)
	{
		x[i] /= sqrt(N);
	}
}

float64** SignalProcess(Wave_XY* wave)
{
	const int GetLength = 512;
	float64 **TrainData = new float64*[Channel_num];
	for (int i = 0; i < PerChannelData; i++)
		TrainData[i] = new float64[GetLength];

	for (int i = 0; i < Channel_num; i++)
	{
		for (int j = 0; j < GetLength; j++)
		{
			TrainData[i][j] = wave->Y[49 + j + i * PerChannelData];
		}
	}
	return TrainData;
}

void OutputData(float64 ** Data)
{
	std::fstream file;
	file.open("write.csv");
	for (int i = 0; i < Channel_num; i++)
	{
		file << Data[i] << ",";
	}
	file.close();
}