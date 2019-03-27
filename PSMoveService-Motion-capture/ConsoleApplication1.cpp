#include "stdafx.h"
#include <windows.h>
#include "PSMoveService/PSMoveClient_CAPI.h"
#include <chrono>
#include <fstream>
#include <time.h>

typedef struct
{
	double yaw, pitch, roll;
} DeviceRotation;

PSMControllerList controllerList;

PSMVector3f ctrlPos[32];
PSMQuatf ctrlRot[32];
PSMController *ctrl[32];
DeviceRotation ctrlOffset[32];

std::ofstream MotionCapture;

static const float k_fScalePSMoveAPIToMeters = 0.01f; // psmove driver in cm

bool PSMConnected = false, PSMoveServiceInit = false;
std::thread *pPSMUpdatethread = NULL;

bool DataRecord = false;
bool Centering = false;
int FPS = 60;

double OffsetYPR(float f, float f2)
{
	f -= f2;
	if (f < -180) {
		f += 360;
	}
	else if (f > 180) {
		f -= 360;
	}

	return f;
}

double RadToDeg(double r) {
	return r * (180 / 3.14159265358979323846); //180 / PI
}

DeviceRotation QuatToYPR(double QuatW, double QuatX, double QuatY, double QuatZ) {
	// roll (x-axis rotation)
	DeviceRotation OutRot;
	double sinr_cosp = 2.0 * (QuatW * QuatX + QuatY * QuatZ);
	double cosr_cosp = 1.0 - 2.0 * (QuatX * QuatX + QuatY * QuatY);
	OutRot.roll = RadToDeg(atan2(sinr_cosp, cosr_cosp));

	// pitch (y-axis rotation)
	double sinp = 2.0 * (QuatW * QuatY - QuatZ * QuatX);
	if (fabs(sinp) >= 1)
		OutRot.pitch = RadToDeg(copysign(3.14159265358979323846 / 2, sinp)); // use 90 degrees if out of range
	else
		OutRot.pitch = RadToDeg(asin(sinp));

	// yaw (z-axis rotation)
	double siny_cosp = 2.0 * (QuatW * QuatZ + QuatX * QuatY);
	double cosy_cosp = 1.0 - 2.0 * (QuatY * QuatY + QuatZ * QuatZ);
	OutRot.yaw = RadToDeg(atan2(siny_cosp, cosy_cosp));

	return OutRot;
}

void PSMoveServiceUpdate()
{
	while (PSMConnected) {
		PSM_Update();

		system("cls");

		printf("\r\n\ Current FPS - %d", FPS);

		if (DataRecord) {
			SetConsoleTitle(_T("PSMoveService Motion capture: recording"));
			printf(", Recording on\r\n");
		}
		else {
			SetConsoleTitle(_T("PSMoveService Motion capture"));
			printf(", Recording off\r\n");
		}

		for (int i = 0; i < controllerList.count; i++)
		{
			PSM_GetControllerPosition(controllerList.controller_id[i], &ctrlPos[i]);
			PSM_GetControllerOrientation(controllerList.controller_id[i], &ctrlRot[i]);
			
			DeviceRotation OutRot = { 0, 0, 0 };
			OutRot = QuatToYPR(ctrlRot[i].w, ctrlRot[i].x, ctrlRot[i].y, ctrlRot[i].z);

			if (Centering) {
				ctrlOffset[i].yaw = OutRot.yaw;
				ctrlOffset[i].pitch = OutRot.pitch;
				ctrlOffset[i].roll = OutRot.roll;
			}

			ctrl[i] = PSM_GetController(controllerList.controller_id[i]);
			printf(" %d %5.2f %5.2f %5.2f %5.2f %5.2f %5.2f\r\n", i, ctrlPos[i].x  * k_fScalePSMoveAPIToMeters, ctrlPos[i].y  * k_fScalePSMoveAPIToMeters, ctrlPos[i].z * k_fScalePSMoveAPIToMeters, OutRot.yaw, OutRot.pitch, OutRot.roll);
		
			if (DataRecord) {
				MotionCapture << i;
				MotionCapture << " ";
				MotionCapture << ctrlPos[i].x  * k_fScalePSMoveAPIToMeters;
				MotionCapture << " ";
				MotionCapture << ctrlPos[i].y  * k_fScalePSMoveAPIToMeters;
				MotionCapture << " ";
				MotionCapture << ctrlPos[i].z  * k_fScalePSMoveAPIToMeters;
				MotionCapture << " ";
				MotionCapture << OffsetYPR(OutRot.yaw, ctrlOffset[i].yaw);
				MotionCapture << " ";
				MotionCapture << OffsetYPR(OutRot.pitch, ctrlOffset[i].pitch);
				MotionCapture << " ";
				MotionCapture << OffsetYPR(OutRot.roll, ctrlOffset[i].roll);

				MotionCapture << std::endl;
			}
		}

		if (Centering)
			Centering = false;

		printf("\r\n F2 or Cross - record on, F3 or Circle record off.\r\n");
		printf(" Numpad 6, 3, 1 - change FPS record.\r\n");
		printf("\r\n F6 ord Move - centering. Escape - close.\r\n");

		std::this_thread::sleep_for(std::chrono::milliseconds((1 / FPS) * 1000)); 

	}
}

void ConnectToPSMoveService()
{
	if (PSM_Initialize(PSMOVESERVICE_DEFAULT_ADDRESS, PSMOVESERVICE_DEFAULT_PORT, PSM_DEFAULT_TIMEOUT) == PSMResult_Success)
	{
		memset(&controllerList, 0, sizeof(PSMControllerList));
		PSM_GetControllerList(&controllerList, PSM_DEFAULT_TIMEOUT);
	}

	unsigned int data_stream_flags =
		PSMControllerDataStreamFlags::PSMStreamFlags_includePositionData |
		PSMControllerDataStreamFlags::PSMStreamFlags_includePhysicsData |
		PSMControllerDataStreamFlags::PSMStreamFlags_includeCalibratedSensorData |
		PSMControllerDataStreamFlags::PSMStreamFlags_includeRawTrackerData;

	for (int i = 0; i < controllerList.count; i++)
	{
		if (PSM_AllocateControllerListener(controllerList.controller_id[i]) == PSMResult_Success && PSM_StartControllerDataStream(controllerList.controller_id[i], data_stream_flags, PSM_DEFAULT_TIMEOUT) == PSMResult_Success)
			PSMConnected = true;
	}

	if (PSMConnected)
		pPSMUpdatethread = new std::thread(PSMoveServiceUpdate);
}

int main()
{
	SetConsoleTitle(_T("PSMoveService Motion capture"));

	ConnectToPSMoveService();
	
	if (PSMConnected) {
		printf("PSM connected\r\n");
	} else {
		printf("PSM not connected\r\n");
	}

	TCHAR FileName[511] = _T("MotionCapture");

	//_tcscpy(, );
	TCHAR szTime[15] = { 0 };

	SYSTEMTIME sysTime;
	GetLocalTime(&sysTime);
	_stprintf(szTime, _T("_%02d.%02d.%d_%02d.%02d.txt"), sysTime.wDay, sysTime.wMonth, sysTime.wYear,sysTime.wHour, sysTime.wMinute);

	//_tcscat_s(FileName, sizeof(FileName), _T(".txt"));
	_tcscat_s(FileName, sizeof(FileName), szTime);

	//MessageBox(NULL, FileName, FileName, MB_OK);

	MotionCapture.open(FileName);
	if (MotionCapture.is_open())
	{
		MotionCapture << controllerList.count << std::endl;
	}

	printf("Controllers count = %d\r\n");

	Sleep(10000); 

	while (true) {
		if ((GetAsyncKeyState(VK_ESCAPE) & 0x8000) != 0)
				break;
		if ((GetAsyncKeyState(VK_NUMPAD3) & 0x8000) != 0)
			FPS = 30;
		if ((GetAsyncKeyState(VK_NUMPAD6) & 0x8000) != 0)
			FPS = 60;
		if ((GetAsyncKeyState(VK_NUMPAD1) & 0x8000) != 0)
			FPS = 1;

		if ((GetAsyncKeyState(VK_F2) & 0x8000) != 0)
			DataRecord = true;

		if ((GetAsyncKeyState(VK_F4) & 0x8000) != 0)
			DataRecord = false;

		if ((GetAsyncKeyState(VK_F6) & 0x8000) != 0)
			Centering = true;

		for (int i = 0; i < controllerList.count; i++)
		{
			if (ctrl[i]->ControllerState.PSMoveState.MoveButton == PSMButtonState_DOWN)
				Centering = true;
			if (ctrl[i]->ControllerState.PSMoveState.CrossButton == PSMButtonState_DOWN)
				DataRecord = true;
			if (ctrl[i]->ControllerState.PSMoveState.CircleButton == PSMButtonState_DOWN)
				DataRecord = false;
		}
	}

	if (PSMConnected) {
		PSMConnected = false;
		if (pPSMUpdatethread) {
			pPSMUpdatethread->join();
			delete pPSMUpdatethread;
			pPSMUpdatethread = nullptr;
		}

		for (int i = 0; i < controllerList.count; i++)
		{
			PSM_StopControllerDataStream(controllerList.controller_id[i], PSM_DEFAULT_TIMEOUT);
			PSM_FreeControllerListener(controllerList.controller_id[i]);
		}

		PSM_Shutdown();
	}
	MotionCapture.close();

	system("cls");

	printf("\r\n r57zone\r\n www.r57zone.github.io\r\n");

	Sleep(1000);

    return 0;
}