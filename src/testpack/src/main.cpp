#include "KFApp.h"

//#define Mahony_main		main  // Example-1
//#define SINSGPS_main		main  // Example-2
#define GPSCFG_main		main  // Example-3

////////////////////////////////////////////////////////////////////////////////
int Mahony_main(void)
{
	CMahony mahony(10.0);
	CVect3 eb = CVect3(-4.0,1.3,0.0)*glv.dps;  // 陀螺零偏 deg/s
	CVect3 db = O31;
	
	mcu_init(0);
	while(1)
	{
		if(GAMT_OK_flag==0) continue;
		GAMT_OK_flag = 0;
		CVect3 wm = (*(CVect3*)mpu_Data_value.Gyro*glv.dps-eb)*TS;
		CVect3 vm = (*(CVect3*)mpu_Data_value.Accel*glv.g0-db)*TS;
		mahony.Update(wm, vm, TS);
		AVPUartOut(q2att(mahony.qnb));
	}
}

///////////////////////////////////////////////////////////////////////////
int SINSGPS_main(void)
{
	CKFApp kf(TS);

	double yaw0 = C360CC180(100.0*glv.deg);  // 北偏东为正
	CVect3 gpspos=LLH(34.24858459,108.91009771,403), gpsvn=O31;
	kf.Init(CSINS(a2qua(CVect3(0,0,yaw0)), O31, gpspos));  // 请正确初始化方位和位置
	CVect3 eb = CVect3(-4.0,1.3,0.0)*glv.dps;  // 陀螺零偏 deg/s
	CVect3 db = O31;
		 
	mcu_init(0);
	while(1)
	{
		if(GAMT_OK_flag==0) continue;
		GAMT_OK_flag = 0;
		kf.SetCalcuBurden(TIM2->CNT,0);
		CVect3 wm = (*(CVect3*)mpu_Data_value.Gyro*glv.dps-eb)*TS;
		CVect3 vm = (*(CVect3*)mpu_Data_value.Accel*glv.g0-db)*TS;
		if(GPS_OK_flag)
		{
			GPS_OK_flag = 0;
			if(gps_Data_value.GPS_numSV>6&&gps_Data_value.GPS_pDOP<5.0f)
			{
				gpsvn = *(CVect3*)gps_Data_value.GPS_Vn; gpspos = *(CVect3*)gps_Data_value.GPS_Pos;
//				kf.SetMeasGNSS(gpspos, gpsvn);
			}
		}
//		if(kf.iter==-2) kf.SetMeasGNSS(gpspos, CVect3(0,0,0.01));
		kf.Update(&wm, &vm, 1, TS, 3);
		AVPUartOut(kf);
//		out_data.Vn[2] = kf.SetCalcuBurden(TIM2->CNT,1);  // for debug
//		out_data.Pos[4] = kf.timcnt1 + kf.timcnt0/100.0;
	}
}

///////////////////////////////////////////////////////////////////////////
int GPSCFG_main(void)
{
	mcu_init(1);
	while(1)	{	;	}
}
