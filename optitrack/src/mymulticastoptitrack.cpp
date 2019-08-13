#include "optitrack/mymulticastoptitrack.h"
#include <cstring>

MyMultiCastOptitrack::MyMultiCastOptitrack()
{

}

MyMultiCastOptitrack::~MyMultiCastOptitrack()
{

}
#define ID_RENDERTIMER 101
#define MATH_PI 3.14159265F
typedef struct { float x, y, z, w; } Quat; /* Quaternion */
void ConvertRHSPosZupToYUp(float& x, float& y, float& z);
void ConvertRHSRotZUpToYUp(float& qx, float& qy, float& qz, float& qw);
#define EulFrmR	     1
#define EulRepNo     0
#define EulRepYes    1
#define EulParOdd    1
#define EulSafe	     "\000\001\002\000"
#define EulNext	     "\001\002\000\001"
enum QuatPart { X, Y, Z, W };
/* EulGetOrd unpacks all useful information about order simultaneously. */
#define EulGetOrd(ord,i,j,k,h,n,s,f) {unsigned o=ord;f=o&1;o>>=1;s=o&1;o>>=1;\
    n=o&1;o>>=1;i=EulSafe[o&3];j=EulNext[i+n];k=EulNext[i+1-n];h=s?k:i;}
/* EulOrd creates an order value between 0 and 23 from 4-tuple choices. */
#define EulOrd(i,p,r,f)	   (((((((i)<<1)+(p))<<1)+(r))<<1)+(f))
#define EulOrdXYZr    EulOrd(Z,EulParOdd,EulRepNo,EulFrmR)
typedef Quat EulerAngles;
typedef float HMatrix[4][4]; /* Right-handed, for column vectors */
EulerAngles ea;
EulerAngles Eul_FromHMatrix(HMatrix M, int order);
EulerAngles Eul_FromQuat(Quat q, int order);
float RadiansToDegrees(float fRadians);

//将收到的消息解析到表格中

void MyMultiCastOptitrack::SetUpMyDataTableWidget( char *inPut)
{
	////
	////myTable->setColumnCount(0);
	////======================================
	//// 消息解析
	char* ptr = inPut;
	int order;		
	sFrameOfMocapData* recvMsg = new sFrameOfMocapData;
	memcpy(recvMsg, ptr, 12); ptr += 12;
	int m_MarkerCount = recvMsg->nMarkerSets;
	//myTable->setColumnCount(myDataCount);
	string *m_name = new string[m_MarkerCount];
	//Marker名称解析
	for (int ii = 0; ii < recvMsg->nMarkerSets; ii++)
	{
		strcpy(recvMsg->MocapData[ii].szName, ptr);
		int nDataBytes = (int)strlen(recvMsg->MocapData[ii].szName) + 1;
		ptr += nDataBytes;
		m_name[ii] = recvMsg->MocapData[ii].szName;		
		memcpy(&recvMsg->MocapData[ii].nMarkers, ptr, 4); ptr += 4;
		recvMsg->MocapData[ii].Markers = new MarkerData[recvMsg->MocapData[ii].nMarkers];
		for (int jj = 0; jj < recvMsg->MocapData[ii].nMarkers; jj++)
		{
			memcpy(recvMsg->MocapData[ii].Markers[jj], ptr, 12); ptr += 12;
		}
	}
	memcpy(&recvMsg->nOtherMarkers, ptr, 4); ptr += 4;
	if (recvMsg->nOtherMarkers > 0)
	{
		recvMsg->OtherMarkers = new MarkerData[recvMsg->nOtherMarkers];
		for (int ii = 0; ii < recvMsg->nOtherMarkers; ii++)
		{
			memcpy(recvMsg->OtherMarkers[ii], ptr, 12); ptr += 12;
		}
	}

	//刚体信息
	memcpy(&recvMsg->nRigidBodies, ptr, 4); ptr += 4;
	int m_RigidCount = recvMsg->nRigidBodies;		//收到的刚体个数
	for (int ii = 0; ii < recvMsg->nRigidBodies; ii++)
	{
		memcpy(&recvMsg->RigidBodies[ii].ID, ptr, 4); ptr += 4;
		memcpy(&recvMsg->RigidBodies[ii].x, ptr, 4); ptr += 4;
		memcpy(&recvMsg->RigidBodies[ii].y, ptr, 4); ptr += 4;
		memcpy(&recvMsg->RigidBodies[ii].z, ptr, 4); ptr += 4;

		//转换为毫米
		recvMsg->RigidBodies[ii].x = recvMsg->RigidBodies[ii].x;// *1000;
		recvMsg->RigidBodies[ii].y = recvMsg->RigidBodies[ii].y;// *1000;
		recvMsg->RigidBodies[ii].z = recvMsg->RigidBodies[ii].z;// *1000;

		memcpy(&recvMsg->RigidBodies[ii].qx, ptr, 4); ptr += 4;
		memcpy(&recvMsg->RigidBodies[ii].qy, ptr, 4); ptr += 4;
		memcpy(&recvMsg->RigidBodies[ii].qz, ptr, 4); ptr += 4;
		memcpy(&recvMsg->RigidBodies[ii].qw, ptr, 4); ptr += 4;
		memcpy(&recvMsg->RigidBodies[ii].nMarkers, ptr, 4); ptr += 4;

		recvMsg->RigidBodies[ii].Markers = new MarkerData[recvMsg->RigidBodies[ii].nMarkers];
		for (int jj = 0; jj < recvMsg->RigidBodies[ii].nMarkers; jj++)
		{
			memcpy(recvMsg->RigidBodies[ii].Markers[jj], ptr, 12); ptr += 12;
		}
		recvMsg->RigidBodies[ii].MarkerIDs = new int[recvMsg->RigidBodies[ii].nMarkers];
		for (int jj = 0; jj < recvMsg->RigidBodies[ii].nMarkers; jj++)
		{
			memcpy(&recvMsg->RigidBodies[ii].MarkerIDs[jj], ptr, 4); ptr += 4;
		}
		recvMsg->RigidBodies[ii].MarkerSizes = new float[recvMsg->RigidBodies[ii].nMarkers];
		for (int jj = 0; jj < recvMsg->RigidBodies[ii].nMarkers; jj++)
		{
			memcpy(&recvMsg->RigidBodies[ii].MarkerSizes[jj], ptr, 4); ptr += 4;
		}
		memcpy(&recvMsg->RigidBodies[ii].MeanError, ptr, 4); ptr += 4;
		memcpy(&recvMsg->RigidBodies[ii].params, ptr, 4); ptr += 2;
	}
	//消息装配
		if (m_name[0] == "Rigid Body 1")
	{
	//		//位置信息
			float pos_X = recvMsg->RigidBodies[0].x;
			float pos_Y = -recvMsg->RigidBodies[0].y;
			float pos_Z = recvMsg->RigidBodies[0].z;

			//四元数求角度
			// RigidBody orientation
			// convert position
			ConvertRHSPosZupToYUp(pos_X, pos_Y, pos_Z);
			// convert orientation
			ConvertRHSRotZUpToYUp(recvMsg->RigidBodies[0].qx, recvMsg->RigidBodies[0].qy, recvMsg->RigidBodies[0].qz, recvMsg->RigidBodies[0].qw);
			// Convert Motive quaternion output to euler angles
			// Motive coordinate conventions : X(Pitch), Y(Yaw), Z(Roll), Relative, RHS
			Quat q;
			q.x = recvMsg->RigidBodies[0].qx;
			q.y = recvMsg->RigidBodies[0].qy;
			q.z = recvMsg->RigidBodies[0].qz;
			q.w = recvMsg->RigidBodies[0].qw;
			order = EulOrdXYZr;
			ea = Eul_FromQuat(q, order);
			UavData.x = pos_X;
			UavData.y = pos_Y;
            UavData.z = pos_Z;
            UavData.pitch=ea.x;
            UavData.roll=ea.y;
            UavData.yaw=ea.z;
           UavData.qx = q.x;
           UavData.qy = q.y;
           UavData.qz = q.z;
           UavData.qw = q.w;

	//	}
	}

}

//====================================
// 四元数转角度
float RadiansToDegrees(float fRadians)
{
	return fRadians * (180.0F / 3.14159265F);
}

//
void ConvertRHSPosZupToYUp(float& x, float& y, float& z)
{
	/*
	[RHS, Y-Up]     [RHS, Z-Up]

	Y
	Y                 Z /
	|__ X             |/__ X
	/
	Z

	Xyup  =  Xzup
	Yyup  =  Zzup
	Zyup  =  -Yzup
	*/
	float yOriginal = y;
	y = z;
	z = -yOriginal;
}

void ConvertRHSRotZUpToYUp(float& qx, float& qy, float& qz, float& qw)
{
	// -90 deg rotation about +X
	float qRx, qRy, qRz, qRw;
	float angle = -90.0f * MATH_PI / 180.0f;
	qRx = sin(angle / 2.0f);
	qRy = 0.0f;
	qRz = 0.0f;
	qRw = cos(angle / 2.0f);

	// rotate quat using quat multiply
	float qxNew, qyNew, qzNew, qwNew;
	qxNew = qw*qRx + qx*qRw + qy*qRz - qz*qRy;
	qyNew = qw*qRy - qx*qRz + qy*qRw + qz*qRx;
	qzNew = qw*qRz + qx*qRy - qy*qRx + qz*qRw;
	qwNew = qw*qRw - qx*qRx - qy*qRy - qz*qRz;

	qx = qxNew;
	qy = qyNew;
	qz = qzNew;
	qw = qwNew;
}


/* Convert quaternion to Euler angles (in radians). */
EulerAngles Eul_FromQuat(Quat q, int order)
{
	HMatrix M;
	double Nq = q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w;
	double s = (Nq > 0.0) ? (2.0 / Nq) : 0.0;
	double xs = q.x*s, ys = q.y*s, zs = q.z*s;
	double wx = q.w*xs, wy = q.w*ys, wz = q.w*zs;
	double xx = q.x*xs, xy = q.x*ys, xz = q.x*zs;
	double yy = q.y*ys, yz = q.y*zs, zz = q.z*zs;
	M[X][X] = 1.0 - (yy + zz); M[X][Y] = xy - wz; M[X][Z] = xz + wy;
	M[Y][X] = xy + wz; M[Y][Y] = 1.0 - (xx + zz); M[Y][Z] = yz - wx;
	M[Z][X] = xz - wy; M[Z][Y] = yz + wx; M[Z][Z] = 1.0 - (xx + yy);
	M[W][X] = M[W][Y] = M[W][Z] = M[X][W] = M[Y][W] = M[Z][W] = 0.0; M[W][W] = 1.0;
	return (Eul_FromHMatrix(M, order));
}

/* Convert matrix to Euler angles (in radians). */
EulerAngles Eul_FromHMatrix(HMatrix M, int order)
{
	EulerAngles ea;
	int i, j, k, h, n, s, f;
	EulGetOrd(order, i, j, k, h, n, s, f);
	if (s == EulRepYes) {
		double sy = sqrt(M[i][j] * M[i][j] + M[i][k] * M[i][k]);
		if (sy > 16 * __FLT_EPSILON__) {
			ea.x = atan2((double)M[i][j], (double)M[i][k]);
			ea.y = atan2(sy, (double)M[i][i]);
			ea.z = atan2(M[j][i], -M[k][i]);
		}
		else {
			ea.x = atan2(-M[j][k], M[j][j]);
			ea.y = atan2(sy, (double)M[i][i]);
			ea.z = 0;
		}
	}
	else {
		double cy = sqrt(M[i][i] * M[i][i] + M[j][i] * M[j][i]);
		if (cy > 16 * __FLT_EPSILON__) {
			ea.x = atan2(M[k][j], M[k][k]);
			ea.y = atan2((double)-M[k][i], cy);
			ea.z = atan2(M[j][i], M[i][i]);
		}
		else {
			ea.x = atan2(-M[j][k], M[j][j]);
			ea.y = atan2((double)-M[k][i], cy);
			ea.z = 0;
		}
	}
	if (n == EulParOdd) { ea.x = -ea.x; ea.y = -ea.y; ea.z = -ea.z; }
	if (f == EulFrmR) { float t = ea.x; ea.x = ea.z; ea.z = t; }
	ea.w = order;
	return (ea);
}
