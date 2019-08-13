#ifndef MYMULTICASTOPTITRACK_H
#define MYMULTICASTOPTITRACK_H
#ifdef MYMULTICASTOPTITRACK_LIB
# define MYMULTICASTOPTITRACK_EXPORT Q_DECL_EXPORT
#else
# define  Q_DECL_IMPORT
#endif


#include <string>
#include <math.h>
using namespace std;
//**********************
//******** ½á¹¹Ìå ******
//**********************
//**************************
//
// model limits
#define MAX_MODELS                  200     // maximum number of MarkerSets 
#define MAX_RIGIDBODIES             1000    // maximum number of RigidBodies
#define MAX_NAMELENGTH              256     // maximum length for strings
#define MAX_MARKERS                 200     // maximum number of markers per MarkerSet
#define MAX_RBMARKERS               20      // maximum number of markers per RigidBody
#define MAX_SKELETONS               100     // maximum number of skeletons
#define MAX_SKELRIGIDBODIES         200     // maximum number of RididBodies per Skeleton
#define MAX_LABELED_MARKERS         1000    // maximum number of labeled markers per frame
#define MAX_UNLABELED_MARKERS       1000    // maximum number of unlabeled (other) markers per frame

#define MAX_FORCEPLATES             8       // maximum number of force plates
#define MAX_ANALOG_CHANNELS         32      // maximum number of data channels (signals) per analog/force plate device
#define MAX_ANALOG_SUBFRAMES        30      // maximum number of analog/force plate frames per mocap frame

#define MAX_PACKETSIZE				100000	// max size of packet (actual packet size is dynamic)

typedef float MarkerData[3];                // posX, posY, posZ
// MarkerSet Data (single frame of one MarkerSet)
typedef struct
{
	char szName[MAX_NAMELENGTH];            // MarkerSet name
	int nMarkers;                           // # of markers in MarkerSet
	MarkerData* Markers;                    // Array of marker data ( [nMarkers][3] )

} sMarkerSetData;
// Rigid Body Data (single frame of one rigid body)
typedef struct sRigidBodyData
{
	int ID;                                 // RigidBody identifier
	float x, y, z;                          // Position
	float qx, qy, qz, qw;                   // Orientation
	int nMarkers;                           // Number of markers associated with this rigid body
	MarkerData* Markers;                    // Array of marker data ( [nMarkers][3] )
	int* MarkerIDs;                         // Array of marker IDs
	float* MarkerSizes;                     // Array of marker sizes
	float MeanError;                        // Mean measure-to-solve deviation
	short params;                           // Host defined tracking flags
	sRigidBodyData()
	{
		Markers = 0; MarkerIDs = 0; MarkerSizes = 0; params = 0;
	}
} sRigidBodyData;
typedef struct
{
	int skeletonID;                                          // Skeleton identifier
	int nRigidBodies;                                        // # of rigid bodies
	sRigidBodyData* RigidBodyData;                           // Array of RigidBody data
} sSkeletonData;
// Marker
typedef struct
{
	int ID;                                 // Unique identifier
	float x;                                // x position
	float y;                                // y position
	float z;                                // z position
	float size;                             // marker size
	short params;                            // host defined parameters
} sMarker;
typedef struct
{
	int nFrames;                                    // # of analog frames of data in this channel data packet (typically # of subframes per mocap frame)
	float Values[MAX_ANALOG_SUBFRAMES];             // values
} sAnalogChannelData;
typedef struct
{
	int ID;                                         // ForcePlate ID (from data description)
	int nChannels;                                  // # of channels (signals) for this force plate
	sAnalogChannelData ChannelData[MAX_ANALOG_CHANNELS];// Channel (signal) data (e.g. Fx[], Fy[], Fz[])
	short params;                                   // Host defined flags
} sForcePlateData;

typedef struct
{
	float x;                                // x position
	float y;                                // y position
	float z;                                // z position
	float pitch;                             // marker size
	float roll;                            // host defined parameters
	float yaw;
    float qx;
	float qy;
	float qz;
	float qw;
} DATA;

//***********************************************
//***********************************************
// Single frame of data (for all tracked objects)
typedef struct
{
	unsigned short iMessage;                // message ID (e.g. NAT_FRAMEOFDATA)
	unsigned short nDataBytes;              // Num bytes in payload
	int iFrame;                                     // host defined frame number
	int nMarkerSets;                                // # of marker sets in this frame of data
	sMarkerSetData MocapData[MAX_MODELS];           // MarkerSet data
	int nOtherMarkers;                              // # of undefined markers
	MarkerData* OtherMarkers;                       // undefined marker data
	int nRigidBodies;                               // # of rigid bodies
	sRigidBodyData RigidBodies[MAX_RIGIDBODIES];    // Rigid body data
	int nSkeletons;                                 // # of Skeletons
	sSkeletonData Skeletons[MAX_SKELETONS];         // Skeleton data
	int nLabeledMarkers;                            // # of Labeled Markers
	sMarker LabeledMarkers[MAX_LABELED_MARKERS];    // Labeled Marker data (labeled markers not associated with a "MarkerSet")
	int nForcePlates;                               // # of force plates
	sForcePlateData ForcePlates[MAX_FORCEPLATES];   // Force plate data
	float fLatency;                                 // host defined time delta between capture and send
	unsigned int Timecode;                          // SMPTE timecode (if available)
	unsigned int TimecodeSubframe;                  // timecode sub-frame data
	double fTimestamp;                              // FrameGroup timestamp
	short params;                                   // host defined parameters
} sFrameOfMocapData;
//*********************************************
//*********************************************


class  MyMultiCastOptitrack
{
	//Q_OBJECT
public:
	MyMultiCastOptitrack();
	~MyMultiCastOptitrack();
	void SetUpMyDataTableWidget(char *inPut);
	DATA UavData;
private:
	//void setTableWidgetCentreAlignment(QTableWidget* myTable, int m_Row, int m_Column, QString myName);
	
};

#endif // MYMULTICASTOPTITRACK_H
