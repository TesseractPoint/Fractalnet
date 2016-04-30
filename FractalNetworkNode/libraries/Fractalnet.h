#ifndef __FRACTALNET_H__
#define __FRACTALNET_H__


typedef enum {TRANSPORT_NODE=0, ACTIVE_NODE} FractalNodeType;
typedef enum {HUMANTELEMETRY=0, MOTIONTELEMETRY, AUDIO, DATA, MESSAGE} FractalPayloadType;
typedef enum {UNCOMMISSIONED=0, LISTENING, TRANSPORTING, RESPONDING} FractalNodeState;

struct FractalDate
{
	unsigned long unixdate;
	unsigned long millis;
};

struct FractalCoord3
{
	double x;
	double y;
	double z;
};

struct HumanTelemetryPayload
{
	int heartrate; //count per second
	int resprate; //count per second
	int temperature; //tenths of degrees C
	int suitpressure; //millibar
	int co2concentration;
};

struct MotionTelemetryPayload
{
	unsigned long channel;
	double roll;
	double pitch;
	double yaw;
	double acceleration[3];
	long   aux[12];
};

struct AudioPayload
{
	unsigned long  channel;
	unsigned char  sample[1024];
};

struct DataPayload
{
	unsigned long channel;
	unsigned int length;
	unsigned char data[1024];
};

struct MessagePayload
{
	unsigned long info1;
	unsigned long info2;
	char message[1024];
};

struct FractalPacket
{
	unsigned long source;
	unsigned long dest;
	unsigned long via;
	unsigned int  type;
        unsigned int  length;
	FractalDate   timestamp;
	unsigned int  checksum;
        union{ 
		struct HumanTelemetryPayload hutel;
		struct MotionTelemetryPayload motel;
		struct AudioPayload autel;
		struct DataPayload datel;
		struct MessagePayload metel;
	};
};



#endif // __FRACTALNET_H__

