#include <sys/types.h>
#include <sys/socket.h>
#include <string.h>
#include <unistd.h>
#include <pthread.h> //for threading , link with lpthread
#include <linux/can.h>
#include <linux/if.h>
#include <stdio.h>
#include <sys/ioctl.h>

uint8_t teamID = 128;
uint8_t status=1;
int32_t nav_accel=2;
int32_t nav_pos=3;
int32_t nav_vel=4;

int32_t pw_volt=5;
int32_t pw_cur=6;
int32_t pw_temp=7;
int32_t ctrl_temp=8;
uint32_t nav_strip=9;


//TCP dump

int32_t nav_yaw = 10;
int32_t nav_pitch = 11;
int32_t nav_roll= 12;
int32_t nav_lts_brake1=13;
int32_t nav_lts_brake2=14
int32_t nav_lts_brake3=15;
int32_t nav_lts_brake4=16;

int32_t ctrl_pres=17;
int32_t ctrl_lts_height1=18;
int32_t ctrl_lts_height2=19;
int32_t ctrl_lts_height3=20;
int32_t ctrl_lts_height4=21;
int32_t pw_amb_temp=20;
int32_t pw_amb_pressure=21;


#define LVElec	0
#define Idle	1
#define Ready	2
#define AwaitPusherAttach	3
#define Pushing	4
#define LevAndBraking	5
#define DescAndBrakeRetr	6
#define Rolling	7
#define LSD 8
#define PodStop	9
#define EmStop	10
#define Fault	11
#define EB0	12
#define EB1	13
#define EB2	14
#define PowerOff	15

#define activateClutch 1	//Clutch Engaged
#define deactivateClutch 1	//Clutch Disngaged

volatile int system_state=0;

can_frame packet;

char c[2000], d[2000];
for (int i=0;i<2000;i++)
{
    c[i]=0;
    d[i]=0;
}
*((uint8_t*))(c)=teamID;
char *switch_pointer =c;
pthread_attr_t attr;
pthread_t tid;
int s = 0;

pthread_mutex_t lock;
//CAN desc
int can_sock;
int nbytes;

//TCP client sock (from Base stations)
int client_sock[2];
struct sockaddr_in client[2];

void *connection_handler(void *);


//---------------------------TCP packet handlers---------------------------


typedef void (*IntFunctionWithOneParameter) (int s);

//podOp0
void LinActUp(int sk){
	if (system_state < Ready){	//Check if Pod is offline
   	   packet.can_id = 0x84;
	   packet.can_dlc = 1;
	   packet.data[0] = 0x01;	//Enable HBridge(MSBits), Set direction to up(LSBits)		
	   write(can_sock, &packet, sizeof(struct can_frame));
	}
}   

//podOp1
void LinActDown(int sk){
    if (system_state < Ready){	//Check if Pod is offline
   		packet.can_id = 0x84;
   		packet.can_dlc = 1;
   		packet.data[0] = 0x00;	//Enable HBridge(MSBits), Set direction to down(LSBits)		 
   		write(can_sock, &packet, sizeof(struct can_frame));
   	}
}

//podOp2
void LowSpeedDrive(int sk){
	int read_size;
	char client_message[2000];
    
    if (system_state < Ready){	//Check if Pod is offline
    	if( (read_size = recv(sk , client_message , 1 , 0)) > 0 ){	//Read next pack to get RPM value of LSD
    		packet.data[0] = (int)client_message[0];	//Set value of LSD RPM
	        memset(client_message, 0, 2000);
	    }
	    if(read_size == 0)
	    {
	        puts("Client disconnected");
	        fflush(stdout);
	    }
	    else if(read_size == -1)
	    {
	        perror("recv failed");
	    }
	    packet.can_id = 0x83;
   		packet.can_dlc = 1;
   		write(can_sock, &packet, sizeof(struct can_frame));
   	}
}

//podOp3
void Braking(int sk){
    int read_size;
	char client_message[2000];

    if (system_state < Ready){	//Check if Pod is offline
    	if( (read_size = recv(sk , client_message , 1 , 0)) > 0 ){	//Read another packet to get value of Braking actuation in mm
    		packet.data[0] = (int)client_message[0]; 	//Set value of braking actuation in mm
	        memset(client_message, 0, 2000);
	    }
	    if(read_size == 0)
	    {
	        puts("Client disconnected");
	        fflush(stdout);
	    }
	    else if(read_size == -1)
	    {
	        perror("recv failed");
	    }
    	packet.can_id = 0x82;
   		packet.can_dlc = 1;		
   		write(can_sock, &packet, sizeof(struct can_frame));
   	}
}

//podOp4
void ClutchEng(int sk){
    if (system_state < Ready){	//Check if Pod is offline
    	packet.can_id = 0x81;
   		packet.can_dlc = 1;
   		packet.data[0] = activateClutch;		//Clutch Pin = 1 => Clutch Engaged
   		write(can_sock, &packet, sizeof(struct can_frame));
   	}
}

//podOp5
void ClutchDiseng(int sk){
    if (system_state < Ready){	//Check if Pod is offline
    	packet.can_id = 0x81;
   		packet.can_dlc = 1;
   		packet.data[0] = deactivateClutch;		//Clutch Pin = 0 => Clutch Disengaged
   		write(can_sock, &packet, sizeof(struct can_frame));
	}
}

//podOp6
void EmBrake(int sk){
	packet.can_id = 0xE0;
	packet.can_dlc = 1;
	packet.data[0] = deactivateClutch;		//Disengage Clutch
	write(can_sock, &packet, sizeof(struct can_frame));
}

//podOp7
void GoOnline(int sk){
    packet.can_id = 0x24;		//Change system state to Awaiting Pusher Attachment
	packet.can_dlc = 1;
	packet.data[0] = AwaitPusherAttach;		
	write(can_sock, &packet, sizeof(struct can_frame));
}

//podOp8
void PowerOn (int sk){
	packet.can_id = 0x24;		//Change system state to Idle state
	packet.can_dlc = 1;
	packet.data[0] = Idle;		
	write(can_sock, &packet, sizeof(struct can_frame));	
}

//Array of Functions for TCP handlers
IntFunctionWithOneParameter podOp[] = 
{
        LinActUp,		//podOp[0] 
        LinActDown, 	//podOp[1]
        LowSpeedDrive,	//podOp[2]
        Braking,		//podOp[3]
        ClutchEng,		//podOp[4]
        ClutchDiseng,	//podOp[5]
        EmBrake,		//podOp[6]
        GoOnline,		//podOp[7]
        PowerOn			//podOp[8]
};

//----------------------------------------------------------------------------------


//-----------------------------CAN Packet Handlers-------------------------------------------------------------------------------------

//void 
typedef void (*CANfunctions) (struct can_frame);

void SysState (can_frame *bcast )
{
    *(c+8)=(uint8_t) (bcast->data) ;

}

void SysStateAck (can_frame *bcast)
{

    *(c+592)=uint8_t(bcast->data);
}

void nav_51 (can_frame *bcast) // accleration, pitch, yaw, roll
{

    *((int32_t *)(c+2))=nav_accel;
    *((int32_t *)(c+))=nav_yaw;
    *((int32_t *)(c+))=nav_pitch;
    *((int32_t *)(c+))=nav_roll;

}

void nav_53 (can_frame *bcast) //position velocity
{
    *((int32_t *)(c+6))=nav_pos;
    *((int32_t *)(c+10))=nav_vel;
}

void nav_54 (can_frame *bcast)
{
    *((int32_t *)(c+))=nav_lts_brake1;
    *((int32_t *)(c+))=nav_lts_brake2;

}

void nav_55 (can_frame *bcast)
{
    *((int32_t *)(c+))=nav_lts_brake3;
    *((int32_t *)(c+))=nav_lts_brake4;
}

void nav_56 (can_frame *bcast)
{
    *((uint32_t *)(c+30))=nav_strip; 
}

void pow_61 (can_frame *bcast)
{   
    *((int32_t *)(c+14))=pw_volt;
    *((int32_t *)(c+))=pw_amb_temp;
    *((int32_t *)(c+))=pw_amb_pressure;      
}

void pow_62 (can_frame *bcast)
{
    *((int32_t *)(c+18))=pw_cur;   
    *((int32_t *)(c+22))=pw_temp;   
}
void ctrl_31(can_frame *bcast)
{
    *((int32_t *)(c+26))=ctrl_temp;
    *((int32_t *)(c+))=ctrl;  
}

void TempPress (can_frame *bcast)
{
    *(c+400)=int32_t(bcast->data) //Temp from control
    *(c+432)=int32_t(bcast->(data+4)) //Pressure from control

}

void ctrl_32 (can_frame *bcast)
{
    *((int32_t *)(c+))=ctrl_lts_height12;
}

void LTSHeight34 (can_frame *bcast)
{
    *(c+528)=int32_t(bcast->data);
    *(c+560)=int32_t(bcast->(data+4));
}


CANfunctions canOp[] = 
{
        SysState,		//canOp[0] 
        SysStateAck, 	//canOp[1]
        Acc
        Gyro,		//canOp[2]
        PosVelo,		//canOp[3]
        LTSBraking12,	//canOp[4]
        LTSBraking34,	//canOp[5]
        RRS,			//canOp[6]
        Volt4TempPress,	//canOp[7]
        Curr4,			//canOp[8]
        Temp4,			//canOp[9]
        TempPress,		//canOp[10]
        LTSHeight12,	//canOp[11]
        LTSHeight34		//canOp[12]
};

//------------------------------------------------------------------------------------------------------------------

 void sendPacket_pod()
 {
    c= d;
    d= switch_pointer;
    switch_pointer=c;        

    write(sock,d,2000);

            
 } 


int canHandler()
{
   	pthread_t tid_can;

   	struct sockaddr_can addr;
    struct can_frame frame;
    struct ifreq ifr;
	
	const char *ifname = "can1";
    
    if((can_sock = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("Error while opening socket");
        return -1;
    }
    
    strcpy(ifr.ifr_name, ifname);

    ioctl(can_sock, SIOCGIFINDEX, &ifr);
    
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
	printf("%s at index %d\n", ifname, ifr.ifr_ifindex);

    if(bind(can_sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("Error in socket bind");
        return -2;
    }

    while(1)
    {
        nbytes = read(can_sock, &frame, sizeof(struct can_frame));
	    if (nbytes < 0) {
            perror("can raw socket read");
            return 1;
        }
        
        else if (nbytes < sizeof(struct can_frame)) {
			fprintf(stderr, "read: incomplete CAN frame\n");
			return 1;
		}
        
		else{
			pthread_create( &tid, &attr, &canOp[](), );
		}
    }
    return 0;
}

int spacexTelemetry()
{
	sockaddr_in sa;
	uint8_t a[34];
	a[0] = teamID;
	
	int sfd=socket(AF_INET,SOCK_DGRAM,0);
	inet_aton("127.0.0.1",(in_addr*)&(sa.sin_addr.s_addr));
	sa.sin_family=AF_INET;
	sa.sin_port=htons(30000);
	connect(sfd,(const sockaddr *) &sa, sizeof(sa));
	int32_t *accel= (int32_t)d[16];
    int32_t *pos=(int32_t)d[48];
    int32_t *vel=(int32_t)d[80];
    int32_t *volt=(int32_t)d[112];
    int32_t *cur=(int32_t)d[144];
    int32_t *bat_temp=(int32_t)d[176];
    int16_t *pod_temp=(int16_t)d[208];
    uint32_t *strip=(uint32_t)d[224];
    while(1)
	{
		a[1] = status;
		memcpy(a+2,&accel,4);
		memcpy(a+6,&pos,4);
		memcpy(a+10,&vel,4)
;		memcpy(a+14,&volt,4);
		memcpy(a+18,&cur,4);
		memcpy(a+22,&bat_temp,4);
		memcpy(a+26,&pod_temp,4);
		memcpy(a+30,&strip,4);
		send(sfd,a,34,0);
		sleep(1);
	}
}

int fail(const char* a)
{
	//oh noes!!!
	return 0;
}

void *connection_handler(void *socket_desc)
{
    //Get the socket descriptor
	//TCP socket desc
	int sock = *(int*)socket_desc;
	int read_size;
	char *message , client_message[2000];	
      
    //Send some messages to the client
    // message = "Greetings! I am your connection handler\n";
    // write(sock , message , strlen(message));
     
    // message = "Now type something and i shall repeat what you type \n";
    // write(sock , message , strlen(message));
     
    //Receive a message from client
    while( (read_size = recv(sock , client_message , 1 , 0)) > 0 )
    {
        podOp[(int) client_message[0]](sock);
		memset(client_message, 0, 2000);
    }
     
    if(read_size == 0)
    {
        puts("Client disconnected");
        EmBrake(0);
        fflush(stdout);
    }
    else if(read_size == -1)
    {
        perror("recv failed");
    }
         
    return 0;
} 			

int main()
{
	s = pthread_attr_init(&attr);
	s += pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
	
	s += pthread_create( &tid, &attr, (void* (*)(void*))&canHandler, NULL);	//Thread to listen on the CAN network
	s += pthread_create( &tid, &attr, (void* (*)(void*))&spacexTelemetry, NULL);	//Thread for UDP sending of systen data
	s += pthread_create( &tid, &attr, (void* (*)(void*))&remoteControlTask, NULL);	//Thread for TCP communication with Base Station
	if(s!=0)
		fail("pthread");
	while(1){}
}