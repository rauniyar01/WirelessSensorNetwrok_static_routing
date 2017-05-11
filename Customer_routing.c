#include "core/contiki.h"
#include "core/dev/leds.h"			// Use LEDs.
#include "core/sys/clock.h"			// Use CLOCK_SECOND.
#include "dev/cc2420/cc2420.h"				// For cc2420_set_channel().
#include "core/sys/node-id.h"		// Manipulate the node_id.
#include "core/net/rime/rime.h"		// Establish connections.
#include "core/net/rime/broadcast.h"		// Establish connections.
#include "core/net/linkaddr.h"
#include "core/net/packetbuf.h"
#include "lib/sensors.h"                 //required for sensor_sensor struct that is used to define phidget struct
#include "platform/z1/dev/z1-phidgets.h" //definition of the phidget struct
#include "dev/i2cmaster.h"  // Include IC driver
#include <stdio.h>
#define UNICAST_RIME_CHANNEL 146
#define BROADCAST_RIME_CHANNEL 129
#define IN 250
#define N 6
#define NO_FORCE -1
/********************System Varibales**************************/
static uint8_t timer_interval = 1;	// In seconds
linkaddr_t recAddr;
/********************System Varibales**************************/
static float  SensorValue;
static void unicast_recv(struct unicast_conn *c, const linkaddr_t *from);
uint32_t voltage;

/********************Struct for send and receive**************************/
typedef struct{
	uint8_t Flag;
	uint8_t Flag_v;
	uint8_t sendnodeID;
	uint16_t timeID;
	char content[10]; //used for sending hello
	uint8_t x;
	uint8_t y;
	uint8_t value;
	uint8_t Matrix[N][N];
}Messagetype;
typedef struct{
	uint8_t unicast_receiver_mote_id;
	uint8_t unicast_original_mote_id;
	uint8_t unicast_flag;//unicast_flag ==1 means customer wants a helicopter / 0 means not
}UniMessagetype;
/********************Struct for send and receive**************************/

/*************************Temporary Variable******************************/
int routing_table[N-1][N-1];
int path[N]={0};
int g1;
int nb, nj;
int rececounter;
uint8_t RevFlag = 0;// 0 means send Hello!; 1 means send the finished Matrix; 2 means Matrix value
uint8_t Matrix[N][N]; //for saving the local finished Matrix
uint16_t messIDbuf[1000];
uint8_t nodeIDbuf[1000];// position variables for for messageID and nodeID buffers
int i = 0;				// position variables for for messageID and nodeID buffers
int jr, ir;
int jr1,ir1;
/*************************Temporary Variable******************************/
static linkaddr_t generateLinkAddress(uint8_t nodeId);

void dijsktra(int source, int target);
static int getForceSensorValue(uint8_t phidget_input);
static void  forward_msg(Messagetype *message);
static void routingtable(void);
static struct broadcast_conn broadcast;
static struct unicast_conn unicast;
static Messagetype RecMessage;
static UniMessagetype SendMessage_unicast;
static UniMessagetype RecMessage_unicast;

static void broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from) {
		leds_on(LEDS_GREEN);
		printf("Broadcast message received from %d: [RSSI %d]\n",from->u8[0], packetbuf_attr(PACKETBUF_ATTR_RSSI));
		packetbuf_copyto(&RecMessage);
		if (RecMessage.Flag==0 || RecMessage.Flag==1)
		{
			RevFlag = RecMessage.Flag;
		}
		else
		{
			printf("wrong mote\n");
			RevFlag = 10;
		}
		//printf("Received RevFlag %d\n", RevFlag);
		if (RevFlag == 0)// 0 means send Hello!; 1 means send the finished Matrix; flag_v = 1 means Matrix value
		{

			if (RecMessage.Flag_v == 1)
			{
				forward_msg(&RecMessage);
			}
			else
			{
				RecMessage.timeID = clock_seconds();
				RecMessage.sendnodeID = node_id;
				RecMessage.value = -packetbuf_attr(PACKETBUF_ATTR_RSSI);
				RecMessage.x = from->u8[0];
				RecMessage.y = node_id;
				forward_msg(&RecMessage);
			}
			//printf("the value to be sent is : %d between %d is %d\n", RecMessage.x, RecMessage.y, RecMessage.value );
		}
		else if (RevFlag == 1)
		{
			printf("\n");
			printf("/********************************the following is Matrix******************************/");
			printf("\n");
			int ik, jk;//for assignment of the matrix
			for (ik = 0;ik<N;ik++)
			{
				for (jk = 0;jk<N;jk++)
					{
						Matrix[ik][jk] = RecMessage.Matrix[ik][jk];
					}
			}
			printf("Matrix is\n");
			for( nj=0;nj<N;nj++)
		    {
				for( nb=0;nb<N;nb++)
				{
				   printf("%d ", Matrix[nj][nb]);
				}
				printf("\n");
			}
			forward_msg(&RecMessage);
		   /*
			if (RecMessage.sendnodeID<N+1)
			{
			RecMessage.timeID = clock_seconds();
			RecMessage.sendnodeID = node_id;
			RecMessage.Flag = 1;
			forward_msg(&RecMessage);
			}
			*/

			printf("/************************************above is Matrix***********************************/");
			printf("\n");
			printf("\n");
			printf("/***********************the following are path and Routing Table***********************/");
			printf("\n");
			routingtable();
			for(g1=0;g1< (N-1);g1++)
			{
				printf ("the routing table is : %d %d \n",routing_table[g1][1], routing_table[g1][2]);
			}
			printf("/************************the above are path andRouting Table***********************/");
			printf("\n");
		}
		else
		{
			printf("received from the wrong mote");
		}
		leds_off(LEDS_GREEN);
}
static void unicast_recv(struct unicast_conn *c, const linkaddr_t *from) {
	printf("Unicast message received from %d.%d [RSSI %d]\n", from->u8[0], from->u8[1], packetbuf_attr(PACKETBUF_ATTR_RSSI));
	int th,nexmote_id;
	packetbuf_copyto(&RecMessage_unicast);
	printf("\n");
		if (RecMessage_unicast.unicast_receiver_mote_id != node_id)
	    {
			 leds_on(LEDS_BLUE);
			printf("\n");
			printf("/***********************the following is receive unicast********************************/");
			printf("\n\n\n");
			//forward the message
			 for (th=0; th<N; th++)
			 {
				if ((routing_table[th][2])== RecMessage_unicast.unicast_receiver_mote_id)
				{
					nexmote_id=routing_table[th][1];
					recAddr = generateLinkAddress(nexmote_id);
				}
		     }


			 printf("I will forward a request or permission from mote %d.", RecMessage_unicast.unicast_original_mote_id);
			 printf("\n\n\n");
			 packetbuf_copyfrom(&RecMessage_unicast, sizeof(RecMessage_unicast));
			 unicast_send(&unicast, &recAddr);
			 leds_off(LEDS_BLUE);
			 printf("\n");
			 printf("/***********************the above is receive unicast********************************/");
			 printf("\n");
		 }

		else if (RecMessage_unicast.unicast_flag == 2)
		{
		  printf("\n");
		  printf("/***********************the following is receive unicast********************************/");
		  printf("\n");
		  printf("\n");
		  printf("I have received a permission from the center( %d ).", RecMessage_unicast.unicast_original_mote_id);
		  printf("\n");
		  printf("\n");
		  printf("/***********************the above is receive unicast********************************/");
		  printf("\n");
		}
		else
		{
			printf("wrong");
		}
}
static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static const struct unicast_callbacks unicast_call = {unicast_recv};
//--------------------- PROCESS CONTROL BLOCK ---------------------

PROCESS(customer_process, "customer_process");
AUTOSTART_PROCESSES(&customer_process);

//------------------------ PROCESS' THREAD ------------------------
PROCESS_THREAD(customer_process, ev, data){
		PROCESS_EXITHANDLER(broadcast_close(&broadcast));
		PROCESS_EXITHANDLER(unicast_close(&unicast);)
		PROCESS_BEGIN();

		Messagetype test ;
		SENSORS_ACTIVATE(phidgets);
		static int force = NO_FORCE;
		static struct etimer et;
		static uint8_t txPower = 3;
		int th;
		int nexmote_id;
		cc2420_set_txpower(txPower);
		// Configure your team's channel (11 - 26).
		cc2420_set_channel(26);
		// Set the node ID to generate a RIME address.
		//node_id_burn(7);
		// Load the Node ID stored in flash memory.
	    //node_id_restore();
		// Open broadcast connection.
		broadcast_open(&broadcast, BROADCAST_RIME_CHANNEL, &broadcast_call);
		unicast_open(&unicast, UNICAST_RIME_CHANNEL, &unicast_call);
		while(1)
		{
				etimer_set(&et, CLOCK_SECOND * timer_interval);
				PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
				force = getForceSensorValue(PHIDGET3V_2);
				if (force>10)
				{
					printf("\n");
					printf("/***********************the following is unicast********************************/");
					printf("\n");
					printf(" I am going to send a request for a helicopter to the center");
					printf("\n");
					for (th=0; th<N; th++)
					{
						if (routing_table[th][2] == 1)
						{
							nexmote_id=routing_table[th][1];
						}
					}
					SendMessage_unicast.unicast_original_mote_id=node_id;
					SendMessage_unicast.unicast_receiver_mote_id=1;
					SendMessage_unicast.unicast_flag = 1;
					printf("nexmote_id: %d\n", nexmote_id);
					recAddr = generateLinkAddress(nexmote_id);
					leds_on(LEDS_BLUE);
					packetbuf_copyfrom(&SendMessage_unicast, sizeof(SendMessage_unicast));
					unicast_send(&unicast, &recAddr);
					leds_off(LEDS_BLUE);

					printf("\n");
					printf("/***********************the above is unicast********************************/");
					printf("\n");
				 }

			//	printf("your force is %d\n", force);

		}
    PROCESS_END();						// End of the process.
}

static int getForceSensorValue(uint8_t phidget_input){
    voltage =  phidgets.value(PHIDGET3V_2);
    //voltage = 0;
 	SensorValue = voltage/ 4.096 ;
 	return SensorValue;
}

static void  forward_msg(Messagetype *messageptr)
{
    // reset the message ID and node ID buffers when they are full
	if(i > 999)
	{
		i = 0;
		memset(messIDbuf, 0, 1000);
		memset(nodeIDbuf, 0, 1000);
	}
	int j = 0;
	while(messIDbuf[j] != '\0')
	{
	   if(messIDbuf[j] == messageptr->timeID && nodeIDbuf[j] == messageptr->sendnodeID)
	   	{
	   		return;
	   	}
	   ++j;
	}
	leds_on(LEDS_RED);
	// update the message and node ID buffers
	messIDbuf[i] = messageptr->timeID;
	nodeIDbuf[i] = messageptr->sendnodeID;
	++i;
	messIDbuf[i] = '\0';
	nodeIDbuf[i] = '\0';
	//printf("forward  from node ID: %d Flag: %d  \n ", messageptr->sendnodeID, messageptr->Flag);
	packetbuf_copyfrom(messageptr, sizeof(*messageptr));
	broadcast_send(&broadcast);
	leds_off(LEDS_RED);
	return;
}

void dijsktra(int source,int target)
{

    int dist[N],prev[N],selected[N]={0},i1,m,min,start,d,j1;

    for(i1=0;i1< N;i1++)
    {
        dist[i1] = IN;
        prev[i1] = -1;
    }
    start = source;
    selected[start]=1;  //1 means this node has been processed, 0 means not
    dist[start] = 0;	//initial shortest path distance
    while(selected[target] ==0)			//stop until the target is processed
    {
        min = IN;
        m = 0;							//the next node with shortest distance
        for(i1=0;i1< N;i1++)
        {
            d = dist[start] + Matrix[start][i1];
            if(d< dist[i1]&&selected[i1]==0)
            {
                dist[i1] = d;
                prev[i1] = start;
            }
            if(min>dist[i1] && selected[i1]==0)
            {
                min = dist[i1];
                m = i1;
            }
        }
        start = m;
        selected[start] = 1;
    }
    start = target;
    j1 = 0;
    while(start != -1)
    {
        path[++j1] = start;
        start = prev[start];
    }
    path[j1+1] = -1;
   /*
    j1 = 1;

    printf("the  path is : ");
    while ( path[j1] != -1)
    {
    	printf("  %d  ", path[j1]);
    	j1++;
    }
    printf("\n ");
    */
}

void routingtable(void)
 {

 	int t,b1,r;
 	int k1=0;
 	for (t=0;t<N; t++)
 	{
 	      if (t != node_id-1)
 	      {
 		     dijsktra(node_id-1,t);
 		     for (b1=1; b1<N;b1++)
 		     {
 		        if ((path[b1-1] != -1)&&(path[b1]==-1))
 			    {
 				    r=b1;
 				    routing_table[k1][1]= path[r-2]+1;
 			    }
 		     }
 		    //the next node of the shortest path
			routing_table[k1][2]= path[1]+1;   //destination
			k1=k1+1;
 	      }
   }
 }

static linkaddr_t generateLinkAddress(uint8_t nodeId){
	linkaddr_t addr;

	addr.u8[0] = nodeId;
	addr.u8[1] = 0;

	return addr;
}



















