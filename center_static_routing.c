#include "core/contiki.h"
#include "core/dev/leds.h"			// Use LEDs.
#include "core/sys/clock.h"			// Use CLOCK_SECOND.
#include "dev/cc2420/cc2420.h"				// For cc2420_set_channel().
#include "core/sys/node-id.h"		// Manipulate the node_id.
#include "core/net/rime/rime.h"		// Establish connections.
#include "core/net/rime/broadcast.h"		// Establish connections.
#include "core/net/linkaddr.h"
#include "core/net/packetbuf.h"
#include <stdio.h>

#include "cpu/cc2538/usb/usb-serial.h"	// For UART-like I/O over USB.
#include "dev/serial-line.h"			// For UART-like I/O over USB.

#define UNICAST_RIME_CHANNEL 146
#define BROADCAST_RIME_CHANNEL 129
#define IN 250
#define N 6


/********************Struct for send and receive**************************/
typedef struct{
	uint8_t Flag;
	uint8_t Flag_v;
	uint8_t sendnodeID;
	uint16_t timeID;
	char content[10]; //used for sending hello
	uint8_t x;		  //x for sending mote y for receiving mote
	uint8_t y;
	uint8_t value;
	uint8_t Matrix[N][N];
}Messagetype;
typedef struct{
	uint8_t unicast_receiver_mote_id;
	uint8_t unicast_original_mote_id;
	uint8_t unicast_flag;//unicast_flag ==1 means customer wants a helicopter / 0 means not
}UniMessagetype;

typedef struct{
	uint8_t S_ID;
	uint8_t D_ID;
	uint16_t messageID;
	char content[30];
	uint8_t path[4];
	uint8_t hop;
	uint8_t centor_sequence_No;
}Messagetype_to_h;

/********************Struct for send and receive**************************/

/********************System Varibales**************************/
static uint8_t timer_interval = 1;	// In seconds
static UniMessagetype SendMessage_unicast;
static UniMessagetype RecMessage_unicast;
static struct unicast_conn unicast;
static struct broadcast_conn broadcast;
linkaddr_t recAddr;
/********************System Varibales**************************/

/*************************Temporary Variable******************************/
uint8_t order_a_helicopter=0; // for sending back ACK to the customer
uint8_t unicast_original_mote_id;
static uint8_t RevFlag = 0; // 0 means send Hello!; 1 means send the finished Matrix;flag_v = 1 means Matrix value
uint8_t Matrix[N][N];
//uint8_t In_Matrix[N][N];   //in every element: 1 means this value has been changed in this turn, 0 means not and should be set as 250
uint8_t routing_table[N-1][N-1];
int path[N]={0};  //the shortest path found by dijsktra
uint8_t t;
uint8_t nb, nj;
uint8_t g1;
uint8_t a;

/*************************Temporary Variable******************************/

/*************************Declaration of functions*************************/
static linkaddr_t generateLinkAddress(uint8_t nodeId);
int dijsktra(int source, int target);
static void routingtable(void);
/*************************Declaration of functions*************************/

static void broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from) {
	leds_on(LEDS_GREEN);
	static Messagetype RecMessage;
	packetbuf_copyto(&RecMessage);
	Matrix[(RecMessage.x)-1][(RecMessage.y)-1] = RecMessage.value-40;
	Matrix[(RecMessage.y)-1][(RecMessage.x)-1] = RecMessage.value-40;
	//In_Matrix[(RecMessage.x)-1][(RecMessage.y)-1] = 1;
	//In_Matrix[(RecMessage.y)-1][(RecMessage.x)-1] = 1;
	//printf("Broadcast message received from %d.%d: [RSSI %d]\n", from->u8[0], from->u8[1], packetbuf_attr(PACKETBUF_ATTR_RSSI));
	//printf("x: %d  y: %d  value: %d\n", RecMessage.x, RecMessage.y, RecMessage.value);
	leds_off(LEDS_GREEN);
}


static void unicast_recv(struct unicast_conn *c, const linkaddr_t *from) {

	packetbuf_copyto(&RecMessage_unicast);
	if (RecMessage_unicast.unicast_flag == 1)
	{
		leds_on(LEDS_BLUE);
		//printf("/**************the following is receive unicast***************/");
		printf("Unicast message received from %d.%d\n", from->u8[0], from->u8[1]);
		printf("\n\n");
		printf("The customer %d  want to order a helicopter.", RecMessage_unicast.unicast_original_mote_id);
		printf("\n\n");

		order_a_helicopter=1;
		unicast_original_mote_id = RecMessage_unicast.unicast_original_mote_id;
		leds_off(LEDS_BLUE);

		//printf("/***************the above is receive unicast******************/");

	}
}


static const struct broadcast_callbacks broadcast_call = {broadcast_recv};
static const struct unicast_callbacks unicast_call = {unicast_recv};
//--------------------- PROCESS CONTROL BLOCK ---------------------

PROCESS(center_process, "center_process");
PROCESS(update, "update matrix");
PROCESS(control, "control Revflag");
AUTOSTART_PROCESSES(&center_process, &update, &control);

//------------------------ PROCESS' THREAD ------------------------
PROCESS_THREAD(center_process, ev, data){
		PROCESS_EXITHANDLER(broadcast_close(&broadcast));
		PROCESS_BEGIN();

		int i, j;
		for (i = 0;i<N;i++)
		{
			for (j = 0;j<N;j++)
			{
				Matrix[i][j] = IN;
			}
		}
		// struct event timer
		static struct etimer et;
		int th;
		int nexmote_id;
		static uint8_t txPower = 3;
		cc2420_set_txpower(txPower);
		// Configure your team's channel (11 - 26).
		cc2420_set_channel(26);
		// Set the node ID to generate a RIME address.
		//node_id_burn(1);
		// Load the Node ID stored in flash memory.
		//node_id_restore();
		// Open broadcast connection.
		broadcast_open(&broadcast, BROADCAST_RIME_CHANNEL, &broadcast_call);
		unicast_open(&unicast, UNICAST_RIME_CHANNEL, &unicast_call);


		// Main process loop:
		while(1) {
				etimer_set(&et, CLOCK_SECOND * timer_interval);
				PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));



				leds_on(LEDS_RED);
				static Messagetype SendMessage; // for sending first hello, second Matrix

				if (RevFlag == 0)// 0 means send Hello!; 1 means send the finished Matrix; 2 means send back permission to the customer value
				{

					SendMessage.Flag = 0;
					SendMessage.Flag_v = 0;
					sprintf(SendMessage.content,"hello");
					SendMessage.timeID = clock_seconds();
					SendMessage.sendnodeID = node_id;
					packetbuf_copyfrom(&SendMessage, sizeof(SendMessage));
					broadcast_send(&broadcast);
				}
				else if (RevFlag == 1)
				{
					int p;
					for(p=0;p<3;++p)
					{
						SendMessage.Flag = 1;
						SendMessage.timeID = clock_seconds();
						SendMessage.sendnodeID = node_id;
						for (i = 0;i<N;i++)
						{
							for (j = 0;j<N;j++)
							{
							SendMessage.Matrix[i][j] = Matrix[i][j];
							}
						}
						packetbuf_copyfrom(&SendMessage, sizeof(SendMessage));
						broadcast_send(&broadcast);
					}


					printf("\n\n");
					printf("Matrix is\n");
					for( nj=0;nj<N;nj++)
					{
						for( nb=0;nb<N;nb++)
						{
							printf("%*d", 6, Matrix[nj][nb]);
						}
						printf("\n");
					}
					printf("\n\n");
					printf("/********the following is Routing Table**************/");
					printf("\n");
					routingtable();
					printf ("destination     the next mode\n");
					for(g1=0;g1< (N-1);g1++)
					{
						printf("      %d                  %d \n", routing_table[g1][2], routing_table[g1][1]);
					}
					printf("\n");
					printf("/********the above is Routing Table*****************/");
					printf("\n\n");
					for (i = 0;i<N;i++)
					{
						for (j = 0;j<N;j++)
						{
							Matrix[i][j] = IN;
						}
					}
					RevFlag = 2;
				}


				if (order_a_helicopter == 1)
				{
					SendMessage_unicast.unicast_original_mote_id = node_id;
					SendMessage_unicast.unicast_receiver_mote_id = unicast_original_mote_id;
					SendMessage_unicast.unicast_flag = 2;
					for (th=0; th<N; th++)
					{
						if (routing_table[th][2] == unicast_original_mote_id)
						{
							nexmote_id=routing_table[th][1];
							recAddr = generateLinkAddress(nexmote_id);
						}
					}

					leds_on(LEDS_BLUE);
					packetbuf_copyfrom(&SendMessage_unicast, sizeof(SendMessage_unicast));
					unicast_send(&unicast, &recAddr);
					leds_off(LEDS_BLUE);


					printf("\n");
					printf("Send permission back to the customer via unicast.");
					printf("\n\n");

					// sending customer info to helicopter

					order_a_helicopter = 0;
				}



				etimer_reset(&et);
				leds_off(LEDS_RED);
			  }
    PROCESS_END();						// End of the process.
	}

PROCESS_THREAD(update, ev, data){
	//PROCESS_EXITHANDLER(broadcast_close(&broadcast));
	PROCESS_BEGIN();
	static struct etimer et1;

	etimer_set(&et1, CLOCK_SECOND * timer_interval*15);
	PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et1));
	RevFlag = 1;// 1 means the Matrix is finished
	etimer_reset(&et1);
	PROCESS_END();


}

PROCESS_THREAD(control, ev, data)
{
	PROCESS_BEGIN();
	unicast_open(&unicast, UNICAST_RIME_CHANNEL, &unicast_call);
	// Makes use of the default serial input handler.
	uart0_set_input(serial_line_input_byte);
	uint8_t *uartRxBuffer;

	while(1)
	{
		PROCESS_WAIT_EVENT_UNTIL(ev == serial_line_event_message && data != NULL);
		uartRxBuffer = (uint8_t*)data;
		//printf("uartRxBuffer[1]; %d\n", uartRxBuffer[1]);
		if(uartRxBuffer[0] == 1)
		{
			if(uartRxBuffer[1] == 0)
			{
				RevFlag = 0;
				cc2420_set_channel(26);
				process_start(&update, NULL);
			}
			else
			{
				leds_on(LEDS_BLUE);
				static Messagetype_to_h mth;
				cc2420_set_channel(25);
				recAddr = generateLinkAddress(2);
				mth.S_ID = 1;
				mth.D_ID = unicast_original_mote_id;
				//printf("mth.S_ID: %d, mth.D_ID: %d\n", mth.S_ID, mth.D_ID);
				packetbuf_copyfrom(&mth, sizeof(mth));
				unicast_send(&unicast, &recAddr);
				printf("\ncall helicopter\n");
				//printf("node id: %d\n", node_id);
				leds_off(LEDS_BLUE);
			}
		}
	}

	PROCESS_END();
}

int dijsktra(int source,int target)
{


	    int dist[N],prev[N],selected[N]={0, 0, 0, 0, 0, 0},i1,m,min,start,d,j1;
	    //printf("disjistra is executed %d\n", selected[target]);
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
	            if(d<= dist[i1]&&selected[i1]==0)
	            {
	                dist[i1] = d;
	                prev[i1] = start;
	            }
	            if(min>=dist[i1] && selected[i1]==0)
	            {
	                min = dist[i1];
	                m = i1;
	            }
	        }
	        //printf("selected[%d]: %d\n", target, selected[target]);
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
	    return dist[target];
}


static void routingtable(void)
 {

 	int t,b1,r,sum[N-1];
 	int k1=0;
 	for (t=0;t<N; t++)
 	{
 	      if (t != node_id-1)
 	      {
 		     sum[k1] = dijsktra(node_id-1,t);
 		     for (b1=1; b1<N;b1++)
 		     {
 		        if ((path[b1-1] != -1)&&(path[b1]==-1))
 			    {
 				    r=b1;
 				    if(sum[k1] != 250)
 				    {
 				    	routing_table[k1][1]= path[r-2]+1;
 				    }
 				    else
 				    {
 				    	routing_table[k1][1]= 250;
 				    }
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



