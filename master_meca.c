//IP MECA 192.168.2.103
//tempo di ciclo garantito 2ms, minimo 1ms
#include <sys/mman.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <unistd.h>
#include <sched.h>
#include <time.h>
#include <pthread.h>
#include <math.h>

#include "ethercat.h"

#define MECA500 1
//timeout in us 
#define EC_TIMEOUT_INIT_TO_PRE_OP 2000000 //da init a pre_op
#define EC_TIMEOUT_TO_SAFE_OP 9000000 //da pre_p a safe_op e da safe_op a op
#define EC_TIMEOUT_PRE_OP_TO_INIT 5000000 //da pre_op a init
#define EC_TIMEOUT_OP_TO_SAFE_OP 200000 //da op a safe_op
#define EC_TIMEOUT_SDO_CYCLE 100000 //tempo di ritorno del frame ethercat
#define EC_TIMEOUT_SDO_RESPONSE 2000000 //tempo di risposta del singolo slave
#define NSEC_PER_SEC 1000000000 //nanosecondi ogni secondo
#define CYCLE_TIME 1000000 //ciclo in ns

#define stack8k (8 * 1024)

char IOmap[4096];
pthread_t thread1,thread2,thread3;
int dorun = 0;
int64 toff, gl_delta;
int expectedWKC;
long int time1;
long int time2;
long int cycle;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;
int shutdown=0;
int j=0;

//struttura che rappresenta le entry dell'Object Dictionary
typedef struct 
{
        int index;
        int sub_index;
        int size;
        int value;
      
} OBentry;

//struttura che rappresenta gli ingressi dell'MECA500 (Rx)
typedef struct PACKED
{		
	    uint32 robot_control;
       uint32 motion_control;
       uint32 motion_command;
       float var1;
       float var2;
       float var3;
       float var4;
       float var5;
       float var6; 
      
} out_MECA500t;

out_MECA500t  * out_MECA500;

//struttura che rappresenta le uscite dell'MECA500
/*typedef struct PACKED
{
        int32 position_actual_value;
        int32 velocity_actual_value;
        int16 torque_actual_value;
        uint16 statusword;
        int32 current_actual_value;
        
} in_MECA500t;

in_MECA500t * in_MECA500;*/

//SM2 master->slave
//SM3 slave->master
int MECA500_setup(uint16 slave){
int retval;
//disattivo i PDO in inviati allo slave
OBentry TXPDO_number={0x1c13,0x00,sizeof(uint8),0};
OBentry RXPDO_number={0x1c12,0x00,sizeof(uint8),0};
/*
retval=ec_SDOread(slave, TXPDO_number.index,TXPDO_number.sub_index, FALSE, 
&(TXPDO_number.size), &(TXPDO_number.value), EC_TIMEOUTSAFE);
 printf("sub_index=%d,value=%d,esito=%d\n",TXPDO_number.sub_index,TXPDO_number.value,retval);
*/
printf("---DATI DA SCRIVERE---");
retval=ec_SDOwrite(slave, TXPDO_number.index,TXPDO_number.sub_index, 
FALSE, TXPDO_number.size, &(TXPDO_number.value), EC_TIMEOUTSAFE);
printf("sub_index=%d,value=%x,esito=%d\n\n",TXPDO_number.sub_index,TXPDO_number.value,
retval);

retval=ec_SDOwrite(slave, RXPDO_number.index,RXPDO_number.sub_index, 
FALSE, RXPDO_number.size, &(RXPDO_number.value), EC_TIMEOUTSAFE);
printf("sub_index=%d,value=%x,esito=%d\n\n",RXPDO_number.sub_index,RXPDO_number.value,
retval);
//mappo tutti i PDO
OBentry TXPDO={0x1c13,0x01,sizeof(uint16),0x1a00};
OBentry RXPDO={0x1c12,0x01,sizeof(uint16),0x1600};

//posso assegnarli così perchè gli oggetti in cui è descritta la mappatura
//sono consecutivi   
while(TXPDO.value<=0x1a08){
 retval=ec_SDOwrite(slave, TXPDO.index,TXPDO.sub_index, FALSE, TXPDO.size, 
 &(TXPDO.value), EC_TIMEOUTSAFE);
 printf("sub_index=%d,value=%x,esito=%d\n",TXPDO.sub_index,TXPDO.value,retval);
 if(retval>0){
 TXPDO_number.value++;	
 TXPDO.value++;
 TXPDO.sub_index++;	  
 }
}

/*
TXPDO.value=0x1a00;
while(TXPDO.sub_index<=0x0c){
 retval=ec_SDOwrite(slave, TXPDO.index,TXPDO.sub_index, FALSE, TXPDO.size, 
 &(TXPDO.value), EC_TIMEOUTSAFE);
 printf("esito_sub_index%d=%d\n",TXPDO.sub_index,retval);
 TXPDO.sub_index++;
 TXPDO_number.value++;
}*/

//abilito i PDO

retval=ec_SDOwrite(slave, TXPDO_number.index,TXPDO_number.sub_index, 
FALSE, TXPDO_number.size, &(TXPDO_number.value), EC_TIMEOUTSAFE);
printf("esito_numero=%d\n",retval);

//controllo che siano memorizzati i dati corretti
printf("---DATI LETTI---");
for(int i=0x1; i<=TXPDO_number.value; i++){
 retval=ec_SDOread(slave, TXPDO.index,i, FALSE, &(TXPDO.size), 
 &(TXPDO.value), EC_TIMEOUTSAFE);
 printf("sub_index=%d,value=%x,esito=%d\n",i,TXPDO.value,retval); 
}

while(RXPDO.value<=0x1602){
 retval=ec_SDOwrite(slave, RXPDO.index,RXPDO.sub_index, FALSE, RXPDO.size, 
 &(RXPDO.value), EC_TIMEOUTSAFE);
 printf("sub_index=%d,value=%x,esito=%d\n",RXPDO.sub_index,RXPDO.value,retval);
 if(retval>0){
 RXPDO_number.value++;	
 RXPDO.value++;
 RXPDO.sub_index++;	  
 }
}

//abilito i PDO
retval=ec_SDOwrite(slave, RXPDO_number.index,RXPDO_number.sub_index, 
FALSE, RXPDO_number.size, &(RXPDO_number.value), EC_TIMEOUTSAFE);
printf("esito_numero=%d\n",retval);
ec_dcsync0(MECA500,TRUE,CYCLE_TIME,CYCLE_TIME/2);

return 0;
}

//funzione che aggiorna quando ecathread dovrà svegliarsi
void add_timespec(struct timespec *ts, int64 addtime){
   int64 sec, nsec;

   nsec = addtime % NSEC_PER_SEC;
   sec = (addtime - nsec) / NSEC_PER_SEC;
   ts->tv_sec += sec;
   ts->tv_nsec += nsec;
   if ( ts->tv_nsec > NSEC_PER_SEC )
   {
      nsec = ts->tv_nsec % NSEC_PER_SEC;
      ts->tv_sec += (ts->tv_nsec - nsec) / NSEC_PER_SEC;
      ts->tv_nsec = nsec;
   }
}

//sincronizzazione del clock del master e della rete
void ec_sync(int64 reftime, int64 cycletime , int64 *offsettime){
   static int64 integral = 0;
   int64 delta;
  
   delta = (reftime) % cycletime;
   if(delta> (cycletime / 2)) { delta= delta - cycletime; }
   if(delta>0){ integral++; }
   if(delta<0){ integral--; }
   *offsettime = -(delta / 100) - (integral / 20);
   gl_delta = delta;
}

OSAL_THREAD_FUNC ecatthread(){
   struct timespec ts, tleft;
   int ht;
   int64 cycletime;
   struct sched_attr attr;
   attr.size=sizeof(attr);
   sched_fifo(&attr,40,0);
   printf("Start real time thread\n");
   clock_gettime(CLOCK_MONOTONIC, &ts);
   ht = (ts.tv_nsec / 1000000) + 1; /* round to nearest ms */
   ts.tv_nsec = ht * 1000000;
   cycletime = CYCLE_TIME; /* cycletime in ns */
   toff = 0;
   //dorun = 0;
    /* eseguo il pinning della pagine attuali e future occupate dal thread per garantire
        prevedibilità nelle prestazioni real-time */
   if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1){
         printf("mlockall failed: %m\n");
         pthread_cancel(pthread_self());
        }
        
    ec_send_processdata();
   
	for(int j=0;j<5000;j++){
      time1=ec_DCtime;
	   add_timespec(&ts, cycletime + toff);
      clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, &tleft);
	   wkc=ec_receive_processdata(EC_TIMEOUTRET);
	   ec_sync(ec_DCtime, cycletime, &toff);
      ec_send_processdata();
      time2=ec_DCtime;
      cycle=time2-time1;
	   }

      shutdown=1;
}


OSAL_THREAD_FUNC MECA_test(char *ifname){
   int cnt;
   
   printf("Starting MECA test\n");

   //inizializza SOEM e lo collega alla porta ifname
   if (ec_init(ifname))
   {
      printf("ec_init on %s succeeded.\n",ifname);
       //trova e autoconfigura gli salve
      if ( ec_config_init(FALSE) > 0 )
      {
         printf("%d slaves found and configured.\n",ec_slavecount);
         //ec_slave[MECA500].blockLRW=1;
		   ec_slave[0].state = EC_STATE_PRE_OP;
		   ec_writestate(0);
		   ec_statecheck(0, EC_STATE_PRE_OP, EC_TIMEOUT_INIT_TO_PRE_OP);

         //MECA500_setup(MECA500);

		 //quando avviene la transizione PRE_OP->SAFE_OP chiama MECA500setup per 
		 //settare i parametri e mappare i PDO 
		   ec_slave[MECA500].PO2SOconfig = MECA500_setup; 
		 
        //configura il meccanismo DC
         ec_configdc();
         //mappa i PDO mappati precedentemente nel buffer locale
         ec_config_map(&IOmap);
         
       out_MECA500 = (out_MECA500t*) ec_slave[MECA500].outputs; //output del master
       //in_MECA500 = (in_MECA500t*) ec_slave[1].inputs;  //input del master
        
         //legge e conserva lo stato nel vettore ec_slave[]
         ec_readstate();
         for(cnt = 1; cnt <= ec_slavecount ; cnt++)
         {
            printf("Slave:%d Name:%s Output size:%3dbits Input size:%3dbits State:%2d delay:%d.%d\n",
                  cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
                  ec_slave[cnt].state, (int)ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
          }
          
         expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
         printf("Calculated workcounter %d\n", expectedWKC);
         
         ec_slave[0].state = EC_STATE_SAFE_OP;
		   ec_writestate(0);
		   ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUT_TO_SAFE_OP);
         
         ec_readstate();
         for(cnt = 1; cnt <= ec_slavecount ; cnt++)
         {
            printf("Slave:%d Name:%s Output size:%3dbits Input size:%3dbits State:%2d delay:%d.%d\n",
                  cnt, ec_slave[cnt].name, ec_slave[cnt].Obits, ec_slave[cnt].Ibits,
                  ec_slave[cnt].state, (int)ec_slave[cnt].pdelay, ec_slave[cnt].hasdc);
          }
         printf("Request operational state for all slaves\n");
         ec_slave[0].state = EC_STATE_OPERATIONAL;
         ec_writestate(0);
         //aspetta che tutti raggiungano OPERATIONAL
         osal_thread_create(&thread1, stack8k * 2, &ecatthread, NULL);
         ec_statecheck(0, EC_STATE_OPERATIONAL,  EC_TIMEOUT_TO_SAFE_OP);
         //crea il thread real-time per lo scambio dati
         //dorun = 1;
         
         if (ec_slave[0].state == EC_STATE_OPERATIONAL)
         {
            printf("Operational state reached for all slaves.Actual state=%d\n",ec_slave[0].state);
            inOP = TRUE;
           //ciclo per stampare i dati in tempo reale
            while(!shutdown)
            {   
               printf("PDO n.i=%d,robot_control=%u,motion_control=%u,calc_cycle=%ld[ns],cycle_read=%ld[ns] \n",
                  j,out_MECA500->robot_control,out_MECA500->motion_control,
                  cycle,ec_slave[1].DCcycle);
               /*   
               printf("statusword %4x,controlword %x, position_actual_value %d",
               in_MECA500->statusword,out_MECA500->controlword,in_MECA500->position_actual_value);
               */
               fflush(stdout);
               osal_usleep(1000);
            }
            //dorun = 0;
            inOP = FALSE;
         }
         else
         {
            printf("Not all slaves reached operational state.\n");
             ec_readstate();
             for(int j = 1; j<=ec_slavecount ; j++)
             {
                 if(ec_slave[j].state != EC_STATE_OPERATIONAL)
                 {
                     printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                         j, ec_slave[j].state, ec_slave[j].ALstatuscode, ec_ALstatuscode2string(ec_slave[j].ALstatuscode));
                 }
             }
         }
      }
      else
      {
         printf("No slaves found!\n");
      }
      printf("End MECA test, close socket\n");
      
      /*ec_slave[0].state = EC_STATE_INIT;
	   ec_writestate(0);
	   ec_statecheck(0, EC_STATE_INIT, EC_TIMEOUT_TO_SAFE_OP);*/
      ec_close();
   }
   else
   {
      printf("No socket connection on %s\nExcecute as root\n",ifname);
   }
}

OSAL_THREAD_FUNC ecatcheck(){
    int slave;

    while(1)
    {
        if( inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
               needlf = FALSE;
               printf("\n");
            }
            // one ore more slaves are not responding 
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
               if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
               {
                  ec_group[currentgroup].docheckstate = TRUE;
                  if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                  {
                     printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                     ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state == EC_STATE_SAFE_OP)
                  {
                     printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                     ec_slave[slave].state = EC_STATE_OPERATIONAL;
                     ec_writestate(slave);
                  }
                  else if(ec_slave[slave].state > EC_STATE_NONE)
                  {
                     if (ec_reconfig_slave(slave, EC_TIMEOUTRET3 ))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d reconfigured\n",slave);
                     }
                  }
                  else if(!ec_slave[slave].islost)
                  {
                     // re-check state 
                     ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                     if (ec_slave[slave].state == EC_STATE_NONE)
                     {
                        ec_slave[slave].islost = TRUE;
                        printf("ERROR : slave %d lost\n",slave);
                     }
                  }
               }
               if (ec_slave[slave].islost)
               {
                  if(ec_slave[slave].state == EC_STATE_NONE)
                  {
                     if (ec_recover_slave(slave, EC_TIMEOUTRET3 ))
                     {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d recovered\n",slave);
                     }
                  }
                  else
                  {
                     ec_slave[slave].islost = FALSE;
                     printf("MESSAGE : slave %d found\n",slave);
                  }
               }
            }
            if(!ec_group[currentgroup].docheckstate)
               printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        osal_usleep(10000);
    }
}

int main(int argc, char *argv[]){
  
   printf("SOEM (Simple Open EtherCAT Master)\nMECA test\n");
   
   if (argc > 0)
   {
     //dorun = 0;
	  //create RT thread 
     //osal_thread_create(&thread1, stack8k * 2, &ecatthread, NULL);

      // create thread to handle slave error handling in OP 
      osal_thread_create(&thread2, stack8k * 4, &ecatcheck, NULL);

      // start acyclic part 
      osal_thread_create(&thread3, stack8k * 4, &MECA_test, argv[1]);
      pthread_join(thread3,NULL);
      //MECA_test(argv[1]);
   }
    
    else
    {
      printf("Usage: MECA_test ifname \neno1 or enp2s0\n");
   }
   printf("End program\n");
   return (0);
}