/*
 * File:
 *    DAQevent_Noinhibit_MK.c
 *
 * Description:
 *    Simply evolving program to test the mpd library
 *
 */


#include "mpdConfig.h"

#include "jvme.h"
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
/* Include MPD definitions */
#include "mpdLib.h"
#include <stdbool.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
//#include <sys/time.h>

DMA_MEM_ID vmeIN, vmeOUT;
/*! Buffer node pointer */
extern DMANODE *the_event;
/*! Data pointer */
extern unsigned int *dma_dabufp;


// Base address of v262 module:
//unsigned int  v262_BASE_ADDR =0xcc3300;
//unsigned int  V262_FIRM_REVISION_R = 0xFA; //firmware register on v262;
//#define V262_NIM_PULSE_W 0x08
//#define V262_ECL_LEVEL_W 0x04
//#define V262_NIM_LEVEL_W 0x06
//#define V262_NIM_LEVEL_R 0x0A



// routine to catch "Ctrl-C":
//extern bool ctrl_c = false;
bool ctrl_c = false;

void ctrl_c_HANDLER(int signo)
{
  if (signo == SIGINT)
    printf("\n CTRL-C pressed \n\r");
  ctrl_c = true; // tell main loop that it has to exit
  return;
}



// time interval measurement:
struct timeval t1, t2, t3, t4, t5, t6, t7, t8, t9, t10;


void nsleep(long us)
{
  struct timespec wait;
  //printf("Will sleep for is %ld\n", diff); //This will take extra ~70 microseconds

  wait.tv_sec = us / (1000 * 1000);
  wait.tv_nsec = (us % (1000 * 1000)) * 1000;
  nanosleep(&wait, NULL);
}

int tdiff_usecs(struct timeval t2, struct timeval t1)
{
  return (t2.tv_sec-t1.tv_sec)*1000000 + (t2.tv_usec-t1.tv_usec);
}


int
main(int argc, char *argv[])
{
  int h,i,j,k,kk,m, evt;
  int fnMPD=10;
  int error_count;
  uint32_t hcount;
  int scount, sfreq;
  int sch0, sch1;
  int hgain;
  int rdone, rtout, trigtout;

  int dCnt;
  int channel_data_offset[16];
  int ch_nwread[16];
  int timeout = 0;

  uint16_t mfull,mempty;
  uint32_t e_head, e_head0, e_size;
  uint32_t e_data32[130];
  uint32_t e_trai, e_eblo;

#define MAX_HDATA 4096
#define MAX_SDATA 1024
  uint32_t hdata[MAX_HDATA]; // histogram data buffer
  uint32_t sdata[MAX_SDATA]; // samples data buffer

  char outfile[1000];
  int acq_mode = 1;
  int n_event=3;//00000;
  int Hzcnt = 100;

  if (argc<2) {
    printf("SYNTAX: %s [out_data_file 0xacq_mode #events]\n",argv[0]);
    printf("        acq_mode_bit0 : EVENT readout");
    printf("        acq_mode_bit1 : SAMPLE check");
    printf("        acq_mode_bit2 : HISTO output");
    printf("        #events: number of events only for EVENT_READOUT\n");
  }

  if (argc>1) {
    sprintf(outfile,"%s",argv[1]);
  } else {
    sprintf(outfile,"out.txt");
  }

  if (argc>2) {
    sscanf(argv[2],"%x",&acq_mode);
  }

  if (argc>3) {
    sscanf(argv[3],"%d",&n_event);
  }

  printf("\nMPD Library Tests\n");
  printf("----------------------------\n");
  printf(" outfile = %s\n",outfile);
  printf(" acq_mode= 0x%x\n",acq_mode);
  printf(" n_event = %d\n", n_event);


  int vint_data;
  uint32_t v_data;
#define MPD_TIMEOUT 10

  extern int mpdOutputBufferBaseAddr;	/* output buffer base address */
  extern int mpdOutputBufferSpace;	/* output buffer space (8 Mbyte) */
#define DMA_BUFSIZE 80000

  //Output file TAG
#define VERSION_TAG 0xE0000000
#define EVENT_TAG   0x10000000
#define MPD_TAG     0x20000000
#define ADC_TAG     0x30000000
#define HEADER_TAG  0x00000040 //devel
  //#define HEADER_TAG  0x40000000 //INFN
#define DATA_TAG    0x0
#define TRAILER_TAG 0x00000050 //devel
  //#define TRAILER_TAG 0x50000000 //INFN

#define FILE_VERSION 0x1
  // End of MPD definition


  /* Default block level */
  unsigned int BLOCKLEVEL = 1;

#define BUFFERLEVEL 1

  int mpd_evt[21];
  //int evt;


  ///VME

  if(vmeOpenDefaultWindows()!=OK)
    {
      printf("ERROR opening default VME windows\n");
      goto CLOSE;
    }

  /* Create the event buffers */
  vmeIN  = dmaPCreate("vmeIN", DMA_BUFSIZE, 1, 0);
  vmeOUT = dmaPCreate("vmeOUT", 0, 0, 0);
  /* dmaPStatsAll(); */
  /* dmaPReInitAll(); */

  vmeDmaConfig(2,2,0); // 2,2,0 - A32-BLK32-SST160  2,3,0 A32-MBLK-sst160
  vmeDmaAllocLLBuffer();

  /*****************
   *   MPD SETUP
   *****************/
  int rval = OK;

  /**
   * Read config file and fill internal variables
   *
   */
  mpdConfigInit("../cfg/config_apv.txt");
  mpdConfigLoad();

  /**
   * Init and config MPD+APV
   */

  // discover MPDs and initialize memory mapping
  mpdInit(0x80000,0x80000,21,0x0);



  fnMPD = mpdGetNumberMPD();
  printf("-------------Number of MPDs:%d\n",fnMPD);

  if (fnMPD<=0) { // test all possible vme slot ?
    printf("ERR: no MPD discovered, cannot continue\n");
    return -1;
  }

  printf(" MPD discovered = %d\n",fnMPD);

  // APV configuration on all active MPDs
  for (k=0;k<fnMPD;k++) { // only active mpd set
    i = mpdSlot(k);

    // first test MPD histo memory write/read
    printf(" test mpd %d histo memory\n",i);

    mpdHISTO_Clear(i,0,-1);
    mpdHISTO_Read(i,0,hdata);
    error_count=0;
    for (j=0;j<MAX_HDATA;j++) {
      // printf("Me-histo read/write ch=%d rval=0x%x\n",j,hdata[j]);
      if (hdata[j]!=j) {
	printf("ERROR matching histo read/write ch=%d rval=0x%x\n",j,hdata[j]);
	error_count++;
      }
    }
    if (error_count) {
      printf("ERROR: HISTO Test fail %d time / %d attempts\n",error_count, MAX_HDATA);
    } else {
      printf("HISTO Read/Write test SUCCESS on MPD slot %d\n",i);
    }
    // END of first test MPD histo memory write/read


    printf(" Try initialize I2C mpd in slot %d\n",i);
    if (mpdI2C_Init(i) != OK) {
      printf("WRN: I2C fails on MPD %d\n",i);
    }
    if (mpdI2C_ApvReset(i) != OK) {
      printf("WRN: I2C ApvReset fails on MPD %d\n",i);
    }

    printf("Try APV discovery and init on MPD slot %d\n",i);
    if (mpdAPV_Scan(i)<=0) { // no apd found, skip next
      //  continue;
    }

    // board configuration (APV-ADC clocks phase)
    printf("Do DELAY setting on MPD slot %d\n",i);
    mpdDELAY25_Set(i, mpdGetAdcClockPhase(i,0), mpdGetAdcClockPhase(i,1));

    // apv reset
    printf("Do APV reset on MPD slot %d\n",i);
    if (mpdI2C_ApvReset(i) != OK) {
      printf("ERR: apv resert faild on mpd %d\n",i);
    }


    // apv configuration

    //   do{
    printf("Configure single APV on MPD slot %d\n",i);
    for (j=0; j < mpdGetNumberAPV(i); j++) {
      if (mpdApvEnabled(i,j)) {
	if (mpdAPV_Config(i,j) != OK) {
	  printf("ERR: config apv card %d failed in mpd %d\n",j,i);
	}
      }
      int count =0;
      /*       do{
	       count++;
	       printf("Configure again, single APV on MPD slot %d\n",i);
	       mpdAPV_Config(i,j);
	       printf("Configure apv card %d in MPD slot %d\n",j,i);
	       } while (count<1);
      */
    }
    //  } while (1);

    // configure adc on MPD
    printf("Configure ADC on MPD slot %d\n",i);
    mpdADS5281_Config(i);

    // configure fir
    // not implemented yet

    // 101 reset on the APV
    printf("Do 101 Reset on MPD slot %d\n",i);
    mpdAPV_Reset101(i);

    // <- MPD+APV initialization ends here

  } // end loop on mpds



  /*------- v262 check firmware ----------------*/

  // printf("\n\n\n\n\n \t\t\t\t v262 check !!!!!!!!\n\n\n\n\n");
  /*
    unsigned int v262_Local_Addr;
    int res= vmeBusToLocalAdrs(0x39, (char *) v262_BASE_ADDR, (char **) &v262_Local_Addr);
    printf("Result: %i \n", res);
    unsigned int * addr;
    addr = v262_Local_Addr + V262_FIRM_REVISION_R;
    printf ("v262 Firmware : %x \n -------------!!!!!! ", vmeRead16(addr));
    //  printf ("v262 Firmware : %x \n -------------!!!!!! ", vmeBusRead16(0x2f,&addr));
    unsigned int rval;
    int mem_res=vmeMemProbe((char *)addr,2,(char *) &rval);
    printf("MemProbe return value: result = %i , ReadVal= %i \n", mem_res,  rval);

    //Trigger latching
    unsigned int *latch_addr;
    latch_addr = v262_Local_Addr + V262_NIM_LEVEL_W;
    unsigned int *pulse_nim_addr;
    pulse_nim_addr = v262_Local_Addr + V262_NIM_PULSE_W;
    unsigned int *latch_to_poll_event;
    latch_to_poll_event = v262_Local_Addr + V262_NIM_LEVEL_R;

    vmeWrite16(pulse_nim_addr, 1); //In order to activate the latching
    vmeWrite16(latch_addr, 1);  // Turn on microbusy latch = Disable the trigger
  */

  /*
  // Latch debuging loop
  int counter = 0;
  vmeWrite16(latch_addr, 0); //Turn off microbusy latch = Enable the trigger
  while(counter <1) {

  vmeWrite16(pulse_nim_addr, 1); //In order to activate the latching
  //vmeWrite16(latch_addr, 1);  // Turn on microbusy latch = Disable the trigger

  usleep(10000);
  //vmeWrite16(latch_addr, 0); //Turn off microbusy latch = Enable the trigger
  //vmeWrite16(pulse_nim_addr, 0);

  //usleep(10000);

  printf("next itteration:!!!\n");
  //counter ++;
  }
  */
  /*-------- end v262 --------------------------*/



  /**********************************
   * SAMPLES TEST
   * sample APV output at 40 MHz
   * only for testing not for normal daq
   **********************************/
  if (acq_mode & 0x2) {

    for (k=0;k<fnMPD;k++) { // only active mpd set
      i = mpdSlot(k);

      mpdSetAcqMode(i, "sample");

      // load pedestal and thr default values
      mpdPEDTHR_Write(i);

      // set clock phase
      mpdDELAY25_Set(i, 27, 27); // use 20 as test, but other values may work better

      // enable acq
      mpdDAQ_Enable(i);

      // wait for FIFOs to get full
      mfull=0;
      mempty=1;
      do {
	mpdFIFO_GetAllFlags(i,&mfull,&mempty);
	printf("SAMPLE test: %x (%x) %x\n",mfull,mpdGetApvEnableMask(i),mempty);
	sleep(1);
      } while (mfull!=mpdGetApvEnableMask(i));

      // read data from FIFOs and estimate synch pulse period
      for (j=0;j<16;j++) { // 16 = number of ADC channel in one MPD
	if (mpdGetApvEnableMask(i) & (1<<j)) { // apv is enabled
	  mpdFIFO_Samples(i,j, sdata, &scount, MAX_SDATA, &error_count);
	  if (error_count != 0) {
	    printf("ERROR returned from FIFO_Samples %d\n",error_count);
	  }
	  printf("MPD/APV : %d / %d, peaks around: ",i,j);
	  sch0=-1;
	  sch1=-1;
	  sfreq=0;
	  for (h=0;h<scount;h++) { // detects synch peaks
	    //  printf("%d "/*"%04x "*/, sdata[h]); // output data
	    if (sdata[h]>2000) { // threshold could be lower
	      if (sch0<0) { sch0=h; sch1=sch0;} else {
		if (h==(sch1+1)) { sch1=h; } else {
		  printf("%d-%d (v %d) ",sch0,sch1, sdata[h]);
		  sch0=h;
		  sch1=sch0;
		  sfreq++;
		}
	      }
	    }
	  }
	  printf("\n Estimated synch period = %f (us) ,expected (20MHz:1.8, 40MHz:0.9)\n",((float) scount)/sfreq/40);
	}
      }

    } // end loop on mpds

  } // sample check mode

  // catch Ctrl-C
  signal(SIGINT, ctrl_c_HANDLER); ///Anusha did for ctrl_c


  /********************************************
   * HISTO Mode; sampled data are histogrammed
   * only for testing, non for normal daq
   ********************************************/

  if (acq_mode & 0x4) {

    for (k=0;k<fnMPD;k++) { // only active mpd set
      i = mpdSlot(k);

      mpdSetAcqMode(i, "histo");
      hgain = 5; // this will be read from config file

      // load pedestal and thr default values
      mpdPEDTHR_Write(i);


      // set ADC gain
      mpdADS5281_SetGain(i,0,hgain,hgain,hgain,hgain,hgain,hgain,hgain,hgain);

      // loop on adc channels

      for (j=0;j<16;j++) {

	mpdHISTO_Clear(i,j,0);

	mpdHISTO_Start(i,j);

	sleep(1); // wait to get some data

	mpdHISTO_Stop(i,j);

	hcount=0;
	mpdHISTO_GetIntegral(i,j,&hcount);
	error_count = mpdHISTO_Read(i,j, hdata);

	if (error_count != OK) {
	  printf("ERROR: reading histogram data from MPD %d ADCch %d\n",i,j);
	  continue;
	}

	printf(" ### histo peaks integral MPD/ADCch = %d / %d (total Integral= %d)\n",i,j,hcount);
	sch0=-1;
	sch1=0;
	for (h=0;h<4096;h++) {

	  if (hdata[h]>0) {
	    if (sch0<0) { printf(" first bin= %d: ", h); } else {
	      if ((sch0+1) < h) { printf( "%d last bin= %d\n first bin= %d: ",sch1,sch0,h); sch1=0; }
	    }
	    sch1 += hdata[h];
	    sch0=h;
	  }

	  //	if ((h%64) == 63) { printf("\n"); }
	}
	if (sch1>0) {
	  printf( "%d last bin= %d\n",sch1,sch0);
	} else { printf("\n");}

      } // end loop adc channels in histo mode

    }

  } // histo mode

    /**************************
     * "Event" readout
     *  normal DAQ starts here
     **************************/

  if (acq_mode & 1) {

    // open out file
    FILE *fout;
    char *outputfilepath = "./"; /// use outputfile path as a GLOBAL variable.
    char *outputfilename = "run";
    char outfilename[512];

    fout = fopen(outfile,"w");
    if (fout == NULL) { fout = stdout; }
    //  fprintf(fout,"VERSION_TAG %x\n", FILE_VERSION | VERSION_TAG);

    for (k=0;k<fnMPD;k++) { // only active mpd set
      i = mpdSlot(k);

      // mpd latest configuration before trigger is enabled
      //mpdSetAcqMode(i, "event");
      mpdSetAcqMode(i, "processsssssss");

      // load pedestal and thr default values
      mpdPEDTHR_Write(i);

      // enable acq
      mpdOutputBufferBaseAddr = 0x08000000;
      printf("%s: INFO: mpdOutputBufferBaseAddr = 0x%08x\n",
	     __func__, mpdOutputBufferBaseAddr);
      mpdDAQ_Enable(i);

    }
    // -> now trigger can be enabled

    //////////////Get the outputfilepath defined in the config file ////////////

    //  printf("DAQhisto: Save histo data to an output file = %s/%s\n", outputfilepath,outputfilename)
    // Check outputfilename: if it is "run", set it to "historunXXXXX.dat"
    //   with XXXXX being the runnumber:

    if (!strcmp(outputfilename, "run"))
      {

	//  printf(outputfilename, "%s/%s", outputfilepath, outputfilename);
        FILE *runnofile = fopen("last.run", "r");
	if (runnofile==NULL)
	  {
	    printf(" *** ERROR *** - Cannot open >last.run< \n");
	    exit(1);
	  }

	int runno=0;
	fscanf(runnofile, "%d", &runno);
	fclose(runnofile);
	// increment runnumber:
	runno++;

	// re-create runnofile:
	runnofile = fopen("last.run", "w");
        fprintf(runnofile, "%d\n", runno);
        fclose(runnofile);
        sprintf(outfilename, "%s/GEMrun%05d.dat", outputfilepath, runno);
	//	printf(" ---> writing events data to >%s<...\n", outfilename);

      }

    else
      {
	printf(" *** ERROR *** - Cannot read the config file");

	// char buff[256];
	// sprintf(buff, "%s", outfilename);
	// sprintf(outfilename, "%s/%s", outfilepath, outfilename);

      };

    ////////////////////////////////
    ///////////////////////////

    //	if (argc>1) {
    //	  sprintf(outfile,"%s",argv[1]);
    //	} else {
    sprintf(outfile,outfilename);
    //	}


    fout = fopen(outfile,"w");
    //sprintf(outfile,outfilename);
    // sprintf(outfile,"histo.txt");

    if (fout == NULL) {
      // fout = stdout;
      printf("ERROR: Cannot open %s for writing...\n",outfile);
      return 1;
    }
    else {
      // printf(" Writing histogram data to %s\n",outfile);
    };
    fflush(stdout);

    ///////////////////////

    gettimeofday(&t1, NULL); // Get Time t1
    gettimeofday(&t8, NULL); // Get Time t8
    /////////////////////////////


    evt=0;

    //vmeWrite16(pulse_nim_addr, 1); //In order to activate the latching
    //vmeWrite16(latch_addr, 0); //Turn off microbusy latch = Enable the trigger

    //printf("\n ===== TRIGGER enabled ===== \n");

    printf(" ============ START ACQ ===========\n");


#ifdef DEBUG
    printf(" ---- Waiting for trigger now -----\n");
#endif

    do { // simulate loop on trigger
      //  sleep(1); // wait for event
#ifdef DEBUG
      printf(" ---- Waiting for event %d to occur -----\n",evt);
#endif

      /* Readout MPD */
      // open out file

      int UseSdram, FastReadout;
      int empty, full, nwords, obuf_nblock;
      int nwread;
      int iw, blen;
      int verbose_level = 2;

      UseSdram = mpdGetUseSdram(mpdSlot(0));	// assume sdram and fastreadout are the same for all MPDs
      FastReadout = mpdGetFastReadout(mpdSlot(0));

      printf("\n\n ========= UseSDRAM= %d , FastReadout= %d\n", UseSdram, FastReadout);

      // -> now trigger can be enabled


      /* Readout MPD */
      // open out file


      int tout, impd, id;
      for (impd = 0; impd < fnMPD; impd++)
	{				// only active mpd set
	  GETEVENT(vmeIN, impd);  /* Initialize dma_dabufp pointer to current event buffer */

	  id = mpdSlot(impd);
	  //   vmeSetQuietFlag(1);
	  //   vmeClearException(1);
	  mpdArmReadout(id);		// prepare internal variables for readout @@ use old buffer scheme, need improvement
	  //blen = mpdApvGetBufferAvailable(i, 0);

	  blen = DMA_BUFSIZE;
	  nwread = 0;

	  if (UseSdram)
	    {

	      int sd_init, sd_overrun, sd_rdaddr, sd_wraddr, sd_nwords;

	      mpdSDRAM_GetParam(id, &sd_init, &sd_overrun, &sd_rdaddr, &sd_wraddr,
				&sd_nwords);
	      if (verbose_level > 0)
		printf
		  ("SDRAM status: init=%d, overrun=%d, rdaddr=0x%x, wraddr=0x%x, nwords=%d\n",
		   sd_init, sd_overrun, sd_rdaddr, sd_wraddr, sd_nwords);

	      tout = 0;
	      while (mpdOBUF_GetBlockCount(id) == 0 && tout < 1000)
		{
		  usleep(10);
		  tout++;
		}
	      if (tout == 1000)
		{
		  timeout = 1;

		  printf
		    ("WARNING: *** Timeout while waiting for data in mpd %d - check MPD/APV configuration\n",
		     id);
		}

	      obuf_nblock = mpdOBUF_GetBlockCount(id);
	      // evb_nblock = mpdGetBlockCount(i);

	      if (obuf_nblock > 0)
		{			// read data

		  mpdOBUF_GetFlags(id, &empty, &full, &nwords);

		  if (verbose_level > 0)
		    printf("OBUFF status: empty=%d, full=%d, nwords=%d\n", empty,
			   full, nwords);

		  if (FastReadout > 0)
		    {		//64bit transfer
		      if (nwords < 128)
			{
			  empty = 1;
			}
		      else
			{
			  nwords *= 2;
			}
		    }

		  if (full)
		    {
		      printf
			("\n\n **** OUTPUT BUFFER FIFO is FULL in MPD %d !!! RESET EVERYTHING !!!\n\n",
			 id);
		      //exit(1);
		    }
		  if (verbose_level > 0)
		    printf("Data waiting to be read in obuf: %d (32b-words)\n",
			   nwords);
		  if (nwords > 0)	// was >=
		    {
		      if (nwords > blen / 4)
			{
			  nwords = blen / 4;
			}

		      mpdOBUF_Read(id, dma_dabufp, nwords, &nwread);

		      if (verbose_level > 0)
			printf
			  ("try to read %d 32b-words 128b-aligned from obuf, got %d 32b-words\n",
			   nwords, nwread);

		      if (nwords != nwread)
			{
			  printf
			    ("ERROR: 32bit-word read count does not match %d %d\n",
			     nwords, nwread);
			}
		      dma_dabufp += nwread;
		    }
		}
	      else
		usleep(10);

	    }
	  else
	    {			// if not Sdram
	      // Individual Channel readout
	      int ichan = 0;
	      memset((char *) channel_data_offset, 0, sizeof(channel_data_offset));
	      memset((char *) ch_nwread, 0, sizeof(ch_nwread));

	      for(ichan = 0; ichan < 16; ichan++)
		{
		  mpdFIFO_IsEmpty(id, ichan, &empty);	//  read fifo channel status

		  if (!empty)
		    {			// read channel fifo
		      nwread = blen / 4;

		      // channel_data_offset[] are index references in data[] to the
		      // beginning of each channel
		      // (these might be helpful if you need to add headers/trailers
		      //  to the data)
		      channel_data_offset[ichan] =
			((int) (dma_dabufp) - (int) (&the_event->data[0])) >> 2;

		      mpdFIFO_ReadSingle(id, ichan,
					 dma_dabufp, // Where to put the data
					 &ch_nwread[ichan],    // Max data to take [4byte words]
					 20);        // Nax number of tries to wait until data is ready
		      if (ch_nwread[ichan] == 0)
			{
			  printf("ERROR: word read count is 0, while some words are expected back\n");
			}
		      else
			{
			  dma_dabufp += ch_nwread[ichan];
			}
		    }
		  nwread += ch_nwread[ichan];
		}

	    }

	  PUTEVENT(vmeOUT); /* Put this event buffer into the vmeOUT queue */

	  // Process &| write data to disk
	  DMANODE *outEvent = dmaPGetItem(vmeOUT);

	  if (outEvent->length > 0)
	    {			// data need to be written on file

	      int zero_count;
	      zero_count = 0;
	      if (verbose_level > 1)
		printf("MPD Slot: %d (dump data on screen)\n", id);	// slot

	      for (iw = 0; iw < outEvent->length; iw++)
		{
		  uint32_t datao;
		  datao = LSWAP(outEvent->data[iw]);

		  if (datao == 0)
		    {
		      zero_count++;
		    }

		  if (verbose_level > 1)
		    {		// && ((iw<500) || ((outEvent->length-iw)<501))) {

		      if ((verbose_level > 2) || (iw < 16)
			  || ((outEvent->length - iw) <= (16 + outEvent->length % 8)))
			{
			  if ((iw % 8) == 0)
			    {
			      printf("0x%06x:", iw);
			    }
			  printf(" 0x%08x", datao);

			  if (((iw % 8) == 7) || (iw == (outEvent->length - 1)))
			    {
			      printf("\n");
			    }
			}

		    }
		  //      if (verbose_level > 1 && iw==500) { printf(" ....\n"); }

		  if ((datao & 0x00E00000) == 0x00A00000)
		    {		// EVENT TRAILER
		      //          printf("\n\n   ***** EVENT TRAILER: datao = 0x%08X mpd_evt[%d] = %d\n",datao,i, mpd_evt[i]);
		      mpd_evt[id]++;
		      //          evt=mpd_evt[i];
		    }
		  //        evt = (evt > mpd_evt[i]) ? mpd_evt[i] : evt; // evt is the smallest number of events of an MPD
		}


	      printf("MPD %d: nwords=%d nwcount=%lu zero_count=%d evt %d\n", id,
		     nwords, outEvent->length, zero_count, mpd_evt[id]);
	    }

	  dmaPFreeItem(outEvent); // Free event buffer and put it back into the vmeIN queue

	}				// active mpd loop




      // maybe user is tired and wants to stop the run:
      //printf("crl_c = %d\n", ctrl_c);
    } while ((evt<n_event) && (ctrl_c == false)); // end loop on events

    //} while ((evt<n_event)); // end loop on events
    if (fout != stdout)  fclose(fout);

    printf("done! \n");
    printf(" ---> writing events data to >%s<...\n",outfile);
    printf(" === STOP DAQ ===\n");

  }



 CLOSE:

  //vmeWrite16(pulse_nim_addr, 1); //In order to activate the latching
  //vmeWrite16(latch_addr, 0); //Turn off microbusy latch = Enable the trigger

  //Release memory
  dmaPFreeAll();

  vmeCloseDefaultWindows();

  exit(0);

}


/*
  Local Variables:
  compile-command: "make -B DEBUG=0"
  End:
*/
