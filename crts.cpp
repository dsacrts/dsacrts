#include <stdio.h>
#include <stdlib.h>
#include <sysexits.h>
#include <math.h>
#include <complex>
#include <liquid/liquid.h>
//Library containing functions for FFT and Energy Detection
#include <fftw3.h>
// Definition of liquid_float_complex changes depending on
// whether <complex> is included before or after liquid.h
#include <liquid/ofdmtxrx.h>
#include <time.h>
#include <string.h>
// For Threading (POSIX Threads)
#include <pthread.h>
// For config file
#include <libconfig.h>

//TCP Header Files
#include <sys/socket.h> // for socket(), connect(), send(), and recv() 
#include <sys/types.h>
#include <arpa/inet.h>  // for sockaddr_in and inet_addr() 
#include <string.h>     // for memset() 
#include <unistd.h>     // for close() 
#include <errno.h>
#include <sys/types.h>  // for killing child process
#include <signal.h>     // for killing child process
#include <uhd/usrp/multi_usrp.hpp>
#include <getopt.h>     // For command line options
#define MAXPENDING 5

// SO_REUSEPORT is defined only defined with linux 3.10+.
// Makes compatible with earlier kernels.
#ifndef SO_REUSEPORT
#define SO_REUSEPORT SO_REUSEADDR
#endif


void usage() {
    printf("crts -- Test cognitive radio engines. Data is logged in the 'data' directory to a file named 'data_crts' with date and time appended.\n");
    printf("  -u,-h  :   usage/help\n");
    printf("  -q     :   quiet - do not print debug info\n");
    printf("  -v     :   verbose - print debug info to stdout (default)\n");
    printf("  -d     :   print data to stdout rather than to file (implies -q unless -v given)\n");
    printf("  -r     :   real transmissions using USRPs (opposite of -s)\n");
    printf("  -s     :   simulation mode (default)\n");
    printf("  -p     :   server port (default: 1402)\n");
    printf("  -c     :   controller - this crts instance will act as experiment controller (needs -r)\n");
    printf("  -a     :   server IP address (when not controller. default: 127.0.0.1)\n");
    printf("  -f     :   center frequency [Hz] (when not controller. default: 460 MHz)\n");
    printf("  -b     :   bandwidth [Hz], (when not controller. default: 1.0 MHz)\n");
    printf("  -G     :   uhd rx gain [dB] (when not controller. default: 20dB)\n");
    printf("  -M     :   number of subcarriers (when not controller. default: 64)\n");
    printf("  -C     :   cyclic prefix length (when not controller. default: 16)\n");
    printf("  -T     :   taper length (when not controller. default: 4)\n");
    printf("  -D     :   using DSA\n");
    printf("  -B     :   node is a broadcasting transmitter that can be linked to by receivers\n");
	printf("  -P     :   primary user\n");
	printf("  -S     :   secondary user\n");
	printf("  -R     :   broadcasting receiver\n");
	printf("  -Q     :   run test program (for debugging)\n");
    //printf("  f     :   center frequency [Hz], default: 462 MHz\n");
    //printf("  b     :   bandwidth [Hz], default: 250 kHz\n");
    //printf("  G     :   uhd rx gain [dB] (default: 20dB)\n");
    //printf("  t     :   run time [seconds]\n");
    //printf("  z     :   number of subcarriers to notch in the center band, default: 0\n");
}






struct CognitiveEngine {
    char modScheme[30];
    char crcScheme[30];
    char innerFEC[30];
    char outerFEC[30];
    char outerFEC_prev[30];
    char adaptationCondition[30];
    float default_tx_power;
    char adaptation[30];
    char goal[30];
    float threshold;
    // TODO: For latestGoalValue, Use different type of variable depending on
    //  what its being compared to
    float latestGoalValue;
    float weightedAvg; 
    float PER;
    float BERLastPacket;
    float BERTotal;
    float frequency;
    float bandwidth;
    float txgain_dB;
    float uhd_txgain_dB;
    float delay_us;
    float weighted_avg_payload_valid_threshold;
    float PER_threshold;
    float BER_threshold;
    float FECswitch;
    double startTime;
    double runningTime; // In seconds
    int iteration;
	unsigned int bitsPerSym;
    unsigned int validPayloads;
    unsigned int errorFreePayloads;
    unsigned int frameNumber;
    unsigned int lastReceivedFrame;
    unsigned int payloadLen;
    unsigned int payloadLenIncrement;
    unsigned int payloadLenMax;
    unsigned int payloadLenMin;
    unsigned int numSubcarriers;
    unsigned int CPLen;
    unsigned int taperLen;
	unsigned int averaging;
	float metric_mem[100]; // For computing running average
	float averagedGoalValue;

};

struct Scenario {
    int addAWGNBasebandTx; //Does the Scenario have noise?
    int addAWGNBasebandRx; //Does the Scenario have noise?
    float noiseSNR;
    float noiseDPhi;
    
    int addInterference; // Does the Scenario have interference?
    
    int addRicianFadingBasebandTx; // Does the Secenario have fading?
    int addRicianFadingBasebandRx; // Does the Secenario have fading?
    float fadeK;
    float fadeFd;
    float fadeDPhi;

	int addCWInterfererBasebandTx; // Does the Scenario have a CW interferer?
	int addCWInterfererBasebandRx; // Does the Scenario have a CW interferer?
	float cw_pow;
	float cw_freq;
};

struct rxCBstruct {
    unsigned int serverPort;
    int verbose;
    float bandwidth;
    char * serverAddr;
	msequence * rx_ms_ptr;
    int ce_num;
    int sc_num;
    int frameNum;
	int client;
};

//Struct passed to dsacallback when a dsa node receives a liquid frame
struct dsaCBstruct {
    unsigned int serverPort;
    int verbose;
    float bandwidth;
    char * serverAddr;
	msequence * rx_ms_ptr;
    int ce_num;
    int sc_num;
    int frameNum;
	int client;

	//Tells program whether the primary user is on or not
	int primaryon;

	//Tells program if secondary transmissions have been received
	int secondarysending;

	//Char that indicates whether the node is primary or secondary and if it is a transmitter or receiver
	char usrptype;

	int number;

	//Records the dsa type being used
	char dsatype;

	//The detection type being used, either match filter or energy detection
	char detectiontype;
};

struct feedbackStruct {
    int             header_valid;
    int             payload_valid;
    unsigned int    payload_len;
    unsigned int    payloadByteErrors;
    unsigned int    payloadBitErrors;
    unsigned int    iteration;
	//int				ce_num;
	//int				sc_num;
    float           evm;
    float           rssi;
    float           cfo;
	int				primaryon;
	int				primary;
	int				secondary;
	int 			block_flag;
};



//Structure for using TCP to pass both feedback structs and info on when primaries and secondaries
//turn on and off
struct message{
	//Type signifies where the message came from
	//P - primary transmitter
	//S - secondary transmitter
	//p - primary receiver
	//s - seconary receiver
	char type;

	//Purpose determines the info the message was meant to send
	//t - transmitter is turning on
	//r - transmitter is turning off
	//f - feedback from receiver of primary transmission
	//F - feedback form receiver of secondary transmission or primary transmitter signalling its finished
	//its cycles
	//u - a liquid frame was received with a header that can't be identified as primary or secondary
	//D - energy detector saying it detected the primary user
	//d - energy detector saying it didn't detect the primary user
	//P - receiver saying it detected the primary user
	//S - receiver saying it detected the secondary user
	char purpose;

	//Feedback that receivers can use TCP to pass to transmitters
	struct feedbackStruct feed;

	//The number increases by one when a message is sent to insure that a program doesn't read a message twice
	int number;

	//Signals when a new message can be read
	int msgreceived;

	//Tells the receiving program where the message came from
	int client;
};

//Adds together the values of 2 feedback structures
struct feedbackStruct feedbackadder(struct feedbackStruct fb1, struct feedbackStruct fb2){
	struct feedbackStruct resultfb;
	resultfb.header_valid = fb1.header_valid + fb2.header_valid;
	resultfb.payload_valid = fb1.payload_valid + fb2.payload_valid;
   	resultfb.payload_len = fb1.payload_len + fb2.payload_len;
	resultfb.payloadByteErrors = fb1.payloadByteErrors + fb2.payloadByteErrors;
   	resultfb.payloadBitErrors = fb1.payloadBitErrors + fb2.payloadBitErrors;
	resultfb.iteration = fb1.iteration + fb2.iteration;
   	resultfb.evm = fb1.evm + fb2.evm;
	resultfb. rssi = fb1.rssi + fb2.rssi;
	resultfb.cfo = fb1.cfo + fb2.cfo;
	return resultfb;
}

struct serverThreadStruct {
    unsigned int serverPort;
    struct feedbackStruct * fb_ptr;
	struct message * m_ptr;
	char type;
};

struct serveClientStruct {
	int client;
	struct feedbackStruct * fb_ptr;
	struct message * m_ptr;
};

struct enactScenarioBasebandRxStruct {
    ofdmtxrx * txcvr_ptr;
    struct CognitiveEngine * ce_ptr;
    struct Scenario * sc_ptr;
};

//Struct passed to thread that interprets receiver feedback for broadcasting transmitters
struct broadcastfeedbackinfo{
	struct message * m_ptr;
	int client;
	int * msgnumber;
	char user;
	int primaryon;
	int energydetected;
};

//Struct for passing variables to fftscan function
struct fftStruct {
	float bandwidth;
	int repeat;
	float channelbandwidth;
	int rate;
	int numbins;
	char * antennae;
	float noiseadder;
	int measuredbins;
	int testnumber;
	int debug;
	float gain;
	float noisemult;
	int noisefloorrepeat;
	int noisefloormeasuredbins;
	int noisefloortestnumber;
};

struct scenarioSummaryInfo{
	int total_frames[60][60];
	int valid_headers[60][60];
	int valid_payloads[60][60];
	int total_bits[60][60];
	int bit_errors[60][60];
	float EVM[60][60];
	float RSSI[60][60];
	float PER[60][60];
};

struct cognitiveEngineSummaryInfo{
	int total_frames[60];
	int valid_headers[60];
	int valid_payloads[60];
	int total_bits[60];
	int bit_errors[60];
	float EVM[60];
	float RSSI[60];
	float PER[60];
};



// Default parameters for a Cognitive Engine
struct CognitiveEngine CreateCognitiveEngine() {
    struct CognitiveEngine ce = {};
    ce.default_tx_power = 30.0;
    ce.threshold = 1.0;                 // Desired value for goal
    ce.latestGoalValue = 0.0;           // Value of goal to be compared to threshold
    ce.iteration = 1;                 // Count of total simulations.
    ce.payloadLen = 120;                // Length of payload in frame generation
    ce.payloadLenIncrement = 2;         // How much to increment payload in adaptations
                                        // Always positive.

    ce.payloadLenMax = 500;             // Max length of payload in bytes
    ce.payloadLenMin = 20;              // Min length of payload in bytes
    ce.numSubcarriers = 64;             // Number of subcarriers for OFDM
    ce.CPLen = 16;                      // Cyclic Prefix length
    ce.taperLen = 4;                     // Taper length
    ce.weightedAvg = 0.0;
    ce.PER = 0.0;
    ce.BERLastPacket = 0.0;
    ce.BERTotal = 0.0;
	ce.bitsPerSym = 1;
    ce.frameNumber = 0;
    ce.lastReceivedFrame = 0;
    ce.validPayloads = 0;
    ce.errorFreePayloads = 0;
    ce.frequency = 460.0e6;
    ce.bandwidth = 1.0e6;
    ce.txgain_dB = -12.0;
    ce.uhd_txgain_dB = 40.0;
    ce.startTime = 0.0;
    ce.runningTime = 0.0; // In seconds
    ce.delay_us = 1000000.0; // In useconds
    ce.weighted_avg_payload_valid_threshold = 0.5;
    ce.PER_threshold = 0.5;
    ce.BER_threshold = 0.5;
    ce.FECswitch = 1;
	ce.averaging = 1;
	memset(ce.metric_mem,0,100*sizeof(float));
	ce.averagedGoalValue = 0;
    strcpy(ce.modScheme, "QPSK");
    strcpy(ce.adaptation, "mod_scheme->BPSK");
    strcpy(ce.goal, "payload_valid");
    strcpy(ce.crcScheme, "none");
    strcpy(ce.innerFEC, "none");
    strcpy(ce.outerFEC, "Hamming74");
    strcpy(ce.outerFEC_prev, "Hamming74");
    //strcpy(ce.adaptationCondition, "weighted_avg_payload_valid"); 
    strcpy(ce.adaptationCondition, "packet_error_rate"); 
    return ce;
} // End CreateCognitiveEngine()

// Default parameter for Scenario
struct Scenario CreateScenario() {
    struct Scenario sc = {};
    sc.addAWGNBasebandTx = 0,
    sc.addAWGNBasebandRx = 0,
    sc.noiseSNR = 7.0f, // in dB
    sc.noiseDPhi = 0.001f,

    sc.addInterference = 0,

    sc.addRicianFadingBasebandTx = 0,
    sc.addRicianFadingBasebandRx = 0,
    sc.fadeK = 30.0f,
    sc.fadeFd = 0.2f,
    sc.fadeDPhi = 0.001f;

	sc.addCWInterfererBasebandTx = 0;
	sc.addCWInterfererBasebandRx = 0;
	sc.cw_pow = 0;
	sc.cw_freq = 0;

    return sc;
} // End CreateScenario()

// Defaults for struct that is sent to rxCallBack()
struct rxCBstruct CreaterxCBStruct() {
    struct rxCBstruct rxCB = {};
    rxCB.serverPort = 1402;
    rxCB.bandwidth = 1.0e6;
    rxCB.serverAddr = (char*) "127.0.0.1";
    rxCB.verbose = 1;
    return rxCB;
} // End CreaterxCBStruct()

// Defaults for struct that is sent to server thread
struct serverThreadStruct CreateServerStruct() {
    struct serverThreadStruct ss = {};
    ss.serverPort = 1402;
    ss.fb_ptr = NULL;
    return ss;
} // End CreateServerStruct()

//Defaults for struct that is sent to client threads
struct serveClientStruct CreateServeClientStruct() {
	struct serveClientStruct sc = {};
	sc.client = 0;
	sc.fb_ptr = NULL;
	return sc;
}; // End CreateServeClientStruct



void feedbackStruct_print(feedbackStruct * fb_ptr)
{
    // TODO: make formatting nicer
    printf("feedbackStruct_print():\n");
    printf("\theader_valid:\t%d\n",       fb_ptr->header_valid);
    printf("\tpayload_valid:\t%d\n",      fb_ptr->header_valid);
    printf("\tpayload_len:\t%u\n",        fb_ptr->payload_len);
    printf("\tpayloadByteErrors:\t%u\n",  fb_ptr->payloadByteErrors);
    printf("\tpayloadBitErrors:\t%u\n",   fb_ptr->payloadBitErrors);
    printf("\tframeNum:\t%u\n",           fb_ptr->iteration);
    //printf("\tce_num:\t%d\n",             fb_ptr->ce_num);
    //printf("\tsc_num:\t%d\n",             fb_ptr->sc_num);
    printf("\tevm:\t%f\n",                fb_ptr->evm);
    printf("\trssi:\t%f\n",               fb_ptr->rssi);
    printf("\tcfo:\t%f\n",                fb_ptr->cfo);
}

int readScMasterFile(char scenario_list[30][60], int verbose )
{
    config_t cfg;                   // Returns all parameters in this structure 
    config_setting_t *setting;
    const char *str;                // Stores the value of the String Parameters in Config file
    int tmpI;                       // Stores the value of Integer Parameters from Config file                

    char current_sc[30];
    int no_of_scenarios=1;
    int i;
    char tmpS[30];
    //Initialization
    config_init(&cfg);
   
   
    // Read the file. If there is an error, report it and exit. 
    if (!config_read_file(&cfg,"master_scenario_file.txt"))
    {
        fprintf(stderr, "\n%s:%d - %s", config_error_file(&cfg), config_error_line(&cfg), config_error_text(&cfg));
        fprintf(stderr, "\nCould not find master scenario file. It should be named 'master_scenario_file.txt'\n");
        config_destroy(&cfg);
        exit(EX_NOINPUT);
    }
    else
        //printf("\nFound master Scenario config file\n")
        ;

  
    //// Get the configuration file name. 
    //if (config_lookup_string(&cfg, "filename", &str))
    //    //printf("File Type: %s\n", str)
    //    ;
    //else
    //    fprintf(stderr, "No 'filename' setting in configuration file.\n");

    // Read the parameter group
    setting = config_lookup(&cfg, "params");
    if (setting != NULL)
    {
        
        if (config_setting_lookup_int(setting, "NumberofScenarios", &tmpI))
        {
            no_of_scenarios=tmpI;
            if (verbose)
                printf ("Number of Scenarios: %d\n",tmpI);
        }
        
       for (i=1;i<=no_of_scenarios;i++)
       //while (strcmp (status,"end"!=0))
       {
         strcpy (current_sc,"scenario_");
         sprintf (tmpS,"%d",i);
         //printf ("\n Scenario Number =%s", tmpS);
         strcat (current_sc,tmpS);
         //printf ("\n CURRENT SCENARIO =%s", current_sc);
         if (config_setting_lookup_string(setting, current_sc, &str))
          {
              strcpy(*((scenario_list)+i-1),str);          
              //printf ("\nSTR=%s\n",str);
          }
        /*else
            printf("\nNo 'param2' setting in configuration file.");
          */
        if (verbose)
            printf ("Scenario File: %s\n", *((scenario_list)+i-1) );
      } 
    }
    config_destroy(&cfg);
    return no_of_scenarios;
} // End readScMasterFile()

int readCEMasterFile(char cogengine_list[30][60], int verbose)
{
    config_t cfg;               // Returns all parameters in this structure 
    config_setting_t *setting;
    const char *str;            // Stores the value of the String Parameters in Config file
    int tmpI;                   // Stores the value of Integer Parameters from Config file             

    char current_ce[30];
    int no_of_cogengines=1;
    int i;
    char tmpS[30];
    //Initialization
    config_init(&cfg);
   
    //printf ("\nInside readCEMasterFile function\n");
    //printf ("%sCogEngine List[0]:\n",cogengine_list[0] );

    // Read the file. If there is an error, report it and exit. 
    if (!config_read_file(&cfg,"master_cogengine_file.txt"))
    {
        fprintf(stderr, "\n%s:%d - %s", config_error_file(&cfg), config_error_line(&cfg), config_error_text(&cfg));
        fprintf(stderr, "\nCould not find master file. It should be named 'master_cogengine_file.txt'\n");
        config_destroy(&cfg);
        exit(EX_NOINPUT);
    }
    else
        //printf("Found master Cognitive Engine config file.\n");
        ;
  
    //// Get the configuration file name. 
    //if (config_lookup_string(&cfg, "filename", &str))
    //    //printf("File Type: %s\n", str)
    //    ;
    //else
    //    fprintf(stderr, "No 'filename' setting in master CE configuration file.\n");

    // Read the parameter group
    setting = config_lookup(&cfg, "params");
    if (setting != NULL)
    {
        if (config_setting_lookup_int(setting, "NumberofCogEngines", &tmpI))
        {
            no_of_cogengines=tmpI;
            if (verbose)
                printf ("Number of Congnitive Engines: %d\n",tmpI);
        }
        
        for (i=1;i<=no_of_cogengines;i++)
        {
            strcpy (current_ce,"cogengine_");
            sprintf (tmpS,"%d",i);
            //printf ("\n Scenario Number =%s", tmpS);
            strcat (current_ce,tmpS);
            //printf ("\n CURRENT SCENARIO =%s", current_sc);
            if (config_setting_lookup_string(setting, current_ce, &str))
            {
                strcpy(*((cogengine_list)+i-1),str);          
                //printf ("\nSTR=%s\n",str);
            }
            if (verbose)
                printf ("Cognitive Engine File: %s\n", *((cogengine_list)+i-1) );
        } 
    }
    config_destroy(&cfg);
    return no_of_cogengines;
} // End readCEMasterFile()

///////////////////Cognitive Engine///////////////////////////////////////////////////////////
////////Reading the cognitive radio parameters from the configuration file////////////////////
int readCEConfigFile(struct CognitiveEngine * ce, char *current_cogengine_file, int verbose)
{
    config_t cfg;               // Returns all parameters in this structure 
    config_setting_t *setting;
    const char * str;           // Stores the value of the String Parameters in Config file
    int tmpI;                   // Stores the value of Integer Parameters from Config file
    double tmpD;                
    char ceFileLocation[60];

    strcpy(ceFileLocation, "ceconfigs/");
    strcat(ceFileLocation, current_cogengine_file);

    if (verbose)
        printf("Reading ceconfigs/%s\n", current_cogengine_file);

    //Initialization
    config_init(&cfg);

    // Read the file. If there is an error, report it and exit. 
    if (!config_read_file(&cfg,ceFileLocation))
    {
        fprintf(stderr, "\n%s:%d - %s", config_error_file(&cfg), config_error_line(&cfg), config_error_text(&cfg));
        config_destroy(&cfg);
        exit(EX_NOINPUT);
    }

    // Get the configuration file name. 
    //if (config_lookup_string(&cfg, "filename", &str))
    //{
    //    if (verbose)
    //        printf("Cognitive Engine Configuration File Name: %s\n", str);
    //}
    //else
    //    printf("No 'filename' setting in configuration file.\n");

    // Read the parameter group
    setting = config_lookup(&cfg, "params");
    if (setting != NULL)
    {
        // Read the strings
        if (config_setting_lookup_string(setting, "adaptation", &str))
        {
            strcpy(ce->adaptation,str);
            if (verbose) printf("Option to adapt: %s\n",str);
        }
       
        if (config_setting_lookup_string(setting, "goal", &str))
        {
            strcpy(ce->goal,str);
            if (verbose) printf("Goal: %s\n",str);
        }
        if (config_setting_lookup_string(setting, "adaptationCondition", &str))
        {
            strcpy(ce->adaptationCondition,str);
            if (verbose) printf("adaptationCondition: %s\n",str);
        }
        if (config_setting_lookup_string(setting, "modScheme", &str))
        {
            strcpy(ce->modScheme,str);
            if (verbose) printf("Modulation Scheme:%s\n",str);
        }
        if (config_setting_lookup_string(setting, "crcScheme", &str))
        {
            strcpy(ce->crcScheme,str);
            if (verbose) printf("CRC Scheme:%s\n",str);
        }
        if (config_setting_lookup_string(setting, "innerFEC", &str))
        {
            strcpy(ce->innerFEC,str);
            if (verbose) printf("Inner FEC Scheme:%s\n",str);
        }
        if (config_setting_lookup_string(setting, "outerFEC", &str))
        {
            strcpy(ce->outerFEC,str);
            if (verbose) printf("Outer FEC Scheme:%s\n",str);
        }

        // Read the integers
        if (config_setting_lookup_int(setting, "iterations", &tmpI))
        {
           //ce->iteration=tmpI;
           if (verbose) printf("Iterations: %d\n", tmpI);
        }
        if (config_setting_lookup_int(setting, "payloadLen", &tmpI))
        {
           ce->payloadLen=tmpI; 
           if (verbose) printf("PayloadLen: %d\n", tmpI);
        }
        if (config_setting_lookup_int(setting, "payloadLenIncrement", &tmpI))
        {
           ce->payloadLenIncrement=tmpI; 
           if (verbose) printf("PayloadLenIncrement: %d\n", tmpI);
        }
        if (config_setting_lookup_int(setting, "payloadLenMax", &tmpI))
        {
           ce->payloadLenMax=tmpI; 
           if (verbose) printf("PayloadLenMax: %d\n", tmpI);
        }
        if (config_setting_lookup_int(setting, "payloadLenMin", &tmpI))
        {
           ce->payloadLenMin=tmpI; 
           if (verbose) printf("PayloadLenMin: %d\n", tmpI);
        }
        if (config_setting_lookup_int(setting, "numSubcarriers", &tmpI))
        {
           ce->numSubcarriers=tmpI; 
           if (verbose) printf("Number of Subcarriers: %d\n", tmpI);
        }
        if (config_setting_lookup_int(setting, "CPLen", &tmpI))
        {
           ce->CPLen=tmpI; 
           if (verbose) printf("CPLen: %d\n", tmpI);
        }
        if (config_setting_lookup_int(setting, "taperLen", &tmpI))
        {
           ce->taperLen=tmpI; 
           if (verbose) printf("taperLen: %d\n", tmpI);
        }
        if (config_setting_lookup_int(setting, "delay_us", &tmpI))
        {
           ce->delay_us=tmpI; 
           if (verbose) printf("delay_us: %d\n", tmpI);
        }
        // Read the floats
        if (config_setting_lookup_float(setting, "default_tx_power", &tmpD))
        {
           ce->default_tx_power=tmpD; 
           if (verbose) printf("Default Tx Power: %f\n", tmpD);
        }
        if (config_setting_lookup_float(setting, "latestGoalValue", &tmpD))
        {
           ce->latestGoalValue=tmpD; 
           if (verbose) printf("Latest Goal Value: %f\n", tmpD);
        }
        if (config_setting_lookup_float(setting, "threshold", &tmpD))
        {
           ce->threshold=tmpD; 
           if (verbose) printf("Threshold: %f\n", tmpD);
        }
        if (config_setting_lookup_float(setting, "frequency", &tmpD))
        {
           ce->frequency=tmpD; 
           if (verbose) printf("frequency: %f\n", tmpD);
        }
        if (config_setting_lookup_float(setting, "txgain_dB", &tmpD))
        {
           ce->txgain_dB=tmpD; 
           if (verbose) printf("txgain_dB: %f\n", tmpD);
        }
        if (config_setting_lookup_float(setting, "bandwidth", &tmpD))
        {
           ce->bandwidth=tmpD; 
           if (verbose) printf("bandwidth: %f\n", tmpD);
        }
        if (config_setting_lookup_float(setting, "uhd_txgain_dB", &tmpD))
        {
           ce->uhd_txgain_dB=tmpD; 
           if (verbose) printf("uhd_txgain_dB: %f\n", tmpD);
        }
        if (config_setting_lookup_float(setting, "weighted_avg_payload_valid_threshold", &tmpD))
        {
           ce->weighted_avg_payload_valid_threshold=tmpD; 
           if (verbose) printf("weighted_avg_payload_valid_threshold: %f\n", tmpD);
        }
        if (config_setting_lookup_float(setting, "PER_threshold", &tmpD))
        {
           ce->PER_threshold=tmpD; 
           if (verbose) printf("PER_threshold: %f\n", tmpD);
        }
        if (config_setting_lookup_float(setting, "BER_threshold", &tmpD))
        {
           ce->BER_threshold=tmpD; 
           if (verbose) printf("BER_threshold: %f\n", tmpD);
        }
		if (config_setting_lookup_float(setting, "averaging", &tmpD))
        {
           ce->averaging=tmpD; 
           if (verbose) printf("Averaging: %f\n", tmpD);
        }
    }
    config_destroy(&cfg);
    return 1;
} // End readCEConfigFile()

int readScConfigFile(struct Scenario * sc, char *current_scenario_file, int verbose)
{
    config_t cfg;               // Returns all parameters in this structure 
    config_setting_t *setting;
    //const char * str;
    int tmpI;
    double tmpD;
    char scFileLocation[60];

    //printf("In readScConfigFile(): string current_scenario_file: \n%s\n", current_scenario_file);

    // Because the file is in the folder 'scconfigs'
    strcpy(scFileLocation, "scconfigs/");
    strcat(scFileLocation, current_scenario_file);
    //printf("In readScConfigFile(): string scFileLocation: \n%s\n", scFileLocation);

    // Initialization 
    config_init(&cfg);

    // Read the file. If there is an error, report it and exit. 
    if (!config_read_file(&cfg, scFileLocation))
    {
        fprintf(stderr, "%s:%d - %s\n", config_error_file(&cfg), config_error_line(&cfg), config_error_text(&cfg));
        config_destroy(&cfg);
        exit(EX_NOINPUT);
    }

    //// Get the configuration file name. 
    //if (config_lookup_string(&cfg, "filename", &str))
    //    if (verbose)
    //        printf("File Name: %s\n", str);
    //else
    //    printf("No 'filename' setting in configuration file.\n");

    // Read the parameter group.
    setting = config_lookup(&cfg, "params");
    if (setting != NULL)
    {
        // Read the integer
        if (config_setting_lookup_int(setting, "addAWGNBasebandTx", &tmpI))
        {
            sc->addAWGNBasebandTx=tmpI;
            if (verbose) printf("addAWGNBasebandTx: %d\n", tmpI);
        }
        //else
        //    printf("No AddNoise setting in configuration file.\n");
        // Read the integer
        if (config_setting_lookup_int(setting, "addAWGNBasebandRx", &tmpI))
        {
            sc->addAWGNBasebandRx=tmpI;
            if (verbose) printf("addAWGNBasebandRx: %d\n", tmpI);
        }
        // Read the double
        if (config_setting_lookup_float(setting, "noiseSNR", &tmpD))
        {
            sc->noiseSNR=(float) tmpD;
            if (verbose) printf("Noise SNR: %f\n", tmpD);
        }
        //else
        //    printf("No Noise SNR setting in configuration file.\n");
       
        // Read the double
        if (config_setting_lookup_float(setting, "noiseDPhi", &tmpD))
        {
            sc->noiseDPhi=(float) tmpD;
            if (verbose) printf("NoiseDPhi: %f\n", tmpD);
        }
        //else
        //    printf("No NoiseDPhi setting in configuration file.\n");

        // Read the integer
        /*
        if (config_setting_lookup_int(setting, "addInterference", &tmpI))
        {
            sc->addInterference=tmpI;
            if (verbose) printf("addInterference: %d\n", tmpI);
        }
        //else
        //    printf("No addInterference setting in configuration file.\n");
        */

        // Read the integer
        if (config_setting_lookup_int(setting, "addRicianFadingBasebandTx", &tmpI))
        {
            sc->addRicianFadingBasebandTx=tmpI;
            if (verbose) printf("addRicianFadingBasebandTx: %d\n", tmpI);
        }
        if (config_setting_lookup_int(setting, "addRicianFadingBasebandRx", &tmpI))
        {
            sc->addRicianFadingBasebandRx=tmpI;
            if (verbose) printf("addRicianFadingBasebandRx: %d\n", tmpI);
        }
        //else
        //    printf("No addRicianFadingBaseband setting in configuration file.\n");

        // Read the double
        if (config_setting_lookup_float(setting, "fadeK", &tmpD))
        {
            sc->fadeK=(float)tmpD;
            if (verbose) printf("fadeK: %f\n", tmpD);
        }
        //else
        //    printf("No fadeK setting in configuration file.\n");

        // Read the double
        if (config_setting_lookup_float(setting, "fadeFd", &tmpD))
        {
            sc->fadeFd=(float)tmpD;
            if (verbose) printf("fadeFd: %f\n", tmpD);
        }
        //else
        //    printf("No fadeFd setting in configuration file.\n");

        // Read the double
        if (config_setting_lookup_float(setting, "fadeDPhi", &tmpD))
        {
            sc->fadeDPhi=(float)tmpD;
            if (verbose) printf("fadeDPhi: %f\n", tmpD);
        }

		// Read the integer
		if (config_setting_lookup_int(setting, "addCWInterfererBasebandTx", &tmpI))
        {
            sc->addCWInterfererBasebandTx=(float)tmpI;
            if (verbose) printf("addCWIntefererBasebandTx: %d\n", tmpI);
        }
		// Read the integer
		if (config_setting_lookup_int(setting, "addCWInterfererBasebandRx", &tmpI))
        {
            sc->addCWInterfererBasebandRx=(float)tmpI;
            if (verbose) printf("addCWIntefererBasebandRx: %d\n", tmpI);
        }

		// Read the double
		if (config_setting_lookup_float(setting, "cw_pow", &tmpD))
        {
            sc->cw_pow=(float)tmpD;
            if (verbose) printf("cw_pow: %f\n", tmpD);
        }

		// Read the double
		if (config_setting_lookup_float(setting, "cw_freq", &tmpD))
        {
            sc->cw_freq=(float)tmpD;
            if (verbose) printf("cw_freq: %f\n", tmpD);
        }
        //else
        //    printf("No cw_freq setting in configuration file.\n");
    }

    config_destroy(&cfg);

    //printf("End of readScConfigFile() Function\n");
    return 1;
} // End readScConfigFile()


// Add AWGN
void enactAWGNBaseband(std::complex<float> * transmit_buffer, unsigned int buffer_len, struct CognitiveEngine ce, struct Scenario sc)
{
    //options
    float dphi  = sc.noiseDPhi;                              // carrier frequency offset
    float SNRdB = sc.noiseSNR;                               // signal-to-noise ratio [dB]
    //unsigned int symbol_len = ce.numSubcarriers + ce.CPLen;  // defining symbol length
    
    //printf("In enactAWGNBaseband: SNRdB=%f\n", SNRdB);

    // noise parameters
    float nstd = powf(10.0f, -SNRdB/20.0f); // noise standard deviation
    float phi = 0.0f;                       // channel phase
   
    std::complex<float> tmp (0, 1); 
    unsigned int i;

    // noise mixing
    //for (i=0; i<symbol_len; i++) {
    for (i=0; i<buffer_len; i++) {
        transmit_buffer[i] = std::exp(tmp*phi) * transmit_buffer[i]; // apply carrier offset
        //transmit_buffer[i] *= cexpf(_Complex_I*phi); // apply carrier offset
        phi += dphi;                                 // update carrier phase
        cawgn(&transmit_buffer[i], nstd);            // add noise
    }
} // End enactAWGNBaseband()

void enactCWInterfererBaseband(std::complex<float> * transmit_buffer, unsigned int buffer_len, struct CognitiveEngine ce, struct Scenario sc)
{
	float fs = ce.bandwidth; // Sample rate of the transmit buffer
	float k = pow(10.0, sc.cw_pow/20.0); // Coefficient to set the interferer power correctly
	//unsigned int symbol_len = ce.numSubcarriers + ce.CPLen;  // defining symbol length

	//printf("Coefficient: %f\n", k);

	for(unsigned int i=0; i<buffer_len; i++)
	{
		transmit_buffer[i] += k*sin(6.283*sc.cw_freq*i/fs); // Add CW tone
	}
} // End enactCWInterfererBaseband()

// Add Rice-K Fading
void enactRicianFadingBaseband(std::complex<float> * transmit_buffer, unsigned int buffer_len, struct CognitiveEngine ce, struct Scenario sc)
{
    // options
    //unsigned int symbol_len = ce.numSubcarriers + ce.CPLen; // defining symbol length

    unsigned int h_len;                                     // doppler filter length
    //if (symbol_len > 94){
    if (buffer_len > 94){
        //h_len = 0.0425*symbol_len;
        h_len = 0.0425*buffer_len;
    }
    else {
        h_len = 4;
    }
    float fd           = sc.fadeFd; // maximum doppler frequency
    float K            = sc.fadeK;  // Rice fading factor
    float omega        = 1.0f;      // mean power
    float theta        = 0.0f;      // angle of arrival
    float dphi = sc.fadeDPhi;       // carrier frequency offset
    float phi = 0.0f;               // channel phase

    // validate input
    if (K < 1.5f) {
        fprintf(stderr, "error: fading factor K must be greater than 1.5\n");
        exit(1);
    } else if (omega < 0.0f) {
        fprintf(stderr, "error: signal power Omega must be greater than zero\n");
        exit(1);
    } else if (fd <= 0.0f || fd >= 0.5f) {
        fprintf(stderr, "error: Doppler frequency must be in (0,0.5)\n");
        exit(1);
    //} else if (symbol_len== 0) {
    } else if (buffer_len== 0) {
        fprintf(stderr, "error: number of samples must be greater than zero\n");
        exit(1);
    }
 
    unsigned int i;

    // allocate array for output samples
    //std::complex<float> * y = (std::complex<float> *) malloc(symbol_len*sizeof(std::complex<float>));
    std::complex<float> * y = (std::complex<float> *) malloc(buffer_len*sizeof(std::complex<float>));
    // generate Doppler filter coefficients
    float h[h_len];
    liquid_firdes_doppler(h_len, fd, K, theta, h);

    // normalize filter coefficients such that output Gauss random
    // variables have unity variance
    float std = 0.0f;
    for (i=0; i<h_len; i++)
        std += h[i]*h[i];
    std = sqrtf(std);
    for (i=0; i<h_len; i++)
        h[i] /= std;

    // create Doppler filter from coefficients
    firfilt_crcf fdoppler = firfilt_crcf_create(h,h_len);

    // generate complex circular Gauss random variables
    std::complex<float> v;    // circular Gauss random variable (uncorrelated)
    std::complex<float> x;    // circular Gauss random variable (correlated w/ Doppler filter)
    float s   = sqrtf((omega*K)/(K+1.0));
    float sig = sqrtf(0.5f*omega/(K+1.0));
        
    std::complex<float> tmp(0, 1);
    //for (i=0; i<symbol_len; i++) {
    for (i=0; i<buffer_len; i++) {
        // generate complex Gauss random variable
        crandnf(&v);

        // push through Doppler filter
        firfilt_crcf_push(fdoppler, v);
        firfilt_crcf_execute(fdoppler, &x);

        // convert result to random variable with Rice-K distribution
        y[i] = tmp*( std::imag(x)*sig + s ) +
                          ( std::real(x)*sig     );
    }
    //for (i=0; i<symbol_len; i++) {
    for (i=0; i<buffer_len; i++) {
        transmit_buffer[i] *= std::exp(tmp*phi);  // apply carrier offset
        phi += dphi;                                  // update carrier phase
        transmit_buffer[i] *= y[i];                   // apply Rice-K distribution
    }

    // destroy filter object
    firfilt_crcf_destroy(fdoppler);

    // clean up allocated array
    free(y);
} // End enactRicianFadingBaseband()

//TODO: enable starting of this thread when a new scenario begins
// This function runs in its own thread waiting to modify the samples every time
// they are recieve by the ofdmtxrx object, but before they are sent to the
// synchronizer. 
void * enactScenarioBasebandRx( void * _arg)
{
    //printf("Scenario being added\n");
    enactScenarioBasebandRxStruct * esbrs = (enactScenarioBasebandRxStruct *) _arg;
    int count = 0;
    pthread_mutex_lock(&esbrs->txcvr_ptr->rx_buffer_mutex);
    //printf("In esbrs: esbrs ready\n");
    //pthread_cond_signal(&(esbrs->txcvr_ptr->esbrs_ready));
    while (true)
    { 
        //pthread_mutex_lock(esbrs->esbrs_ready_mutex_ptr);
	//*esbrs->esbrs_ready_ptr = 1;
	//pthread_mutex_unlock(esbrs->esbrs_ready_mutex_ptr);
	
        // Wait for txcvr rx_worker to signal samples are ready to be modified
        //printf("In esbrs: waiting for buffer to be filled %i\n", count);
	count++;
	pthread_cond_wait(&esbrs->txcvr_ptr->rx_buffer_filled_cond, &esbrs->txcvr_ptr->rx_buffer_mutex);

        // Add appropriate RF impairments for the scenario
        if (esbrs->sc_ptr->addRicianFadingBasebandRx == 1)
        {
            enactRicianFadingBaseband(esbrs->txcvr_ptr->rx_buffer->data(), esbrs->txcvr_ptr->rx_buffer->size(), *esbrs->ce_ptr, *esbrs->sc_ptr);
        }
        if (esbrs->sc_ptr->addCWInterfererBasebandRx == 1)
        {
            // Interference function
            enactCWInterfererBaseband(esbrs->txcvr_ptr->rx_buffer->data(), esbrs->txcvr_ptr->rx_buffer->size(), *esbrs->ce_ptr, *esbrs->sc_ptr);
        }
        if (esbrs->sc_ptr->addAWGNBasebandRx == 1)
        {
            enactAWGNBaseband(esbrs->txcvr_ptr->rx_buffer->data(), esbrs->txcvr_ptr->rx_buffer->size(), *esbrs->ce_ptr, *esbrs->sc_ptr);
        }
	
        // signal to txcvr rx_worker that samples are ready to be sent to synchronizer
        //printf("In esbrs: Buffer modified\n");
	pthread_cond_signal(&(esbrs->txcvr_ptr->rx_buffer_modified_cond));
        // unlock mutex
        //pthread_mutex_unlock(&(esbrs->txcvr_ptr->rx_buffer_mutex));

        //TODO implement killing of this thread when a scenario ends.
    }
    return NULL;
} // End enactScenarioBasebandRx()

// Enact Scenario
void enactScenarioBasebandTx(std::complex<float> * transmit_buffer, unsigned int buffer_len, struct CognitiveEngine ce, struct Scenario sc)
{
    // Add appropriate RF impairments for the scenario
    if (sc.addRicianFadingBasebandTx == 1)
    {
        enactRicianFadingBaseband(transmit_buffer, buffer_len, ce, sc);
    }
    if (sc.addCWInterfererBasebandTx == 1)
    {
        //fprintf(stderr, "WARNING: There is currently no interference scenario functionality!\n");
        // Interference function
        enactCWInterfererBaseband(transmit_buffer, buffer_len, ce, sc);
    }
    if (sc.addAWGNBasebandTx == 1)
    {
        enactAWGNBaseband(transmit_buffer, buffer_len, ce, sc);
    }
    if ( (sc.addAWGNBasebandTx == 0) && (sc.addCWInterfererBasebandTx == 0) && (sc.addRicianFadingBasebandTx == 0))
    {
       	fprintf(stderr, "WARNING: Nothing Added by Scenario!\n");
		//fprintf(stderr, "addCWInterfererBasebandTx: %i\n", sc.addCWInterfererBaseband);
    }
} // End enactScenarioBasebandTx()

void * call_uhd_siggen(void * param)
{

    return NULL;
} // end call_uhd_siggen()

/*
void enactUSRPScenario(struct CognitiveEngine ce, struct Scenario sc, pid_t* siggen_pid)
{
    // Check AWGN
    if (sc.addAWGNBaseband == 1){
       // Center freq of noise
       char freq_opt[40] = "-f";
       char * freq = NULL;
       sprintf(freq, "%f", ce.frequency);
       strcat(freq_opt, freq);

       // Type of output
       char output_opt[20] = "--gaussian";

       // Gain of Output
       char gain_opt[40] = "-g";
       char * noiseGain_dB = NULL;
       sprintf(noiseGain_dB, "%f", 10.0);
       strcat(gain_opt, noiseGain_dB);


       //pthread_create( siggenThread_ptr, NULL, call_uhd_siggen, NULL);
       *siggen_pid = fork();
       if ( *siggen_pid == -1 )
       {
           fprintf(stderr, "ERROR: Failed to fork child for uhd_siggen.\n" );
           _exit(EX_OSERR);
       }
       // If this is the child process
       if (*siggen_pid == 0)
       {
           // TODO: external call to uhd_siggen
           //system("/usr/bin/uhd_siggen_gui");
           execl("/usr/bin/uhd_siggen", "uhd_siggen", freq_opt, output_opt, gain_opt, (char*)NULL);
           perror("Error");
           // Then wait to be killed.
           while (1) {;}
       }
       // Give uhd_siggen time to initialize 
       sleep(8);

       //printf("WARNING: There is currently no USRP AWGN scenario functionality!\n");
       // FIXME: This is just test code. Remove when done.
           printf("siggen_pid= %d\n", *siggen_pid);
           kill(*siggen_pid, SIGKILL);
           //printf("ERROR: %s\n", strerror(errno));
           //perror("Error");
           while(1) {;}
    }
    if (sc.addInterference == 1){
       fprintf(stderr, "WARNING: There is currently no USRP interference scenario functionality!\n");
       // Interference function
    }
    if (sc.addRicianFadingBaseband == 1){
       //enactRicianFadingBaseband(transmit_buffer, ce, sc);
       fprintf(stderr, "WARNING: There is currently no USRP Fading scenario functionality!\n");
    }
    if ( (sc.addAWGNBaseband == 0) && (sc.addCWInterfererBaseband == 0) && (sc.addRicianFadingBaseband == 0) ){
       fprintf(stderr, "WARNING: Nothing Added by Scenario!\n");
    }
} // End enactUSRPScenario()
*/

modulation_scheme convertModScheme(char * modScheme, unsigned int * bps)
{
    modulation_scheme ms;
    // TODO: add other liquid-supported mod schemes
    if (strcmp(modScheme, "QPSK") == 0) {
        ms = LIQUID_MODEM_QPSK;
		*bps = 2;
    }
    else if ( strcmp(modScheme, "BPSK") ==0) {
        ms = LIQUID_MODEM_BPSK;
		*bps = 1;
    }
    else if ( strcmp(modScheme, "OOK") ==0) {
        ms = LIQUID_MODEM_OOK;
		*bps = 1;
    }
    else if ( strcmp(modScheme, "8PSK") ==0) {
        ms = LIQUID_MODEM_PSK8;
		*bps = 3;
    }
    else if ( strcmp(modScheme, "16PSK") ==0) {
        ms = LIQUID_MODEM_PSK16;
		*bps = 4;
    }
    else if ( strcmp(modScheme, "32PSK") ==0) {
        ms = LIQUID_MODEM_PSK32;
		*bps = 5;
    }
    else if ( strcmp(modScheme, "64PSK") ==0) {
        ms = LIQUID_MODEM_PSK64;
		*bps = 6;
    }
    else if ( strcmp(modScheme, "128PSK") ==0) {
        ms = LIQUID_MODEM_PSK128;
		*bps = 7;
    }
    else if ( strcmp(modScheme, "8QAM") ==0) {
        ms = LIQUID_MODEM_QAM8;
		*bps = 3;
    }
    else if ( strcmp(modScheme, "16QAM") ==0) {
        ms = LIQUID_MODEM_QAM16;
		*bps = 4;
    }
    else if ( strcmp(modScheme, "32QAM") ==0) {
        ms = LIQUID_MODEM_QAM32;
		*bps = 5;
    }
    else if ( strcmp(modScheme, "64QAM") ==0) {
        ms = LIQUID_MODEM_QAM64;
		*bps = 6;
    }
    else if ( strcmp(modScheme, "BASK") ==0) {
        ms = LIQUID_MODEM_ASK2;
		*bps = 1;
    }
    else if ( strcmp(modScheme, "4ASK") ==0) {
        ms = LIQUID_MODEM_ASK4;
		*bps = 2;
    }
    else if ( strcmp(modScheme, "8ASK") ==0) {
        ms = LIQUID_MODEM_ASK8;
		*bps = 3;
    }
    else if ( strcmp(modScheme, "16ASK") ==0) {
        ms = LIQUID_MODEM_ASK16;
		*bps = 4;
    }
    else if ( strcmp(modScheme, "32ASK") ==0) {
        ms = LIQUID_MODEM_ASK32;
		*bps = 5;
    }
    else if ( strcmp(modScheme, "64ASK") ==0) {
        ms = LIQUID_MODEM_ASK64;
		*bps = 6;
    }
    else if ( strcmp(modScheme, "128ASK") ==0) {
        ms = LIQUID_MODEM_ASK128;
		*bps = 7;
    }
    else {
        fprintf(stderr, "ERROR: Unknown Modulation Scheme");
        exit(EXIT_FAILURE);
        //TODO: Rather than halt execution,
        // Skip current test if given an unknown parameter.
    }

    return ms;
} // End convertModScheme()

crc_scheme convertCRCScheme(char * crcScheme, int verbose)
{
    crc_scheme check;
    if (strcmp(crcScheme, "none") == 0) {
        check = LIQUID_CRC_NONE;
        if (verbose) printf("check = LIQUID_CRC_NONE\n");
    }
    else if (strcmp(crcScheme, "checksum") == 0) {
        check = LIQUID_CRC_CHECKSUM;
        if (verbose) printf("check = LIQUID_CRC_CHECKSUM\n");
    }
    else if (strcmp(crcScheme, "8") == 0) {
        check = LIQUID_CRC_8;
        if (verbose) printf("check = LIQUID_CRC_8\n");
    }
    else if (strcmp(crcScheme, "16") == 0) {
        check = LIQUID_CRC_16;
        if (verbose) printf("check = LIQUID_CRC_16\n");
    }
    else if (strcmp(crcScheme, "24") == 0) {
        check = LIQUID_CRC_24;
        if (verbose) printf("check = LIQUID_CRC_24\n");
    }
    else if (strcmp(crcScheme, "32") == 0) {
        check = LIQUID_CRC_32;
        if (verbose) printf("check = LIQUID_CRC_32\n");
    }
    else {
        fprintf(stderr, "ERROR: unknown CRC\n");
        exit(EXIT_FAILURE);
        //TODO: Rather than halt execution,
        // Skip current test if given an unknown parameter.
    }

    return check;
} // End convertCRCScheme()

fec_scheme convertFECScheme(char * FEC, int verbose)
{
    // TODO: add other liquid-supported FEC schemes
    fec_scheme fec;
    if (strcmp(FEC, "none") == 0) {
        fec = LIQUID_FEC_NONE;
        if (verbose) printf("fec = LIQUID_FEC_NONE\n");
    }
    else if (strcmp(FEC, "Hamming74") == 0) {
        fec = LIQUID_FEC_HAMMING74;
        if (verbose) printf("fec = LIQUID_FEC_HAMMING74\n");
    }
    else if (strcmp(FEC, "Hamming128") == 0) {
        fec = LIQUID_FEC_HAMMING128;
        if (verbose) printf("fec = LIQUID_FEC_HAMMING128\n");
    }
    else if (strcmp(FEC, "Golay2412") == 0) {
        fec = LIQUID_FEC_GOLAY2412;
        if (verbose) printf("fec = LIQUID_FEC_GOLAY2412\n");
    }
    else if (strcmp(FEC, "SEC-DED2216") == 0) {
        fec = LIQUID_FEC_SECDED2216;
        if (verbose) printf("fec = LIQUID_FEC_SECDED2216\n");
    }
    else if (strcmp(FEC, "SEC-DED3932") == 0) {
        fec = LIQUID_FEC_SECDED3932;
        if (verbose) printf("fec = LIQUID_FEC_SECDED3932\n");
    }
    else if (strcmp(FEC, "SEC-DED7264") == 0) {
        fec = LIQUID_FEC_SECDED7264;
        if (verbose) printf("fec = LIQUID_FEC_SECDED7264\n");
    }
    else {
        fprintf(stderr, "ERROR: unknown FEC\n");
        exit(EXIT_FAILURE);
        //TODO: Rather than halt execution,
        // Skip current test if given an unknown parameter.
    }
    return fec;
} // End convertFECScheme()

// Create Frame generator with CE and Scenario parameters
ofdmflexframegen CreateFG(struct CognitiveEngine ce, struct Scenario sc, int verbose) {

    //printf("Setting inital ofdmflexframegen options:\n");
    // Set Modulation Scheme
    if (verbose) printf("Modulation scheme: %s\n", ce.modScheme);
    modulation_scheme ms = convertModScheme(ce.modScheme, &ce.bitsPerSym);

    // Set Cyclic Redundency Check Scheme
    crc_scheme check = convertCRCScheme(ce.crcScheme, verbose);

    // Set inner forward error correction scheme
    if (verbose) printf("Inner FEC: ");
    fec_scheme fec0 = convertFECScheme(ce.innerFEC, verbose);

    // Set outer forward error correction scheme
    // TODO: add other liquid-supported FEC schemes
    if (verbose) printf("Outer FEC: ");
    fec_scheme fec1 = convertFECScheme(ce.outerFEC, verbose);

    // Frame generation parameters
    ofdmflexframegenprops_s fgprops;

    // Initialize Frame generator and Frame Synchronizer Objects
    ofdmflexframegenprops_init_default(&fgprops);
    fgprops.mod_scheme      = ms;
    fgprops.check           = check;
    fgprops.fec0            = fec0;
    fgprops.fec1            = fec1;
    //printf("About to create fg...\n");
    ofdmflexframegen fg = ofdmflexframegen_create(ce.numSubcarriers, ce.CPLen, ce.taperLen, NULL, &fgprops);
    //printf("fg created\n");

    return fg;
} // End CreateFG()

int rxCallback(unsigned char *  _header,
               int              _header_valid,
               unsigned char *  _payload,
               unsigned int     _payload_len,
               int              _payload_valid,
               framesyncstats_s _stats,
               void *           _userdata)
{
    struct rxCBstruct * rxCBS_ptr = (struct rxCBstruct *) _userdata;
    int verbose = rxCBS_ptr->verbose;
	msequence rx_ms = *rxCBS_ptr->rx_ms_ptr; 

    // Variables for checking number of errors 
    unsigned int payloadByteErrors  =   0;
    unsigned int payloadBitErrors   =   0;
    int j,m;
	unsigned int tx_byte;

    // Calculate byte error rate and bit error rate for payload
    for (m=0; m<(signed int)_payload_len; m++)
    {
		tx_byte = msequence_generate_symbol(rx_ms,8);
		printf( "%1i %1i\n", (signed int)_payload[m], tx_byte );
        if (((int)_payload[m] != tx_byte))
        {
            payloadByteErrors++;
            for (j=0; j<8; j++)
            {
				if ((_payload[m]&(1<<j)) != (tx_byte&(1<<j)))
                   payloadBitErrors++;
            }      
        }           
    }               
                    
    // Data that will be sent to server
    // TODO: Send other useful data through feedback array
	//printf("CALLBACK!!!\n\n");
    struct feedbackStruct fb = {};
    fb.header_valid         =   _header_valid;
    fb.payload_valid        =   _payload_valid;
    fb.payload_len          =   _payload_len;
    fb.payloadByteErrors    =   payloadByteErrors;
    fb.payloadBitErrors     =   payloadBitErrors;
    fb.evm                  =   _stats.evm;
    fb.rssi                 =   _stats.rssi;
    fb.cfo                  =   _stats.cfo;	
	//fb.ce_num				=	_header[0];
	//fb.sc_num				=	_header[1];
	fb.iteration			=	0;
	fb.block_flag			=	0;

	for(int i=0; i<4; i++)	fb.iteration += _header[i+2]<<(8*(3-i));

    if (verbose)
    {
        printf("In rxcallback():\n");
        printf("Header: %i %i %i %i %i %i %i %i\n", _header[0], _header[1], 
            _header[2], _header[3], _header[4], _header[5], _header[6], _header[7]);
        feedbackStruct_print(&fb);
    }

    // Receiver sends data to server
    write(rxCBS_ptr->client, (void*)&fb, sizeof(fb));

    return 0;

} // end rxCallback()

ofdmflexframesync CreateFS(struct CognitiveEngine ce, struct Scenario sc, struct rxCBstruct* rxCBs_ptr)
{
     ofdmflexframesync fs =
             ofdmflexframesync_create(ce.numSubcarriers, ce.CPLen, ce.taperLen, NULL, rxCallback, (void *) rxCBs_ptr);

     return fs;
} // End CreateFS();

// Transmit a packet of data.
// This will need to be modified once we implement the USRPs.
//int txGeneratePacket(struct CognitiveEngine ce, ofdmflexframegen * _fg, unsigned char * header, unsigned char * payload)
//{
//    return 1;
//} // End txGeneratePacket()

//int txTransmitPacket(struct CognitiveEngine ce, ofdmflexframegen * _fg, std::complex<float> * frameSamples, 
//                        uhd::tx_metadata_t md, uhd::tx_streamer::sptr txStream, int usingUSRPs)
//{
//    int isLastSymbol = ofdmflexframegen_writesymbol(*_fg, frameSamples);
//
//    return isLastSymbol;
//} // End txTransmitPacket()

// TODO: Alter code for when usingUSRPs
//int rxReceivePacket(struct CognitiveEngine ce, ofdmflexframesync * _fs, std::complex<float> * frameSamples, int usingUSRPs)
//{
//    unsigned int symbolLen = ce.numSubcarriers + ce.CPLen;
//    ofdmflexframesync_execute(*_fs, frameSamples, symbolLen);
//    return 1;
//} // End rxReceivePacket()

void * serveTCPclient(void * _sc_ptr){
	struct serveClientStruct * sc_ptr = (struct serveClientStruct*) _sc_ptr;
	struct feedbackStruct read_buffer;
	struct feedbackStruct *fb_ptr = sc_ptr->fb_ptr;
	while(1){
        bzero(&read_buffer, sizeof(read_buffer));
        read(sc_ptr->client, &read_buffer, sizeof(read_buffer));
		if (read_buffer.evm && !fb_ptr->block_flag) {*fb_ptr = read_buffer; fb_ptr->block_flag = 1;}
    }
    return NULL;
}


//Reads the messages from a TCP link to a DSA congnitive radio
void * serveTCPDSAclient(void * _sc_ptr){
	//int latestprimary = 0;
	//int latestsecondary = 0;
	int number = 0;
	//int primarytest = 0;
	struct serveClientStruct * sc_ptr = (struct serveClientStruct*) _sc_ptr;
	int client = sc_ptr->client;
	struct message read_buffer;
	struct message *m_ptr = sc_ptr->m_ptr;
	while(1){
		//The main thread sets this to 0 after it has finished dealing with the last message
		if(m_ptr->msgreceived == 0){
		    bzero(&read_buffer, sizeof(read_buffer));
		    read(client, &read_buffer, sizeof(read_buffer));
			//Checks that the message received is a new messsage and that it has a proper type
			if(read_buffer.number > number and (read_buffer.type == 'p' or read_buffer.type == 's' or read_buffer.type == 'P' or read_buffer.type == 'S')){
				*m_ptr = read_buffer;
				//printf("%c %c %d\n", read_buffer.type, read_buffer.purpose, read_buffer.number);
				m_ptr->msgreceived = 1;
				number = read_buffer.number;
			}
		}
		//if (read_buffer && !fb_ptr->block_flag) {*fb_ptr->evm = read_buffer; fb_ptr->block_flag = 1;}
    }
    return NULL;
}

/*struct fftthreadinfo{
	struct CognitiveEngine ce;
	struct fftstruct fftinfo;
	float noisefloor;
	uhd::usrp::multi_usrp::sptr usrp;
	int primaryon;
}

void * fftthread(void * v_ptr){
	struct fftthreadinfo * fti_ptr = (struct fftthreadinfo*) v_ptr;
	while(1){
		fti_ptr->primaryon = fftscan(fti_ptr->ce, fti_ptr->usrp, fti_ptr->noisefloor, fti_ptr->fftinfo);
	}
}*/
	
	




//Thread that a broadcasting transmitter runs to interpret receiver feedback
void * feedbackThread(void * v_ptr){
	struct broadcastfeedbackinfo * bfi_ptr = (struct broadcastfeedbackinfo*) v_ptr;
	struct message * m_ptr = bfi_ptr->m_ptr;
	int client = bfi_ptr->client;
	struct message msg;
	msg.type = bfi_ptr->user;
	int clientlist[10];
	clientlist[0] = 0;
	int clientlistlength = 1;
	struct feedbackStruct basicfb;
	int fbnum = 1;

	//Zeroes out the feedback structures so they can be added to when
	//new feedback is received
	
	basicfb.header_valid = 0;
	basicfb.payload_valid = 0;
   	basicfb.payload_len = 0;
	basicfb.payloadByteErrors = 0;
   	basicfb.payloadBitErrors = 0;
	basicfb.iteration = 0;
   	basicfb.evm = 0.0;
	basicfb. rssi = 0.0;
	basicfb.cfo = 0.0;
	basicfb.block_flag = 0;
		
	
	int primary = 0;
	int secondary = 0;
	int loop = 1;
	int h;
	
	while(loop){
		//When a new message is received
		if(m_ptr->msgreceived == 1){
			//If the message is from a primary receiver
			if(m_ptr->type == 'p'){ 
				//Receiver saying that it received a primary transmission
				if(m_ptr->purpose == 'P'){

					primary++;
				}
				//receiver saying it received secondary transmission
				if(m_ptr->purpose == 'S'){

					secondary++;
				}
				if(m_ptr->purpose == 'u'){

					msg.purpose = 'u';
					msg.number = *bfi_ptr->msgnumber;
					write(client, (const void *)&msg, sizeof(msg));
					(*bfi_ptr->msgnumber)++;
				}
				//Feedback from primary transmission
				if(m_ptr->purpose == 'f'){
					primary++;
					//Checks if the message's client is in the client list
					//If it isn't then the transmitter hasn't received feedback from that node for that
					//transmission so it adds it to the list and adds its feedback to basicfb
					//If it is in the clientlist then the program assumes that the feedback received
					//is from a new transmission and that all feedback gathered before was from an older
					//one. The collected feedback is averaged and sent to the controller. Then basicfb
					//is zeroed out and the client list is emptied. The client is added to thel is and
					//its feedback is the first added to basicfb
					for(h = 0; h<clientlistlength; h++){
						if(clientlist[h] == m_ptr->client){
							break;
						}
					}
					
					if(h == clientlistlength){
						fbnum++;
						basicfb = feedbackadder(basicfb, m_ptr->feed);
						clientlist[clientlistlength] = m_ptr->client;
						clientlistlength++;
						}
					else{
						//printf("%d\n", client);
						basicfb.header_valid /= fbnum;
						basicfb.payload_valid /= fbnum;
					   	basicfb.payload_len /= fbnum;
						basicfb.payloadByteErrors /= fbnum;
					   	basicfb.payloadBitErrors /= fbnum;
						basicfb.iteration /= fbnum;
					   	basicfb.evm /= fbnum;
						basicfb. rssi /= fbnum;
						basicfb.cfo /= fbnum;
						basicfb.block_flag /= fbnum;
						msg.feed = basicfb;
						msg.purpose = 'f';
						msg.number = *bfi_ptr->msgnumber;
						//printf("Sending primary feedback\n");
						write(client, (const void *)&msg, sizeof(msg));
						(*bfi_ptr->msgnumber)++;
						basicfb.header_valid = 0;
						basicfb.payload_valid = 0;
					   	basicfb.payload_len = 0;
						basicfb.payloadByteErrors = 0;
					   	basicfb.payloadBitErrors = 0;
						basicfb.iteration = 0;
					   	basicfb.evm = 0.0;
						basicfb. rssi = 0.0;
						basicfb.cfo = 0.0;
						basicfb.block_flag = 0;	
						basicfb = feedbackadder(basicfb, m_ptr->feed);			
						fbnum = 1;
						clientlist[0] = m_ptr->client;
						clientlistlength = 1;
					}
						
				}
				//Receiver giving feedback from secondary transmission
				if(m_ptr->purpose == 'F'){;
					secondary++;
				}
			}
			if(m_ptr->type == 's'){
				
				//index = finder(clientlist, &clientlistlength, msg.client); 
				//Receiver saying that it received a primary transmission
				if(m_ptr->purpose == 'P'){
					bfi_ptr->primaryon = 1;
					primary++;
				}

				//Energy detector detects the primary user
				if(m_ptr->purpose == 'D'){
					bfi_ptr->energydetected = 1;
					primary++;
				}

				//Energy detector detects that the spectrum is free
				if(m_ptr->purpose == 'd'){
					bfi_ptr->energydetected = 0;
					primary++;
				}
				//receiver saying it received secondary transmission
				if(m_ptr->purpose == 'S'){
					secondary++;
				}

				//A frame with an unknown header was received
				//That means it was so distorted it was unknown if it was primary or secondary
				if(m_ptr->purpose == 'u'){
					msg.purpose = 'u';
					msg.number = *bfi_ptr->msgnumber;
					write(client, (const void *)&msg, sizeof(msg));
					(*bfi_ptr->msgnumber)++;
				}
				//Feedback from secondary transmission
				if(m_ptr->purpose == 'F'){
					secondary++;
					//Checks if the message's client is in the client list
					//If it isn't then the transmitter hasn't received feedback from that node for that
					//transmission so it adds it to the list and adds its feedback to basicfb
					//If it is in the clientlist then the program assumes that the feedback received
					//is from a new transmission and that all feedback gathered before was from an older
					//one. The collected feedback is averaged and sent to the controller. Then basicfb
					//is zeroed out and the client list is emptied. The client is added to thel is and
					//its feedback is the first added to basicfb
					for(h = 0; h<clientlistlength; h++){
						if(clientlist[h] == m_ptr->client){
							break;
						}
					}
					
					if(h == clientlistlength){
						fbnum++;
						basicfb = feedbackadder(basicfb, m_ptr->feed);
						clientlist[clientlistlength] = m_ptr->client;
						clientlistlength++;
						}
					else{
						basicfb.header_valid /= fbnum;
						basicfb.payload_valid /= fbnum;
					   	basicfb.payload_len /= fbnum;
						basicfb.payloadByteErrors /= fbnum;
					   	basicfb.payloadBitErrors /= fbnum;
						basicfb.iteration /= fbnum;
					   	basicfb.evm /= fbnum;
						basicfb. rssi /= fbnum;
						basicfb.cfo /= fbnum;
						basicfb.block_flag /= fbnum;
						msg.feed = basicfb;
						msg.purpose = 'f';
						msg.number = *bfi_ptr->msgnumber;
						write(client, (const void *)&msg, sizeof(msg));
						(*bfi_ptr->msgnumber)++;
						basicfb.header_valid = 0;
						basicfb.payload_valid = 0;
					   	basicfb.payload_len = 0;
						basicfb.payloadByteErrors = 0;
					   	basicfb.payloadBitErrors = 0;
						basicfb.iteration = 0;
					   	basicfb.evm = 0.0;
						basicfb. rssi = 0.0;
						basicfb.cfo = 0.0;
						basicfb.block_flag = 0;	
						basicfb = feedbackadder(basicfb, m_ptr->feed);			
						fbnum = 1;
						clientlist[0] = m_ptr->client;
						clientlistlength = 1;
					}
						
				}
				//Receiver giving feedback from primary transmission
				if(m_ptr->purpose == 'f'){
					bfi_ptr->primaryon = 1;
					primary++;
				}
			}
		m_ptr->msgreceived = 0;
		}
	};
	printf("Testing Complete\n");
	return NULL;
	
}
	


// Create a TCP socket for the server and bind it to a port
// Then sit and listen/accept all connections and write the data
// to an array that is accessible to the CE
void * startTCPServer(void * _ss_ptr)
{

    //printf("(Server thread called.)\n");

    struct serverThreadStruct * ss_ptr = (struct serverThreadStruct*) _ss_ptr;

    //  Local (server) address
    struct sockaddr_in servAddr;   
    // Parameters of client connection
    struct sockaddr_in clientAddr;              // Client address 
    socklen_t client_addr_size;  // Client address size
    int socket_to_client = -1;
    int reusePortOption = 1;

	pthread_t TCPServeClientThread[5]; // Threads for clients
	int client = 0; // Client counter
        
    // Create socket for incoming connections 
    int sock_listen;
    if ((sock_listen = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        fprintf(stderr, "Transmitter Failed to Create Server Socket.\n");
        exit(EXIT_FAILURE);
    }

    // Allow reuse of a port. See http://stackoverflow.com/questions/14388706/socket-options-so-reuseaddr-and-so-reuseport-how-do-they-differ-do-they-mean-t
    if (setsockopt(sock_listen, SOL_SOCKET, SO_REUSEPORT, (void*) &reusePortOption, sizeof(reusePortOption)) < 0 )
    {
        fprintf(stderr, " setsockopt() failed\n");
        exit(EXIT_FAILURE);
    }

    // Construct local (server) address structure 
    memset(&servAddr, 0, sizeof(servAddr));       // Zero out structure 
    servAddr.sin_family = AF_INET;                // Internet address family 
    servAddr.sin_addr.s_addr = htonl(INADDR_ANY); // Any incoming interface 
    servAddr.sin_port = htons(ss_ptr->serverPort);              // Local port 
    // Bind to the local address to a port
    if (bind(sock_listen, (struct sockaddr *) &servAddr, sizeof(servAddr)) < 0)
    {
        fprintf(stderr, "ERROR: bind() error\n");
        exit(EXIT_FAILURE);
    }

    // Listen and accept connections indefinitely
    while (1)
    {
        // listen for connections on socket
        if (listen(sock_listen, MAXPENDING) < 0)
        {
            fprintf(stderr, "ERROR: Failed to Set Sleeping (listening) Mode\n");
            exit(EXIT_FAILURE);
        }
        //printf("\n(Server is now in listening mode)\n");

        // Accept a connection from client
        //printf("Server is waiting to accept connection from client\n");
        socket_to_client = accept(sock_listen, (struct sockaddr *)&clientAddr, &client_addr_size);
        //printf("socket_to_client= %d\n", socket_to_client);
        if(socket_to_client< 0)
        {
            fprintf(stderr, "ERROR: Sever Failed to Connect to Client\n");
            exit(EXIT_FAILURE);
        }
		// Create separate thread for each client as they are accepted.
		else {
			struct serveClientStruct sc = CreateServeClientStruct();
			sc.client = socket_to_client;
			sc.fb_ptr = ss_ptr->fb_ptr;
			sc.m_ptr = ss_ptr->m_ptr;
			if(ss_ptr->type == 'd'){
			pthread_create( &TCPServeClientThread[client], NULL, serveTCPDSAclient, (void*) &sc);
			}
			else{
			pthread_create( &TCPServeClientThread[client], NULL, serveTCPclient, (void*) &sc);
			}

		}
        //printf("Server has accepted connection from client\n");
	}// End While loop
	
} // End startTCPServer()

int ceProcessData(struct CognitiveEngine * ce, struct feedbackStruct * fbPtr, int verbose)
{
    if (verbose)
    {
        printf("In ceProcessData():\n");
        feedbackStruct_print(fbPtr);
    }

    ce->validPayloads += fbPtr->payload_valid;

    if (fbPtr->payload_valid && (!(fbPtr->payloadBitErrors)))
    {
        ce->errorFreePayloads++;
        if (verbose) printf("Error Free payload!\n");
    }

    ce->PER = ((float)ce->frameNumber-(float)ce->errorFreePayloads)/((float)ce->frameNumber);
    ce->lastReceivedFrame = fbPtr->iteration;
    ce->BERLastPacket = ((float)fbPtr->payloadBitErrors)/((float)(ce->payloadLen*8));
    ce->weightedAvg += (float) fbPtr->payload_valid;

    //printf("ce->goal=%s\n", ce->goal);

    // Update goal value
    if (strcmp(ce->goal, "payload_valid") == 0)
    {
        if (verbose) printf("Goal is payload_valid. Setting latestGoalValue to %d\n", fbPtr->payload_valid);
        ce->latestGoalValue = fbPtr->payload_valid;
    }
    else if (strcmp(ce->goal, "X_valid_payloads") == 0)
    {
        if (verbose) printf("Goal is X_valid_payloads. Setting latestGoalValue to %u\n", ce->validPayloads);
        ce->latestGoalValue = (float) ce->validPayloads;
    }
    else if (strcmp(ce->goal, "X_errorFreePayloads") == 0)
    {
        if (verbose) printf("Goal is X_errorFreePayloads. Setting latestGoalValue to %u\n", ce->errorFreePayloads);
        ce->latestGoalValue = (float) ce->errorFreePayloads;
    }
    else if (strcmp(ce->goal, "X_frames") == 0)
    {
        if (verbose) printf("Goal is X_frames. Setting latestGoalValue to %u\n", ce->frameNumber);
        ce->latestGoalValue = (float) ce->frameNumber;
    }
    else if (strcmp(ce->goal, "X_seconds") == 0)
    {
if (verbose) printf("Goal is X_seconds. Setting latestGoalValue to %f\n", ce->runningTime);
        ce->latestGoalValue = ce->runningTime;
    }
    else
    {
        fprintf(stderr, "ERROR: Unknown Goal!\n");
        exit(EXIT_FAILURE);
    }
    // TODO: implement if statements for other possible goals

    return 1;
} // End ceProcessData()



int ceOptimized(struct CognitiveEngine * ce, int verbose)
{
	// Update running average
	ce->averagedGoalValue -= ce->metric_mem[ce->iteration%ce->averaging]/ce->averaging;
	ce->averagedGoalValue += ce->latestGoalValue/ce->averaging;
	ce->metric_mem[ce->iteration%ce->averaging] = ce->latestGoalValue;

	//printf("\nLatest: %.2f Average: %.2f Memory: %.2f\n\n", ce->latestGoalValue, ce->averagedGoalValue, ce->metric_mem[ce->iteration%ce->averaging]);
	
   	if(ce->frameNumber>ce->averaging){
		if (verbose) 
	   	{
		   printf("Checking if goal value has been reached.\n");
		   printf("ce.averagedGoalValue= %f\n", ce->averagedGoalValue);
		   printf("ce.threshold= %f\n", ce->threshold);
	   	}
	   	if (ce->latestGoalValue >= ce->threshold)
	   	{
		   if (verbose) printf("Goal is reached!\n");
		   return 1;
	   	}
	   	if (verbose) printf("Goal not reached yet.\n");
	}
   	return 0;
} // end ceOptimized()

int ceModifyTxParams(struct CognitiveEngine * ce, struct feedbackStruct * fbPtr, int verbose)
{
    int modify = 0;


    if (verbose) printf("ce->adaptationCondition= %s\n", ce->adaptationCondition);

    // Add 'user input' adaptation
    if(strcmp(ce->adaptationCondition, "user_specified") == 0) {
        // Check if parameters should be modified
        modify = 1;
        if (verbose) printf("user specified adaptation mode. Modifying...\n");
    }

    // Check what values determine if parameters should be modified
    if(strcmp(ce->adaptationCondition, "last_payload_invalid") == 0) {
        // Check if parameters should be modified
        if(fbPtr->payload_valid<1)
        {
            modify = 1;
            if (verbose) printf("lpi. Modifying...\n");
        }
    }
    if(strcmp(ce->adaptationCondition, "weighted_avg_payload_valid<X") == 0) {
        // Check if parameters should be modified
        if (ce->weightedAvg < ce->weighted_avg_payload_valid_threshold)
        {
            modify = 1;
            if (verbose) printf("wapv<X. Modifying...\n");
        }
    }
    if(strcmp(ce->adaptationCondition, "weighted_avg_payload_valid>X") == 0) {
        // Check if parameters should be modified
        if (ce->weightedAvg > ce->weighted_avg_payload_valid_threshold)
        {
            modify = 1;
            if (verbose) printf("wapv>X. Modifying...\n");
        }
    }
    if(strcmp(ce->adaptationCondition, "PER<X") == 0) {
        // Check if parameters should be modified
        if (verbose) printf("PER = %f\n", ce->PER);
        if(ce->PER < ce->PER_threshold)
        {
            modify = 1;
            if (verbose) printf("per<x. Modifying...\n" );
        }
    }
    if(strcmp(ce->adaptationCondition, "PER>X") == 0) {
        // Check if parameters should be modified
        if (verbose) printf("PER = %f\n", ce->PER);
        if(ce->PER > ce->PER_threshold)
        {
            modify = 1;
            if (verbose) printf("per>x. Modifying...\n" );
        }
    }
    if(strcmp(ce->adaptationCondition, "BER_lastPacket<X") == 0) {
        // Check if parameters should be modified
        if (verbose) printf("BER = %f\n", ce->BERLastPacket);
        if(ce->BERLastPacket < ce->BER_threshold)
        {
            modify = 1;
            if (verbose) printf("Ber_lastpacket<x. Modifying...\n" );
        }
    }
    if(strcmp(ce->adaptationCondition, "BER_lastPacket>X") == 0) {
        // Check if parameters should be modified
        if (verbose) printf("BER = %f\n", ce->BERLastPacket);
        if(ce->BERLastPacket > ce->BER_threshold)
        {
            modify = 1;
            if (verbose) printf("Ber_lastpacket>x. Modifying...\n" );
        }
    }
    if(strcmp(ce->adaptationCondition, "last_packet_error_free") == 0) {
        // Check if parameters should be modified
        if(!(fbPtr->payloadBitErrors)){
            modify = 1;
            if (verbose) printf("lpef. Modifying...\n");
        }
    }

    // If so, modify the specified parameter
    if (modify) 
    {
        if (verbose) printf("Modifying Tx parameters...\n");
        // TODO: Implement a similar if statement for each possible option
        // that can be adapted.

        // This can't work because rx can't detect signals with different 
        // number of subcarriers
        //if (strcmp(ce->adaptation, "decrease_numSubcarriers") == 0) {
        //    if (ce->numSubcarriers > 2)
        //        ce->numSubcarriers -= 2;
        //}

        if(strcmp(ce->adaptationCondition, "user_specified") == 0) {
            // Check if parameters should be modified
            if (verbose) printf("Reading user specified adaptations from user ce file: 'userEngine.txt'\n");
            readCEConfigFile(ce, (char*) "userEngine.txt", verbose);
        }

        if (strcmp(ce->adaptation, "increase_payload_len") == 0) {
            if (ce->payloadLen + ce->payloadLenIncrement <= ce->payloadLenMax) 
            {
                ce->payloadLen += ce->payloadLenIncrement;
            }
        }

        if (strcmp(ce->adaptation, "decrease_payload_len") == 0) {
            if (ce->payloadLen - ce->payloadLenIncrement >= ce->payloadLenMin) 
            {
                ce->payloadLen -= ce->payloadLenIncrement;
            }
        }

        if (strcmp(ce->adaptation, "decrease_mod_scheme_PSK") == 0) {
            if (strcmp(ce->modScheme, "QPSK") == 0) {
                strcpy(ce->modScheme, "BPSK");
            }
            if (strcmp(ce->modScheme, "8PSK") == 0) {
                strcpy(ce->modScheme, "QPSK");
            }
            if (strcmp(ce->modScheme, "16PSK") == 0) {
                strcpy(ce->modScheme, "8PSK");
            }
            if (strcmp(ce->modScheme, "32PSK") == 0) {
                strcpy(ce->modScheme, "16PSK");
            }
            if (strcmp(ce->modScheme, "64PSK") == 0) {
                strcpy(ce->modScheme, "32PSK");
            }
            if (strcmp(ce->modScheme, "128PSK") == 0) {
                strcpy(ce->modScheme, "64PSK");
                //printf("New modscheme: 64PSK\n");
            }
        }
        // Decrease ASK Modulations
        if (strcmp(ce->adaptation, "decrease_mod_scheme_ASK") == 0) {
            if (strcmp(ce->modScheme, "4ASK") == 0) {
                strcpy(ce->modScheme, "BASK");
            }
            if (strcmp(ce->modScheme, "8ASK") == 0) {
                strcpy(ce->modScheme, "4ASK");
            }
            if (strcmp(ce->modScheme, "16ASK") == 0) {
                strcpy(ce->modScheme, "8ASK");
            }
            if (strcmp(ce->modScheme, "32ASK") == 0) {
                strcpy(ce->modScheme, "16ASK");
	    }
            if (strcmp(ce->modScheme, "64ASK") == 0) {
                strcpy(ce->modScheme, "32ASK");
            }
            if (strcmp(ce->modScheme, "128ASK") == 0) {
                strcpy(ce->modScheme, "64ASK");
            }
        }
	// Turn outer FEC on/off
   	if (strcmp(ce->adaptation, "Outer FEC On/Off") == 0){
        if (verbose) printf("Adapt option: outer fec on/off. adapting...\n");
   	    // Turn FEC off
   	    if (ce->FECswitch == 1) {
   	        strcpy(ce->outerFEC_prev, ce->outerFEC);
   	        strcpy(ce->outerFEC, "none");
   	        ce->FECswitch = 0;
   	    }
   	    // Turn FEC on
   	    else {
   	        strcpy(ce->outerFEC, ce->outerFEC_prev);
   	        ce->FECswitch = 1;
   	    }
   	} 
        // Not use FEC
        if (strcmp(ce->adaptation, "no_fec") == 0) {
           if (strcmp(ce->outerFEC, "none") == 0) {
               strcpy(ce->outerFEC, "none");
           }
           if (strcmp(ce->outerFEC, "Hamming74") == 0) {
               strcpy(ce->outerFEC, "none");
           }
           if (strcmp(ce->outerFEC, "Hamming128") == 0) {
               strcpy(ce->outerFEC, "none");
           }
           if (strcmp(ce->outerFEC, "Golay2412") == 0) {
               strcpy(ce->outerFEC, "none");
           }
           if (strcmp(ce->outerFEC, "SEC-DED2216") == 0) {
               strcpy(ce->outerFEC, "none");
           }
           if (strcmp(ce->outerFEC, "SEC-DED3932") == 0) {
               strcpy(ce->outerFEC, "none");
           }
        }
        // FEC modifying (change to higher)
        if (strcmp(ce->adaptation, "increase_fec") == 0) {
           if (strcmp(ce->outerFEC, "SEC-DED3932") == 0) {
               strcpy(ce->outerFEC, "SEC-DED7264");
           } 
           if (strcmp(ce->outerFEC, "SEC-DED2216") == 0) {
               strcpy(ce->outerFEC, "SEC-DED3932");
           }
           if (strcmp(ce->outerFEC, "Golay2412") == 0) {
               strcpy(ce->outerFEC, "SEC-DED2216");
           }
           if (strcmp(ce->outerFEC, "Hamming128") == 0) {
               strcpy(ce->outerFEC, "Golay2412");
           }
           if (strcmp(ce->outerFEC, "Hamming74") == 0) {
               strcpy(ce->outerFEC, "Hamming128");
           }
           if (strcmp(ce->outerFEC, "none") == 0) {
               strcpy(ce->outerFEC, "Hamming74");
           }
        }
        // FEC modifying (change to lower)
        if (strcmp(ce->adaptation, "decrease_fec") == 0) {
           if (strcmp(ce->outerFEC, "Hamming74") == 0) {
               strcpy(ce->outerFEC, "none");
           }
           if (strcmp(ce->outerFEC, "Hamming128") == 0) {
               strcpy(ce->outerFEC, "Hamming74");
           }
           if (strcmp(ce->outerFEC, "Golay2412") == 0) {
               strcpy(ce->outerFEC, "Hamming128");
           }
           if (strcmp(ce->outerFEC, "SEC-DED2216") == 0) {
               strcpy(ce->outerFEC, "Golay2412");
           }
           if (strcmp(ce->outerFEC, "SEC-DED3932") == 0) {
               strcpy(ce->outerFEC, "SEC-DED2216");
           }
           if (strcmp(ce->outerFEC, "SEC-DED7264") == 0) {
               strcpy(ce->outerFEC, "SEC-DED3932");
           }
        }

        if (strcmp(ce->adaptation, "mod_scheme->BPSK") == 0) {
            strcpy(ce->modScheme, "BPSK");
        }
        if (strcmp(ce->adaptation, "mod_scheme->QPSK") == 0) {
            strcpy(ce->modScheme, "QPSK");
        }
        if (strcmp(ce->adaptation, "mod_scheme->8PSK") == 0) {
            strcpy(ce->modScheme, "8PSK");
        }
        if (strcmp(ce->adaptation, "mod_scheme->16PSK") == 0) {
            strcpy(ce->modScheme, "16PSK");
        }
        if (strcmp(ce->adaptation, "mod_scheme->328PSK") == 0) {
            strcpy(ce->modScheme, "32PSK");
        }
        if (strcmp(ce->adaptation, "mod_scheme->64PSK") == 0) {
            strcpy(ce->modScheme, "64PSK");
        }
        if (strcmp(ce->adaptation, "mod_scheme->8QAM") == 0) {
            strcpy(ce->modScheme, "8QAM");
        }
        if (strcmp(ce->adaptation, "mod_scheme->16QAM") == 0) {
            strcpy(ce->modScheme, "16QAM");
        }
        if (strcmp(ce->adaptation, "mod_scheme->32QAM") == 0) {
            strcpy(ce->modScheme, "32QAM");
        }
        if (strcmp(ce->adaptation, "mod_scheme->64QAM") == 0) {
            strcpy(ce->modScheme, "64QAM");
        }
        if (strcmp(ce->adaptation, "mod_scheme->OOK") == 0) {
            strcpy(ce->modScheme, "OOK");
        }
        if (strcmp(ce->adaptation, "mod_scheme->4ASK") == 0) {
            strcpy(ce->modScheme, "4ASK");
        }
        if (strcmp(ce->adaptation, "mod_scheme->8ASK") == 0) {
            strcpy(ce->modScheme, "8ASK");
        }
        if (strcmp(ce->adaptation, "mod_scheme->16ASK") == 0) {
            strcpy(ce->modScheme, "16ASK");
        }
        if (strcmp(ce->adaptation, "mod_scheme->32ASK") == 0) {
            strcpy(ce->modScheme, "32ASK");
        }
        if (strcmp(ce->adaptation, "mod_scheme->64ASK") == 0) {
            strcpy(ce->modScheme, "64ASK");
        }
    }
    return 1;
}   // End ceModifyTxParams()

//uhd::usrp::multi_usrp::sptr initializeUSRPs()
//{
//    //uhd::device_addr_t hint; //an empty hint discovers all devices
//    //uhd::device_addrs_t dev_addrs = uhd::device::find(hint);
//    //std::string str = dev_addrs[0].to_string();
//    //const char * c = str.c_str();
//    //printf("First UHD Device found: %s\n", c ); 
//
//    //std::string str2 = dev_addrs[1].to_string();
//    //const char * c2 = str2.c_str();
//    //printf("Second UHD Device found: %s\n", c2 ); 
//
//    uhd::device_addr_t dev_addr;
//    // TODO: Allow setting of USRP Address from command line
//    dev_addr["addr0"] = "type=usrp1,serial=8b9cadb0";
//    //uhd::usrp::multi_usrp::sptr usrp= uhd::usrp::multi_usrp::make(dev_addr);
//    //dev_addr["addr0"] = "8b9cadb0";
//    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(dev_addr);
//
//    //Lock mboard clocks
//    //usrp->set_clock_source(ref);
//
//    // Set the TX freq (in Hz)
//    usrp->set_tx_freq(450e6);
//    printf("TX Freq set to %f MHz\n", (usrp->get_tx_freq()/1e6));
//    // Wait for USRP to settle at the frequency
//    while (not usrp->get_tx_sensor("lo_locked").to_bool()){
//        usleep(1000);
//        //sleep for a short time 
//    }
//    //printf("USRP tuned and ready.\n");
// 
//    // Set the rf gain (dB)
//    // TODO: Allow setting of gain from command line
//    usrp->set_tx_gain(0.0);
//    printf("TX Gain set to %f dB\n", usrp->get_tx_gain());
// 
//    // Set the rx_rate (Samples/s)
//    // TODO: Allow setting of tx_rate from command line
//    usrp->set_tx_rate(1e6);
//    printf("TX rate set to %f MS/s\n", (usrp->get_tx_rate()/1e6));
//
//    // Set the IF BW (in Hz)
//    usrp->set_tx_bandwidth(500e3);
//    printf("TX bandwidth set to %f kHz\n", (usrp->get_tx_bandwidth()/1e3));
//
//    return usrp;
//} // end initializeUSRPs()


int postTxTasks(struct CognitiveEngine * cePtr, struct feedbackStruct * fb_ptr, int verbose)
{
    // FIXME: Find another way to fix this:: FIXED?
    usleep(cePtr->delay_us);
	std::clock_t timeout_start = std::clock();
	while( !fb_ptr->block_flag ){
		std::clock_t timeout_now = std::clock();
		if(double(timeout_now-timeout_start)/CLOCKS_PER_SEC>1.0e-3) break;
	}
	fb_ptr->block_flag = 0;

    int DoneTransmitting = 0;

    // Process data from rx
    ceProcessData(cePtr, fb_ptr, verbose);
    // Modify transmission parameters (in fg and in USRP) accordingly
    if (!ceOptimized(cePtr, verbose)) 
    {
        if (verbose) printf("ceOptimized() returned false\n");
        ceModifyTxParams(cePtr, fb_ptr, verbose);
    }
    else
    {
        if (verbose) printf("ceOptimized() returned true\n");
        DoneTransmitting = 1;
        //printf("else: DoneTransmitting= %d\n", DoneTransmitting);
    }

    // For debugging
    if (verbose)
    {
        printf("in postTxTasks(): \nce.numSubcarriers= %u\n", cePtr->numSubcarriers);
        printf("ce.CPLen= %u\n", cePtr->CPLen);
    }

    return DoneTransmitting;
} // End postTxTasks()

void updateScenarioSummary(struct scenarioSummaryInfo *sc_sum, struct feedbackStruct *fb, struct CognitiveEngine *ce, int i_CE, int i_Sc){
	sc_sum->valid_headers[i_CE][i_Sc] += fb->header_valid;
	sc_sum->valid_payloads[i_CE][i_Sc] += fb->payload_valid;
	sc_sum->EVM[i_CE][i_Sc] += fb->evm;
	sc_sum->RSSI[i_CE][i_Sc] += fb->rssi;
	sc_sum->total_bits[i_CE][i_Sc] += ce->payloadLen;
	sc_sum->bit_errors[i_CE][i_Sc] += fb->payloadBitErrors;
}

void updateCognitiveEngineSummaryInfo(struct cognitiveEngineSummaryInfo *ce_sum, struct scenarioSummaryInfo *sc_sum, struct CognitiveEngine *ce, int i_CE, int i_Sc){
	// Decrement frameNumber once
	ce->frameNumber--;
	// Store metrics for scenario
	sc_sum->total_frames[i_CE][i_Sc] = ce->frameNumber;
	sc_sum->EVM[i_CE][i_Sc] /= ce->frameNumber;
	sc_sum->RSSI[i_CE][i_Sc] /= ce->frameNumber;
	sc_sum->PER[i_CE][i_Sc] = ce->PER;

	// Display the scenario summary
	printf("Cognitive Engine %i Scenario %i Summary:\nTotal frames: %i\nPercent valid headers: %2f\nPercent valid payloads: %2f\nAverage EVM: %2f\n"
		"Average RSSI: %2f\nAverage BER: %2f\nAverage PER: %2f\n\n", i_CE+1, i_Sc+1, sc_sum->total_frames[i_CE][i_Sc],
		(float)sc_sum->valid_headers[i_CE][i_Sc]/(float)sc_sum->total_frames[i_CE][i_Sc], (float)sc_sum->valid_payloads[i_CE][i_Sc]/(float)sc_sum->total_frames[i_CE][i_Sc],
		sc_sum->EVM[i_CE][i_Sc], sc_sum->RSSI[i_CE][i_Sc], (float)sc_sum->bit_errors[i_CE][i_Sc]/(float)sc_sum->total_bits[i_CE][i_Sc], sc_sum->PER[i_CE][i_Sc]);

	// Store the sum of scenario metrics for the cognitive engine
	ce_sum->total_frames[i_CE] += sc_sum->total_frames[i_CE][i_Sc];
	ce_sum->valid_headers[i_CE] += sc_sum->valid_headers[i_CE][i_Sc];
	ce_sum->valid_payloads[i_CE] += sc_sum->valid_payloads[i_CE][i_Sc];
	ce_sum->EVM[i_CE] += sc_sum->EVM[i_CE][i_Sc];
	ce_sum->RSSI[i_CE] += sc_sum->RSSI[i_CE][i_Sc];
	ce_sum->total_bits[i_CE] += sc_sum->total_bits[i_CE][i_Sc];
	ce_sum->bit_errors[i_CE] += sc_sum->bit_errors[i_CE][i_Sc];
	ce_sum->PER[i_CE] += sc_sum->PER[i_CE][i_Sc];
}

void terminate(int sig){
	exit(1);
}

int dsaCallback(unsigned char * _header,
               int _header_valid,
               unsigned char * _payload,
               unsigned int _payload_len,
               int _payload_valid,
               framesyncstats_s _stats,
               void * _userdata)
{
	//printf(";lkfdsaj;\n");
    struct dsaCBstruct * dsaCBs_ptr = (struct dsaCBstruct *) _userdata;
//If the secondary transmitter is using energy detection then the callback is unnecessary and it is skipped
if(dsaCBs_ptr->usrptype == 'S' and (dsaCBs_ptr->detectiontype == 'e' or dsaCBs_ptr->detectiontype == 't'))
return 1;

//int primary;
int ones = 0;
int zeroes = 0;
int twos = 0;
char received;

//The different kinds of transmissions are determined by the content of the header
//and its number of 0's, 1's, and 2's
for(int i = 0; i<8; ++i){
if(_header[i]==1){
ones++;
}
if(_header[i]==0){
zeroes++;
}
if(_header[i]==2){
twos++;
}
}

//If the message has all 1's or all 2's then a primary transmission was received
//and the dsaCBs struct changes its variables to show this information
if(ones>zeroes || twos>zeroes){
//primary = 1;
if(dsaCBs_ptr->usrptype == 'S'){
dsaCBs_ptr->primaryon = 1;}
received = 'f';
//printf("\n\nPrimary transmission\n\n");
}

//If the message has all zeroes then it is a secondary transmission
if(zeroes>ones && zeroes>twos){
//primary = 0;
//dsaCBs_ptr->primaryon = 0;
dsaCBs_ptr->secondarysending = 1;
received = 'F';
//printf("\n\nSecondary transmission\n\n");
}
if(ones==0 and zeroes == 0){
struct message mess;
mess.type = dsaCBs_ptr->usrptype;
mess.purpose = 'u';
mess.number = dsaCBs_ptr->number;
mess.client = dsaCBs_ptr->client;
++dsaCBs_ptr->number;
write(dsaCBs_ptr->client, (void*)&mess, sizeof(mess));
}

    // Variables for checking number of errors
    unsigned int payloadByteErrors = 0;
    unsigned int payloadBitErrors = 0;
    int j,m;
unsigned int tx_byte;
if(received == 'f') tx_byte = 111;
if(received == 'F') tx_byte = 0;

    // Calculate byte error rate and bit error rate for payload
    for (m=0; m<(signed int)_payload_len; m++)
    {
//tx_byte = msequence_generate_symbol(rx_ms,8);
//printf( "%1i %1i\n", (signed int)_payload[m], tx_byte );
        if (((int)_payload[m] != tx_byte))
        {
            payloadByteErrors++;
            for (j=0; j<8; j++)
            {
if ((_payload[m]&(1<<j)) != (tx_byte&(1<<j)))
                   payloadBitErrors++;
            }
        }
    }
                    
    // Data that will be sent to server
    // TODO: Send other useful data through feedback array
//printf("CALLBACK!!!\n\n");
    struct feedbackStruct fb = {};
    fb.header_valid = _header_valid;
    fb.payload_valid = _payload_valid;
    fb.payload_len = _payload_len;
    fb.payloadByteErrors = payloadByteErrors;
    fb.payloadBitErrors = payloadBitErrors;
    fb.evm = _stats.evm;
    fb.rssi = _stats.rssi;
    fb.cfo = _stats.cfo;	
//fb.ce_num = _header[0];
//fb.sc_num = _header[1];
fb.iteration	=	0;
fb.block_flag	=	0;

for(int i=0; i<4; i++)	fb.iteration += _header[i+2]<<(8*(3-i));

    if (false)
    {
        printf("In dsacallback():\n");
        printf("Header: %i %i %i %i %i %i %i %i\n", _header[0], _header[1],
            _header[2], _header[3], _header[4], _header[5], _header[6], _header[7]);
        feedbackStruct_print(&fb);
    }

    // Receiver sends data to server*/
//struct feedbackStruct fb = {};
//If the usrp is a receiver then it will put the feedback into a message and send it to its transmitter
	if(dsaCBs_ptr->usrptype == 'p' or dsaCBs_ptr->usrptype == 's'){
		struct message mess;
		mess.type = dsaCBs_ptr->usrptype;
		mess.feed = fb;
		mess.purpose = received;
		mess.number = dsaCBs_ptr->number;
		mess.client = dsaCBs_ptr->client;
		++dsaCBs_ptr->number;
		//printf("%c %c %d\n", mess.type, mess.purpose, mess.number);
		write(dsaCBs_ptr->client, (void*)&mess, sizeof(mess));
	}

    return 0;

} // end rxCallback()



int fftscan(struct CognitiveEngine ce, uhd::usrp::multi_usrp::sptr usrp, float noisefloor, struct fftStruct fftinfo){
	int cantransmit;
    std::string args, file, ant, subdev, ref;
	ref = "internal";
    size_t total_num_samps = 0;
    size_t num_bins = fftinfo.numbins;
    double rate = fftinfo.rate;
	double freq = (double)ce.frequency;
	double gain = fftinfo.gain;
	double bw = fftinfo.bandwidth;
    std::string addr, port, mode;
	ant = fftinfo.antennae;

	usrp->set_clock_source(ref);
	usrp->set_rx_rate(rate);
	usrp->set_rx_freq(freq);
	usrp->set_rx_gain(gain);
	usrp->set_rx_bandwidth(bw);
	usrp->set_rx_antenna(ant);

    std::vector<std::string> sensor_names;
    sensor_names = usrp->get_rx_sensor_names(0);
    uhd::stream_args_t stream_args("fc32"); //complex floats
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);

     uhd::stream_cmd_t stream_cmd((total_num_samps == 0)?
     uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS:
     uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE
    );
    stream_cmd.num_samps = total_num_samps;
    stream_cmd.stream_now = true;
    stream_cmd.time_spec = uhd::time_spec_t();
    usrp->issue_stream_cmd(stream_cmd);
    uhd::rx_metadata_t md;
    std::vector<std::complex<float> > buff(num_bins);
    std::vector<std::complex<float> > out_buff(num_bins);
    std::vector<float> out_buff_norm(num_bins);
     std::vector<float> send_avmfft(num_bins);
    std::vector<std::complex<float> > send_cmpfft(num_bins);
    std::vector<std::complex<float> > send_tmsmps;
     
    //initialize fft plan
    fftwf_complex *in = (fftwf_complex*)&buff.front();
    fftwf_complex *out = (fftwf_complex*)&out_buff.front();
    fftwf_plan p;
    p = fftwf_plan_dft_1d(num_bins, in, out, FFTW_FORWARD, FFTW_ESTIMATE);
    int y;
	float totalpower = 0;
	for(y=0; y<fftinfo.repeat; ++y){    
        rx_stream->recv(
            &buff.front(), buff.size(), md, 3.0
        );        
        fftwf_execute(p);
		int x;
        for (unsigned int i=0; i<out_buff.size();i++)
             out_buff_norm[i]=sqrt(pow(abs(out_buff[i]),2));

		for(x=0; x<fftinfo.measuredbins; x++){
			totalpower+=out_buff_norm[x];
		}
	} 
	fftwf_destroy_plan(p);
	totalpower /= fftinfo.measuredbins;
	totalpower = totalpower/fftinfo.repeat;
	if(totalpower > noisefloor){
		cantransmit = 0;
	}
	else{
		cantransmit = 1;
	}
	if(fftinfo.debug==1){
	printf("Cantransmit = %d Measured Power = %f Threshold = %f\n", cantransmit, totalpower, noisefloor);}
	return cantransmit;
}



float noise_floor(struct CognitiveEngine ce, uhd::usrp::multi_usrp::sptr usrp, struct fftStruct fftinfo){
    std::string args, file, ant, subdev, ref;
	ref = "internal";
    size_t total_num_samps = 0;
    size_t num_bins = fftinfo.numbins;
    double rate = fftinfo.rate;
	double freq = ce.frequency;
	double gain = fftinfo.gain;
	double bw = fftinfo.bandwidth;
    std::string addr, port, mode;
	ant = fftinfo.antennae;
	usrp->set_clock_source(ref);
	usrp->set_rx_rate(rate);
	usrp->set_rx_freq(freq);
	usrp->set_rx_gain(gain);
	usrp->set_rx_bandwidth(bw);
	usrp->set_rx_antenna(ant);
    std::vector<std::string> sensor_names;
    sensor_names = usrp->get_rx_sensor_names(0);
    uhd::stream_args_t stream_args("fc32"); //complex floats
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);
     uhd::stream_cmd_t stream_cmd((total_num_samps == 0)?
     uhd::stream_cmd_t::STREAM_MODE_START_CONTINUOUS:
     uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE
    );
    stream_cmd.num_samps = total_num_samps;// total_num_samps=0 means coninuous mode 
    stream_cmd.stream_now = true;
    stream_cmd.time_spec = uhd::time_spec_t();
    usrp->issue_stream_cmd(stream_cmd);
    uhd::rx_metadata_t md;

    std::vector<std::complex<float> > buff(num_bins);
    std::vector<std::complex<float> > out_buff(num_bins);
    std::vector<float> out_buff_norm(num_bins);
   
     std::vector<float> send_avmfft(num_bins);
    std::vector<std::complex<float> > send_cmpfft(num_bins);
    std::vector<std::complex<float> > send_tmsmps;
     
    //initialize fft plan
    fftwf_complex *in = (fftwf_complex*)&buff.front();
    fftwf_complex *out = (fftwf_complex*)&out_buff.front();
    fftwf_plan p;
    p = fftwf_plan_dft_1d(num_bins, in, out, FFTW_FORWARD, FFTW_ESTIMATE);
    int y;
	float totalpower = 0;

	//For loop adds up central bins of multiple FFT runs and finds the average bin value
	for(y=0; y<fftinfo.noisefloorrepeat; ++y){    
        rx_stream->recv(
            &buff.front(), buff.size(), md, 3.0
        );        
        fftwf_execute(p);
		int x;
        for (unsigned int i=0; i<out_buff.size();i++)
             out_buff_norm[i]=sqrt(pow(abs(out_buff[i]),2));
		for(x=0; x<fftinfo.noisefloormeasuredbins; x++){
			totalpower+=out_buff_norm[x];
		}
	} 
	fftwf_destroy_plan(p);
	totalpower /= fftinfo.noisefloormeasuredbins;
	if(fftinfo.debug==1){
	printf("%f\n", (float)(totalpower/fftinfo.noisefloorrepeat + fftinfo.noiseadder));}
	return ((totalpower/fftinfo.noisefloorrepeat) * fftinfo.noisemult) + fftinfo.noiseadder;
}




int main(int argc, char ** argv){
    // Seed the PRNG
    srand(time(NULL));

    // TEMPORARY VARIABLE
    int usingUSRPs = 0;

	//If tester is 1 then the test if statement is run
	int tester = 0;

	//If noise is 1 then the node will be initialized as a noise producer, doing nothing but transmitting noise
	int noise = 0;
	
    int verbose = 1;
    int verbose_explicit = 0;
    int dataToStdout = 0;    

    // For experiments with CR Networks.
    // Specifies whether this crts instance is managing the experiment.
    int isController = 0;

    unsigned int serverPort =1400;
    char * serverAddr = (char*) "127.0.0.1";

    // Frame Synchronizer parameters
    unsigned int numSubcarriers = 64;
    unsigned int CPLen = 16;
    unsigned int taperLen = 4;
    float bandwidth = 1.0e6;
    float frequency = 460.0e6;
    float uhd_rxgain = 20.0;
	int dsa = 0;
	int broadcasting = 0;
	int secondary = 0;
	int primary = 0;
	int receiver = 0;
	int energy = 0;
	int normalcrts = 1;

    // Check Program options
    int d;
    while ((d = getopt(argc,argv,"IEQRDStBPNuhqvdrsp:ca:f:b:G:M:C:T:")) != EOF) {
        switch (d) {
        case 'u':
        case 'h':   usage();                           return 0;
        case 'q':   verbose = 0;                            break;
        case 'v':   verbose = 1; verbose_explicit = 1;      break;
        case 'd':   dataToStdout = 1; 
                    if (!verbose_explicit) verbose = 0;     break;
        case 'r':   usingUSRPs = 1;                         break;
        case 's':   usingUSRPs = 0;                         break;
        case 'p':   serverPort = atoi(optarg);              break;
        case 'c':   isController = 1;                       break;
        case 'a':   serverAddr = optarg;                    break;
        case 'f':   frequency = atof(optarg);               break;
        case 'b':   bandwidth = atof(optarg);               break;
        case 'G':   uhd_rxgain = atof(optarg);              break;
        case 'M':   numSubcarriers = atoi(optarg);          break;
        case 'C':   CPLen = atoi(optarg);                   break;
        case 'T':   taperLen = atoi(optarg);                break;
		case 'D':   dsa = 1; normalcrts = 0; break;
		case 'B':	broadcasting = 1; normalcrts = 0; break;
		case 'Q':   tester = 1; normalcrts = 0; break;
		//Designate the node as a primary user
		case 'P':	primary = 1; dsa = 1; normalcrts = 0; break;

		//Designate the node as a secondary user
		case 'S':	secondary = 1; dsa = 1; normalcrts = 0; break;

		//Designate the node as a receiver
		case 'R':	receiver = 1; normalcrts = 0; break;
		
		//Designate the node as an energy detector
		case 'E':	energy = 1; dsa = 1; normalcrts = 0; break;

		//Designate the node as an interference maker that transmits noise
		case 'I':	noise = 1; isController = 1; normalcrts = 0; break;
	
        //case 'p':   serverPort = atol(optarg);            break;
        //case 'f':   frequency = atof(optarg);           break;
        //case 'b':   bandwidth = atof(optarg);           break;
        //case 'G':   uhd_rxgain = atof(optarg);          break;
        //case 't':   num_seconds = atof(optarg);         break;
        default:
            verbose = 1;
        }   
    }   

    pthread_t TCPServerThread;   // Pointer to thread ID
	pthread_t enactScBbRxThread;   // Pointer to thread ID
    // Threading for using uhd_siggen (for when using USRPs)
    //pthread_t siggenThread;
    //int serverThreadReturn = 0;  // return value of creating TCPServer thread
    //pid_t uhd_siggen_pid;

    // Array that will be accessible to both Server and CE.
    // Server uses it to pass data to CE.
    struct feedbackStruct fb = {};
	fb.primaryon = 0;

    // For creating appropriate symbol length from 
    // number of subcarriers and CP Length
    unsigned int symbolLen;

    // Iterators
    int i_CE = 0;
    int i_Sc = 0;
    int DoneTransmitting = 0;
    int isLastSymbol = 0;
    char scenario_list [30][60];
    char cogengine_list [30][60];

    int NumCE=readCEMasterFile(cogengine_list, verbose);  
    int NumSc=readScMasterFile(scenario_list, verbose);  
    //printf ("\nCalled readScMasterFile function\n");

    // Cognitive engine struct used in each test
    struct CognitiveEngine ce = CreateCognitiveEngine();
    // Scenario struct used in each test
    struct Scenario sc = CreateScenario();

	//Message struct to pass info with TCP
	struct message msg;
	msg.msgreceived = 0;

    //printf("structs declared\n");
    // framegenerator object used in each test
    ofdmflexframegen fg;

    // framesynchronizer object used in each test
    ofdmflexframesync fs;

	// identical pseudo random sequence generators for tx and rx
	msequence tx_ms = msequence_create_default(9u);
	msequence rx_ms = msequence_create_default(9u);

    //printf("frame objects declared\n");

    // Buffers for packet/frame data
    unsigned char header[8];                       // Must always be 8 bytes for ofdmflexframe
    unsigned char payload[1000];                   // Large enough to accomodate any (reasonable) payload that
                                                   // the CE wants to use.

    // pointer for accessing header array when it has float values
    //unsigned int * header_u = (unsigned int *) header;

    std::complex<float> frameSamples[10000];      // Buffer of frame samples for each symbol.
                                                   // Large enough to accomodate any (reasonable) payload that 
                                                   // the CE wants to use.
    // USRP objects
    uhd::tx_metadata_t metaData;
    uhd::usrp::multi_usrp::sptr usrp;
    uhd::tx_streamer::sptr txStream;

	float throughput = 0;
	float total_symbols;
	float payload_symbols;

	// Metric Summary structs for each scenario and each cognitive engine
	struct scenarioSummaryInfo sc_sum;
	struct cognitiveEngineSummaryInfo ce_sum;
                                                   
    ////////////////////// End variable initializations.///////////////////////

	signal(SIGTERM, terminate);
	signal(SIGINT, terminate);
	signal(SIGQUIT, terminate);
	signal(SIGKILL, terminate);

    // Begin TCP Server Thread
    //serverThreadReturn = pthread_create( &TCPServerThread, NULL, startTCPServer, (void*) feedback);
    struct serverThreadStruct ss = CreateServerStruct();
    ss.serverPort = serverPort;
    ss.fb_ptr = &fb;
	ss.m_ptr = &msg;
	if(dsa==1){
		printf("dsa\n");
		ss.type = 'd';
	}
	else{
		ss.type = 'n';
	}
    if (isController or (broadcasting == 1 and usingUSRPs == 1)) 
        pthread_create( &TCPServerThread, NULL, startTCPServer, (void*) &ss);

    struct rxCBstruct rxCBs = CreaterxCBStruct();
    rxCBs.bandwidth = bandwidth;
    rxCBs.serverPort = serverPort;
    rxCBs.serverAddr = serverAddr;
    rxCBs.verbose = verbose;
	rxCBs.rx_ms_ptr = &rx_ms;

    // Allow server time to finish initialization
    usleep(0.1e6);

	//signal (SIGPIPE, SIG_IGN);
	int client;
	const int socket_to_server = socket(AF_INET, SOCK_STREAM, 0);
	if(!isController  || !usingUSRPs){
		// Create a client TCP socket] 
		if( socket_to_server < 0)
		{   
		    fprintf(stderr, "ERROR: Receiver Failed to Create Client Socket. \nerror: %s\n", strerror(errno));
		    exit(EXIT_FAILURE);
		}   
		//printf("Created client socket to server. socket_to_server: %d\n", socket_to_server);

		// Parameters for connecting to server
		struct sockaddr_in servAddr;
		memset(&servAddr, 0, sizeof(servAddr));
		servAddr.sin_family = AF_INET;
		servAddr.sin_port = htons(serverPort);
		servAddr.sin_addr.s_addr = inet_addr(serverAddr);

		// Attempt to connect client socket to server
		int connect_status;
		if((connect_status = connect(socket_to_server, (struct sockaddr*)&servAddr, sizeof(servAddr))))
		{   
		    fprintf(stderr, "Receiver Failed to Connect to server.\n");
		    fprintf(stderr, "connect_status = %d\n", connect_status);
		    exit(EXIT_FAILURE);
		}

		rxCBs.client = socket_to_server;
		client = rxCBs.client;
        if (verbose)
            printf("Connected to Server.\n");
	}
	if((usingUSRPs and isController) or (broadcasting == 1 and usingUSRPs == 1)){
		printf("\nPress any key once all nodes have connected to the TCP server\n");
		getchar();
	}

    // Get current date and time
	
    char dataFilename[50];
    time_t now = time(NULL);
    struct tm *t  = localtime(&now);
    strftime(dataFilename, sizeof(dataFilename)-1, "data/data_crts_%d%b%Y_%T", t);
    // TODO: Make sure data folder exists
    
    // Initialize Data File
    FILE * dataFile;
	if(isController){
    if (dataToStdout)
    {
        dataFile = stdout;
    }
    else
    {
        dataFile = fopen(dataFilename, "w");
    }}

    // Begin running tests

    // For each Cognitive Engine

	if(normalcrts){

   for (i_CE=0; i_CE<NumCE; i_CE++)
    {

		if (verbose) 
            printf("\nStarting Tests on Cognitive Engine %d\n", i_CE+1);
            
        if(isController){
		    // Initialize current CE
			ce = CreateCognitiveEngine();
			readCEConfigFile(&ce,cogengine_list[i_CE], verbose);

			// Send CE info to slave node(s)
			write(client, (void*)&ce, sizeof(ce));
		}
        
        // Run each CE through each scenario
        for (i_Sc= 0; i_Sc<NumSc; i_Sc++)
        {                	
				
            if (isController)
            {                   
        		if (verbose) printf("\n\nStarting Scenario %d\n", i_Sc+1);
                // Initialize current Scenario
                sc = CreateScenario();
                readScConfigFile(&sc,scenario_list[i_Sc], verbose);
                
                // Send Sc info to slave node(s)
                write(client, (void*)&sc, sizeof(sc));

                fprintf(dataFile, "Cognitive Engine %d\nScenario %d\n", i_CE+1, i_Sc+1);
                //All metrics
                /*fprintf(dataFile, "%-10s %-10s %-14s %-15s %-10s %-10s %-10s %-19s %-16s %-18s \n",
                    "linetype","frameNum","header_valid","payload_valid","evm (dB)","rssi (dB)","PER","payloadByteErrors","BER:LastPacket","payloadBitErrors");*/
                //Useful metrics
                fprintf(dataFile, "%-10s %-10s %-10s %-10s %-8s %-12s %-12s %-20s %-19s\n",
                    "linetype","frameNum","evm (dB)","rssi (dB)","PER","Packet BER", "Throughput", "Spectral Efficiency", "Averaged Goal Value");
                fflush(dataFile);
            }
            else
            // If this crts instance is not the controller,
            // then no CEs, adaptations, scenarios, etc.
            {
                ce.numSubcarriers = numSubcarriers;
                ce.CPLen = CPLen;
                ce.taperLen = taperLen;
                ce.frequency = frequency;
                ce.bandwidth = bandwidth;
            }

            // Initialize Receiver Defaults for current CE and Sc
            ce.frameNumber = 1;
            fs = CreateFS(ce, sc, &rxCBs);

            std::clock_t begin = std::clock();
            std::clock_t now;
            // Begin Testing Scenario
            DoneTransmitting = 0;

            // Pointer to ofdmtxrx object for when using USRPs
            // Needs to be outside if statement so object can be closed later.
            //ofdmtxrx * txcvr_ptr;

            //while(!DoneTransmitting)
            //{
                if (usingUSRPs) 
                {
                    //usrp = initializeUSRPs();    
                    // create transceiver object
                    unsigned char * p = NULL;   // default subcarrier allocation
                    if (verbose) 
                        printf("Using ofdmtxrx\n");                    
                    

		    //pthread_mutex_t esbrs_ready_mutex;
		    //int esbrs_ready = 0;
		    ofdmtxrx *txcvr_ptr = new ofdmtxrx(ce.numSubcarriers, ce.CPLen, ce.taperLen, p, rxCallback, (void*) &rxCBs, true);                    
                    //txcvr_ptr = &txcvr;

                    // Start the Scenario simulations from the scenario USRPs
                    //enactUSRPScenario(ce, sc, &uhd_siggen_pid);

                    // Each instance of this while loop transmits one packet
                    while(!DoneTransmitting)
                    {
                        // set properties
                        txcvr_ptr->set_tx_freq(ce.frequency);
                        txcvr_ptr->set_tx_rate(ce.bandwidth);
                        txcvr_ptr->set_tx_gain_soft(ce.txgain_dB);
                        txcvr_ptr->set_tx_gain_uhd(ce.uhd_txgain_dB);
                        //txcvr_ptr->set_tx_antenna("TX/RX");

                        if (!isController)
                        {

                            	txcvr_ptr->set_rx_freq(ce.frequency);
                            	txcvr_ptr->set_rx_rate(ce.bandwidth);
                            	txcvr_ptr->set_rx_gain_uhd(uhd_rxgain);

                            	if (verbose)
                            	{
                                	txcvr_ptr->debug_enable();
                                	printf("Set Rx freq to %f\n", ce.frequency);
                                	printf("Set Rx rate to %f\n", ce.bandwidth);
                                	printf("Set uhd Rx gain to %f\n", uhd_rxgain);
                            	}
                            
                            	// Structs for CE and Sc info
                            	struct CognitiveEngine ce_controller;
                            	struct Scenario sc_controller;
							
                            	int continue_running = 1;
				int rflag;
				char readbuffer[1000];
							
							
                            	//TODO get first ce and sc over tcp connection.
                            	// Start enactScenarioBasebandRx Thread
                            	//struct enactScenarioBasebandRxStruct esbrs = {.txcvr = &txcvr, .ce = &ce, .sc = &sc};
                            	// start enactScenarioBasebandRx thread
                            	//pthread_create( &enactScBbRxThread, NULL, enactScenarioBasebandRx, (void*) &esbrs);
                            
                            	// Receive CE info
                            	rflag = recv(socket_to_server, &readbuffer, sizeof(struct CognitiveEngine), 0);
                            	if(rflag == 0 || rflag == -1){
                            		printf("Error receiving CE info from the controller\n");
					close(socket_to_server);
					exit(1);
				}
			    	else ce_controller = *(struct CognitiveEngine*)readbuffer;
							
			    	// Receive Sc info
			    	rflag = recv(socket_to_server, &readbuffer, sizeof(struct Scenario), 0);
                            	if(rflag == 0 || rflag == -1){
                            		printf("Error receiving Scenario info from the controller\n");
					close(socket_to_server);
					exit(1);
     			    	}
			    	else sc_controller = *(struct Scenario*)readbuffer;
							
			    	// Initialize members of esbrs struct sent to enactScenarioBasebandRx()
			    	//struct enactScenarioBasebandRxStruct esbrs = {.txcvr_ptr = txcvr_ptr, .ce_ptr = &ce_controller, .sc_ptr = &sc_controller};
			    	struct enactScenarioBasebandRxStruct esbrs;
					esbrs.txcvr_ptr = txcvr_ptr;
					esbrs.ce_ptr = &ce_controller;
					esbrs.sc_ptr = &sc_controller;
				
			    	//pthread_mutex_init(&esbrs_ready_mutex, NULL);
				pthread_mutex_lock(&txcvr_ptr->rx_buffer_mutex);			
				pthread_create( &enactScBbRxThread, NULL, enactScenarioBasebandRx, (void*) &esbrs);
							
				// Wait until enactScenarioBasebandRx() has initialized
				//pthread_mutex_lock(&txcvr_ptr->rx_buffer_mutex);
				//while (!*esbrs.esbrs_ready_ptr) 
				//{
				//	pthread_mutex_unlock(esbrs.esbrs_ready_mutex_ptr);
				//	pthread_mutex_lock(esbrs.esbrs_ready_mutex_ptr);
				//}
				//pthread_mutex_unlock(esbrs.esbrs_ready_mutex_ptr);
				//pthread_cond_wait(&txcvr_ptr->esbrs_ready, &txcvr_ptr->rx_buffer_mutex);
				pthread_mutex_unlock(&txcvr_ptr->rx_buffer_mutex);			
				// Start liquid-usrp receiver
				printf("Starting receiver\n");
				txcvr_ptr->start_rx();
							
                            	while(continue_running)
                            	{
					// Wait until server provides more information, closes, or there is an error
					rflag = recv(socket_to_server, &readbuffer, sizeof(struct Scenario)+sizeof(struct CognitiveEngine), 0);
					if(rflag == 0 || rflag == -1){
						printf("Socket closed or failed\n");
						close(socket_to_server);
						msequence_destroy(rx_ms);
						exit(1);
					}
								
				//TODO:
                                // if new scenario:
                                //{
                                    // close enactScenarioBasebandRx Thread
                                    // close current ofdmtxrx object
                                    // update sc and ce
                                    // open new ofdmtxrx object
                                    // open new enactScenarioBasebandRx Thread
                                //}
                                	/*else if(rflag == sizeof(struct Scenario)){
                                		if(verbose) printf("Rewriting Scenario Info");
						pthread_cancel(enactScBbRxThread);
						delete txcvr_ptr;
						sc_controller = *(struct Scenario*)readbuffer;
						ofdmtxrx *txcvr_ptr = new ofdmtxrx(ce.numSubcarriers, ce.CPLen, ce.taperLen, p, rxCallback, (void*) &rxCBs, true);
						struct enactScenarioBasebandRxStruct esbrs = {.txcvr_ptr = txcvr_ptr, .ce_ptr = &ce_controller, .sc_ptr = &sc_controller};							
						pthread_create( &enactScBbRxThread, NULL, enactScenarioBasebandRx, (void*) &esbrs);
					}
					else if(rflag == sizeof(struct CognitiveEngine)){
						if(verbose) printf("Rewriting CE info");
						pthread_cancel(enactScBbRxThread);
						delete txcvr_ptr;
						ce_controller = *(struct CognitiveEngine*)readbuffer;
						ofdmtxrx *txcvr_ptr = new ofdmtxrx(ce.numSubcarriers, ce.CPLen, ce.taperLen, p, rxCallback, (void*) &rxCBs, true);
						struct enactScenarioBasebandRxStruct esbrs = {.txcvr_ptr = txcvr_ptr, .ce_ptr = &ce_controller, .sc_ptr = &sc_controller};							
						pthread_create( &enactScBbRxThread, NULL, enactScenarioBasebandRx, (void*) &esbrs);
					}*/
                                
                            	}
                        }

                        if (verbose) {
                            	txcvr_ptr->debug_enable();
                            	printf("Set frequency to %f\n", ce.frequency);
                            	printf("Set bandwidth to %f\n", ce.bandwidth);
                            	printf("Set txgain_dB to %f\n", ce.txgain_dB);
                            	printf("Set uhd_txgain_dB to %f\n", ce.uhd_txgain_dB);
                            	printf("Set Tx antenna to %s\n", "TX/RX");
                        }

                        int i = 0;
                        // Generate data
                        if (verbose) printf("\n\nGenerating data that will go in frame...\n");
				header[0] = i_CE+1;
				header[1] = i_Sc+1;
                        for (i=0; i<4; i++)
                            	header[i+2] = (ce.frameNumber & (0xFF<<(8*(3-i))))>>(8*(3-i));
				header[6] = 0;
				header[7] = 0;
                        for (i=0; i<(signed int)ce.payloadLen; i++)
                            	payload[i] = (unsigned char)msequence_generate_symbol(tx_ms,8);

                        // Include frame number in header information
                        if (verbose) printf("Frame Num: %u\n", ce.frameNumber);

                        // Set Modulation Scheme
                        if (verbose) printf("Modulation scheme: %s\n", ce.modScheme);
                        modulation_scheme ms = convertModScheme(ce.modScheme, &ce.bitsPerSym);

                        // Set Cyclic Redundency Check Scheme
                        //crc_scheme check = convertCRCScheme(ce.crcScheme);

                        // Set inner forward error correction scheme
                        if (verbose) printf("Inner FEC: ");
                        fec_scheme fec0 = convertFECScheme(ce.innerFEC, verbose);

                        // Set outer forward error correction scheme
                        if (verbose) printf("Outer FEC: ");
                        fec_scheme fec1 = convertFECScheme(ce.outerFEC, verbose);

                        // Replace with txcvr methods that allow access to samples:
                        txcvr_ptr->assemble_frame(header, payload, ce.payloadLen, ms, fec0, fec1);
                        int isLastSymbol = 0;
                        while(!isLastSymbol)
                        {
                            	isLastSymbol = txcvr_ptr->write_symbol();
                            	enactScenarioBasebandTx(txcvr_ptr->fgbuffer, txcvr_ptr->fgbuffer_len, ce, sc);
                            	txcvr_ptr->transmit_symbol();
                        }
                        txcvr_ptr->end_transmit_frame();

                        DoneTransmitting = postTxTasks(&ce, &fb, verbose);
                        // Record the feedback data received
                        //TODO: include fb.cfo

			// Compute throughput and spectral efficiency
			payload_symbols = (float)ce.payloadLen/(float)ce.bitsPerSym;
			total_symbols = (float)ofdmflexframegen_getframelen(txcvr_ptr->fg);
			throughput = (float)ce.bitsPerSym*ce.bandwidth*(payload_symbols/total_symbols);

                        //All metrics
                        /*fprintf(dataFile, "%-10s %-10u %-14i %-15i %-10.2f %-10.2f %-8.2f %-19u %-12.2f %-16u %-12.2f %-20.2f %-19.2f\n", 
				"crtsdata:", fb.frameNum, fb.header_valid, fb.payload_valid, fb.evm, fb.rssi, ce.PER, fb.payloadByteErrors,
				ce.BERLastPacket, fb.payloadBitErrors, throughput, throughput/ce.bandwidth, ce.averagedGoalValue);*/
			//Useful metrics
			fprintf(dataFile, "%-10s %-10i %-10.2f %-10.2f %-8.2f %-12.2f %-12.2f %-20.2f %-19.2f\n", 
				"crtsdata:", fb.iteration,  fb.evm, fb.rssi, ce.PER,
				ce.BERLastPacket, throughput, throughput/ce.bandwidth, ce.averagedGoalValue);
                        fflush(dataFile);

                        // Increment the frame counter
                        ce.frameNumber++;
			ce.iteration++;

                        // Update the clock
                        now = std::clock();
                        ce.runningTime = double(now-begin)/CLOCKS_PER_SEC;

			updateScenarioSummary(&sc_sum, &fb, &ce, i_CE, i_Sc);
                    } // End while not done transmitting loop
                    // TODO: close ofdmtxrx object
                }
                else // If not using USRPs
                {
                    while(!DoneTransmitting)
                    {
                        // Initialize Transmitter Defaults for current CE and Sc
                        fg = CreateFG(ce, sc, verbose);  // Create ofdmflexframegen object with given parameters
                        if (verbose) ofdmflexframegen_print(fg);

                        // Iterator
                        int i = 0;

                        // Generate data
                        if (verbose) printf("\n\nGenerating data that will go in frame...\n");
                        header[0] = i_CE+1;
						header[1] = i_Sc+1;
                        for (i=0; i<4; i++)
                            	header[i+2] = (ce.frameNumber & (0xFF<<(8*(3-i))))>>(8*(3-i));
				header[6] = 0;
				header[7] = 0;
                        for (i=0; i<(signed int)ce.payloadLen; i++)
                            payload[i] = (unsigned char)msequence_generate_symbol(tx_ms,8);

						// Called just to update bits per symbol field
			convertModScheme(ce.modScheme, &ce.bitsPerSym);

                        // Assemble frame
                        ofdmflexframegen_assemble(fg, header, payload, ce.payloadLen);
                        //printf("DoneTransmitting= %d\n", DoneTransmitting);
						
                        // i.e. Need to transmit each symbol in frame.
                        isLastSymbol = 0;

                        while (!isLastSymbol) 
                        {
                            //isLastSymbol = txTransmitPacket(ce, &fg, frameSamples, metaData, txStream, usingUSRPs);
                            isLastSymbol = ofdmflexframegen_writesymbol(fg, frameSamples);
                            symbolLen = ce.numSubcarriers + ce.CPLen;
                            //enactScenarioBasebandTx(frameSamples, symbolLen, ce, sc);
							
                            // Rx Receives packet
							ofdmflexframesync_execute(fs, frameSamples, symbolLen);
                        } // End Transmition For loop
						
                        DoneTransmitting = postTxTasks(&ce, &fb, verbose);

			fflush(dataFile);

			// Compute throughput and spectral efficiency
			payload_symbols = (float)ce.payloadLen/(float)ce.bitsPerSym;
			total_symbols = (float)ofdmflexframegen_getframelen(fg);
			throughput = (float)ce.bitsPerSym*ce.bandwidth*(payload_symbols/total_symbols);

                        //All metrics
                        /*fprintf(dataFile, "%-10s %-10u %-14i %-15i %-10.2f %-10.2f %-8.2f %-19u %-12.2f %-16u %-12.2f %-20.2f %-19.2f\n", 
			"crtsdata:", fb.frameNum, fb.header_valid, fb.payload_valid, fb.evm, fb.rssi, ce.PER, fb.payloadByteErrors,
			ce.BERLastPacket, fb.payloadBitErrors, throughput, throughput/ce.bandwidth, ce.averagedGoalValue);*/
			//Useful metrics
			fprintf(dataFile, "%-10s %-10i %-10.2f %-10.2f %-8.2f %-12.2f %-12.2f %-20.2f %-19.2f\n", 
				"crtsdata:", fb.iteration,  fb.evm, fb.rssi, ce.PER,
				ce.BERLastPacket, throughput, throughput/ce.bandwidth, ce.averagedGoalValue);

                        // Increment the frame counters and iteration counter
                        ce.frameNumber++;
			ce.iteration++;
                        // Update the clock
                        now = std::clock();
                        ce.runningTime = double(now-begin)/CLOCKS_PER_SEC;

			updateScenarioSummary(&sc_sum, &fb, &ce, i_CE, i_Sc);
                    } // End else While loop					
                }


            //} // End Test While loop
            clock_t end = clock();
            double time = (end-begin)/(double)CLOCKS_PER_SEC + ce.iteration*ce.delay_us/1.0e6;
            //fprintf(dataFile, "Elapsed Time: %f (s)", time);
		fprintf(dataFile, "Begin: %li End: %li Clock/s: %li Time: %f", begin, end, CLOCKS_PER_SEC, time);
            fflush(dataFile);

            // Reset the goal
            ce.latestGoalValue = 0.0;
            ce.errorFreePayloads = 0;
            if (verbose) printf("Scenario %i completed for CE %i.\n", i_Sc+1, i_CE+1);
            fprintf(dataFile, "\n\n");
            fflush(dataFile);

		updateCognitiveEngineSummaryInfo(&ce_sum, &sc_sum, &ce, i_CE, i_Sc);

		// Reset frame number
		ce.frameNumber = 0;
            
        } // End Scenario For loop

        if (verbose) printf("Tests on Cognitive Engine %i completed.\n", i_CE+1);

		// Divide the sum of each metric by the number of scenarios run to get the final metric
		ce_sum.EVM[i_CE] /= i_Sc;
		ce_sum.RSSI[i_CE] /= i_Sc;
		ce_sum.PER[i_CE] /= i_Sc;

		// Print cognitive engine summaries
		printf("Cognitive Engine %i Summary:\nTotal frames: %i\nPercent valid headers: %2f\nPercent valid payloads: %2f\nAverage EVM: %2f\n"
			"Average RSSI: %2f\nAverage BER: %2f\nAverage PER: %2f\n\n", i_CE+1, ce_sum.total_frames[i_CE], (float)ce_sum.valid_headers[i_CE]/(float)ce_sum.total_frames[i_CE],
			(float)ce_sum.valid_payloads[i_CE]/(float)ce_sum.total_frames[i_CE], ce_sum.EVM[i_CE], ce_sum.RSSI[i_CE], (float)ce_sum.bit_errors[i_CE]/(float)ce_sum.total_bits[i_CE], ce_sum.PER[i_CE]);

    } // End CE for loop

	// destroy objects
	msequence_destroy(tx_ms);
	msequence_destroy(rx_ms);
	close(socket_to_server);

	if(!usingUSRPs) close(socket_to_server);

    return 0;
}

//If DSA is used with USRPs and not a receiver either a primary transmitter is made or a
//secondary transmitter with sensing capabilities
if(dsa==1 && usingUSRPs && !isController){
	struct dsaCBstruct dsaCBs;
	dsaCBs.bandwidth = rxCBs.bandwidth;
    dsaCBs.serverPort = rxCBs.serverPort;
    dsaCBs.serverAddr = rxCBs.serverAddr;
    dsaCBs.verbose = rxCBs.verbose;
	dsaCBs.rx_ms_ptr = &rx_ms;
	dsaCBs.client = rxCBs.client;
	pthread_t receiverfeedbackThread;
	struct message mess;
	struct fftStruct fftinfo;
	int primarymsgnumber = 1;
	int secondarymsgnumber = 1;
	double primarybursttime;
	double primaryresttime;

	//Total number of primary user cycles
	int totaltime = 10;
	double secondaryscantime;
	int primaryburstrandom;
	int primaryrestrandom;
	int uninterruptedframes = 1;
	int adapt = 0;
	int usescenario = 0;
	
	struct CognitiveEngine puce;
	struct CognitiveEngine suce;
	struct Scenario sc = CreateScenario();
	verbose = 1;
	config_t cfg; // Returns all parameters in this structure
	config_setting_t *setting;
	const char * str; // Stores the value of the String Parameters in Config file
	int tmpI; // Stores the value of Integer Parameters from Config file
	double tmpD;
	char * str2;

	if (verbose)
		printf("Reading %s\n", "master_dsa_file.txt");

	//Initialization
	config_init(&cfg);

	// Read the file. If there is an error, report it and exit.
	if (!config_read_file(&cfg,"master_dsa_file.txt"))
	{
		fprintf(stderr, "\n%s:%d - %s", config_error_file(&cfg), config_error_line(&cfg), config_error_text(&cfg));
		config_destroy(&cfg);
		exit(EX_NOINPUT);
	}
	// Read the parameter group
	setting = config_lookup(&cfg, "params");
	if (setting != NULL)
	{
		// Read the strings
		if (config_setting_lookup_int(setting, "totalcycles", &tmpI))
		{
		totaltime = tmpI;
		}
		if (config_setting_lookup_string(setting, "detectiontype", &str))
		{
		str2 = (char *)str;
		dsaCBs.detectiontype = str[0];
		}
	}
	setting = config_lookup(&cfg, "fft");
	if (setting != NULL)
	{
		// Read the strings
		if (config_setting_lookup_float(setting, "bandwidth", &tmpD))
		{
		fftinfo.bandwidth = tmpD;
		}
		if (config_setting_lookup_float(setting, "channelbandwidth", &tmpD))
		{
		fftinfo.channelbandwidth = tmpD;
		}
		if (config_setting_lookup_int(setting, "rate", &tmpI))
		{
		fftinfo.rate = tmpI;
		}
		if (config_setting_lookup_int(setting, "numbins", &tmpI))
		{
		fftinfo.numbins = tmpI;
		}
		if (config_setting_lookup_int(setting, "repeat", &tmpI))
		{
		fftinfo.repeat = tmpI;
		}
		if (config_setting_lookup_int(setting, "noisefloorrepeat", &tmpI))
		{
		fftinfo.noisefloorrepeat = tmpI;
		}
		if (config_setting_lookup_int(setting, "noisefloortestnumber", &tmpI))
		{
		fftinfo.noisefloortestnumber = tmpI;
		}
		if (config_setting_lookup_int(setting, "noisefloormeasuredbins", &tmpI))
		{
		fftinfo.noisefloormeasuredbins = tmpI;
		}
		/*if (config_setting_lookup_int(setting, "numavrg", &tmpI))
		{
		fftinfo.numavrg = tmpI;
		}*/
		if (config_setting_lookup_int(setting, "measuredbins", &tmpI))
		{
		fftinfo.measuredbins = tmpI;
		}
		if (config_setting_lookup_int(setting, "testnumber", &tmpI))
		{
		fftinfo.testnumber = tmpI;
		}
		if (config_setting_lookup_string(setting, "antennae", &str))
		{
		str2 = (char *)str;
		fftinfo.antennae = str2;
		}
		if (config_setting_lookup_float(setting, "noiseadder", &tmpD))
		{
		fftinfo.noiseadder = tmpD;
		}
		if (config_setting_lookup_int (setting, "debug", &tmpI))
		{
		fftinfo.debug = tmpI;
		}
		if (config_setting_lookup_float (setting, "gain", &tmpD))
		{
		fftinfo.gain = tmpD;
		}
		if (config_setting_lookup_float (setting, "noisemult", &tmpD))
		{
		fftinfo.noisemult = tmpD;
		}
	}
	setting = config_lookup(&cfg, "PU");
	if (setting != NULL && primary == 1)
	{
		// Read the strings
		if (config_setting_lookup_float(setting, "bursttime", &tmpD))
		{
		primarybursttime = tmpD;
		printf("%f\n", primarybursttime);
		}
		if (config_setting_lookup_float(setting, "resttime", &tmpD))
		{
		primaryresttime = tmpD;
		}
		if (config_setting_lookup_int(setting, "burstrandom", &tmpI))
		{
		primaryburstrandom = tmpI;
		}
		if (config_setting_lookup_int(setting, "restrandom", &tmpI))
		{
		primaryrestrandom = tmpI;
		}
		if (config_setting_lookup_int(setting, "adapt", &tmpI))
		{
		adapt = tmpI;
		}
		if (config_setting_lookup_int(setting, "usescenario", &tmpI))
		{
		usescenario = tmpI;
		}
		if (config_setting_lookup_int(setting, "uninterruptedframes", &tmpI))
		{
		uninterruptedframes = tmpI;
		}
		if (config_setting_lookup_string(setting, "ce", &str))
		{
		str2 = (char *)str;
		puce = CreateCognitiveEngine();
		readCEConfigFile(&puce, str2, verbose);
		}
		if (config_setting_lookup_string(setting, "scenario", &str))
		{
		str2 = (char *)str;
		sc = CreateScenario();
		readScConfigFile(&sc, str2, verbose);
		}
	}
	setting = config_lookup(&cfg, "SU");
	if (setting != NULL && (secondary == 1 or energy==1))
	{
		if (config_setting_lookup_float(setting, "scantime", &tmpD))
		{
		secondaryscantime = tmpD;
		}
		if (config_setting_lookup_int(setting, "adapt", &tmpI))
		{
		adapt = tmpI;
		}
		if (config_setting_lookup_int(setting, "usescenario", &tmpI))
		{
		usescenario = tmpI;
		}
		if (config_setting_lookup_int(setting, "uninterruptedframes", &tmpI))
		{
		uninterruptedframes = tmpI;
		}
		if (config_setting_lookup_string(setting, "ce", &str))
		{
		str2 = (char *)str;
		suce = CreateCognitiveEngine();
		readCEConfigFile(&suce, str2, verbose);
		}
		if (config_setting_lookup_string(setting, "scenario", &str))
		{
		str2 = (char *)str;
		sc = CreateScenario();
		readScConfigFile(&sc, str2, verbose);
		}

	}
	//dsaCBs.detectiontype = 'e';
	//If it is a primary transmitter then the USRP ofdmtxrx object tranmists for its burst
	//time then rest for its rest time
	if(primary == 1 and !receiver){
		struct broadcastfeedbackinfo bfi;
		float timedivisor = 5.0;
		if(broadcasting==1){
		timedivisor = 1.0;
		bfi.user = 'P';
		bfi.client = dsaCBs.client;
		bfi.m_ptr = &msg;
		bfi.msgnumber = &primarymsgnumber;
		pthread_create( &receiverfeedbackThread, NULL, feedbackThread, (void*) &bfi);
		}
		mess.type = 'P';
		dsaCBs.usrptype = 'P';
		verbose = 0;
		printf("primary\n");
		int h;

		//The primary user transmits a frame of all 1's for easy identification
		for(h = 0; h<8; h++){
			header[h] = 1;
		};
		for(h = 0; h<(int)puce.payloadLen; h++){
			payload[h] = 111;
		};

		std::clock_t start;
		std::clock_t current;
		unsigned char * p = NULL;   // default subcarrier allocation
		if (verbose) 
		printf("Using ofdmtxrx\n");
		printf("%d %d %d\n", puce.numSubcarriers, puce.CPLen, puce.taperLen);
		ofdmtxrx txcvr(puce.numSubcarriers, puce.CPLen, puce.taperLen, p, rxCallback, (void*) &dsaCBs);
		txcvr.set_tx_freq(puce.frequency);
		txcvr.set_tx_rate(puce.bandwidth);
		txcvr.set_tx_gain_soft(puce.txgain_dB);
		txcvr.set_tx_gain_uhd(puce.uhd_txgain_dB);
        if (verbose) printf("Modulation scheme: %s\n", puce.modScheme);
        modulation_scheme ms = convertModScheme(puce.modScheme, &ce.bitsPerSym);

        // Set Cyclic Redundency Check Scheme
        //crc_scheme check = convertCRCScheme(ce.crcScheme);

        // Set inner forward error correction scheme
        if (verbose) printf("Inner FEC: ");
        fec_scheme fec0 = convertFECScheme(puce.innerFEC, verbose);

        // Set outer forward error correction scheme
        if (verbose) printf("Outer FEC: ");
        fec_scheme fec1 = convertFECScheme(puce.outerFEC, verbose);
		//int on = 1;
		std::clock_t time = 0;
		start = std::clock();
		double primarybasebursttime = primarybursttime;
		double primarybaseresttime = primaryresttime;
		for(int o = 0; o<totaltime; ++o){;
			//Randomizes primary burst and rest times
			primarybursttime = primarybasebursttime + rand() % primaryburstrandom;
			primaryresttime = primarybaseresttime + rand() % primaryrestrandom;
			time = 0;
			start = std::clock();
			printf("Cycle %d\n", o+1);
			printf("PU transmitting\n");
			mess.number = primarymsgnumber;
			mess.purpose = 't';
			write(dsaCBs.client, (const void*)&mess, sizeof(mess));
			primarymsgnumber++;
			//For some reason time is about 5 times slower in this while loop
			while(primarybursttime/timedivisor > time){
				current = std::clock();
				time = ((float)(current-start))/CLOCKS_PER_SEC;
				txcvr.assemble_frame(header, payload, puce.payloadLen, ms, fec0, fec1);
				int isLastSymbol = 0;
				while(!isLastSymbol)
					{
					isLastSymbol = txcvr.write_symbol();
					if(usescenario){
					enactScenarioBasebandTx(txcvr.fgbuffer, txcvr.fgbuffer_len, puce, sc);}
					txcvr.transmit_symbol();
					}
				usleep(100);
		   		txcvr.end_transmit_frame();
				usleep(100);
				//The radio adapts if it can
				if(adapt==1)
				postTxTasks(&puce, &msg.feed, verbose);
				current = std::clock();
				time = ((float)(current-start))/CLOCKS_PER_SEC;
			}
			time = 0;
			start = std::clock();
			printf("PU resting\n");
			mess.number = primarymsgnumber;
			mess.purpose = 'r';
			write(dsaCBs.client, (const void*)&mess, sizeof(mess));
			primarymsgnumber++;
			while(primaryresttime>time){
				current = std::clock();
				time = (current-start)/CLOCKS_PER_SEC;
			}
		}
		//After it has completed its cycles the primary transmitter sends a finished message to the controller
		mess.purpose = 'F';
		mess.number = primarymsgnumber;
		write(dsaCBs.client, (const void*)&mess, sizeof(mess));
		return 1;
	}

	//If it is a secondary user then the node acts as a secondary transmitter
	//Either sensing for the primary user or transmitting with small pauses for sening
	if(secondary == 1 && dsaCBs.detectiontype == 'm' && !receiver){
		struct broadcastfeedbackinfo bfi;
		mess.type = 'S';
		dsaCBs.usrptype = 'S';
		mess.msgreceived = 1;
		verbose = 0;
		if(broadcasting==1){
		bfi.user = 'S';
		bfi.client = dsaCBs.client;
		bfi.m_ptr = &msg;
		bfi.msgnumber = &secondarymsgnumber;
		pthread_create( &receiverfeedbackThread, NULL, feedbackThread, (void*) &bfi);
		}
		printf("secondary\n");

		//The secondary user has a payload of all zeroes
		for(int h = 0; h<8; h++){
			header[h] = 0;
		};
		for(unsigned int h = 0; h<suce.payloadLen; h++){
			payload[h] = 0;
		};
		std::clock_t start;
		std::clock_t current;
		unsigned char * p = NULL;   // default subcarrier allocation
		if (verbose) 
		printf("Using ofdmtxrx\n");
		printf("%d %d %d\n", suce.numSubcarriers, suce.CPLen, suce.taperLen);

		//Sets up transceiver object
		ofdmtxrx txcvr(suce.numSubcarriers, suce.CPLen, suce.taperLen, p, dsaCallback, (void*) &dsaCBs);
		txcvr.set_tx_freq(suce.frequency);
		txcvr.set_tx_rate(suce.bandwidth);
		txcvr.set_tx_gain_soft(suce.txgain_dB);
		txcvr.set_tx_gain_uhd(suce.uhd_txgain_dB);
    	txcvr.set_rx_freq(frequency);
   		txcvr.set_rx_rate(suce.bandwidth);
    	txcvr.set_rx_gain_uhd(uhd_rxgain);
		txcvr.start_rx();
		float time = 0;
		start = std::clock();
		while(true)
			{
			time = 0;
			start = std::clock();
			printf("SU transmitting\n");
			mess.number = secondarymsgnumber;
			mess.purpose = 't';
			write(dsaCBs.client, (const void*)&mess, sizeof(mess));
			secondarymsgnumber++;
			int z;
			//If it does not sense the primary user then the secondary user will transmit
			while(dsaCBs.primaryon==0)
				{
				for(z=0; z<uninterruptedframes; z++){
					modulation_scheme ms = convertModScheme(suce.modScheme, &suce.bitsPerSym);
					fec_scheme fec0 = convertFECScheme(suce.innerFEC, verbose);
					fec_scheme fec1 = convertFECScheme(suce.outerFEC, verbose);
					usleep(1);
					txcvr.assemble_frame(header, payload, suce.payloadLen, ms, fec0, fec1);
					int isLastSymbol = 0;
					while(!isLastSymbol)
						{
						usleep(1);
						isLastSymbol = txcvr.write_symbol();
						if(usescenario)
						enactScenarioBasebandTx(txcvr.fgbuffer, txcvr.fgbuffer_len, suce, sc);
						txcvr.transmit_symbol();
						}
			   		txcvr.end_transmit_frame();
					usleep(100);
					if(adapt==1)
					postTxTasks(&suce, &msg.feed, verbose);
				}
				time = 0.0;
				txcvr.start_rx();
				start = std::clock();

				//The secondary user will wait in this while loop and wait and see if any
				//primary users appear
				mess.number = secondarymsgnumber;
				mess.purpose = 'S';
				write(dsaCBs.client, (const void*)&mess, sizeof(mess));
				secondarymsgnumber++;
				while(0.5 > (float)time)
					{
					current = std::clock();
					time = ((float)(current-start))/CLOCKS_PER_SEC;
					}
				mess.number = secondarymsgnumber;
				mess.purpose = 's';
				write(dsaCBs.client, (const void*)&mess, sizeof(mess));
				secondarymsgnumber++;
				}
			time = 0;
			start = std::clock();
			printf("SU sensing\n");

			//Once the primary user is detected the secondary user stops transmitting
			//and switches to sensing in a new while loop
			mess.number = secondarymsgnumber;
			mess.purpose = 'r';
			write(dsaCBs.client, (const void*)&mess, sizeof(mess));
			secondarymsgnumber++;
			while(dsaCBs.primaryon==1)
				{
				time = 0;
				dsaCBs.primaryon = 0;
				start = std::clock();
				std::clock_t current;

				//The while loop sets primaryon to 0 in the beginning. If the loop
				//finishes without a new primary transmission switching it to 1 then
				//the secondary user will assume it has stopped and resume transmitting
				//This while loop below will run for secondaryscantime seconds
				while(secondaryscantime > time)
					{
					current = std::clock();
					time = (current-start)/CLOCKS_PER_SEC;
					}			
				}
			}
		}
	if(secondary == 1 && dsaCBs.detectiontype == 'r' && !receiver){
		struct broadcastfeedbackinfo bfi;
		mess.type = 'S';
		dsaCBs.usrptype = 'S';
		mess.msgreceived = 1;
		verbose = 0;
		if(broadcasting==1){
		bfi.primaryon = 0;
		bfi.user = 'S';
		bfi.client = dsaCBs.client;
		bfi.m_ptr = &msg;
		bfi.msgnumber = &secondarymsgnumber;
		pthread_create( &receiverfeedbackThread, NULL, feedbackThread, (void*) &bfi);
		}
		printf("secondary\n");

		//The secondary user has a payload of all zeroes
		for(int h = 0; h<8; h++){
			header[h] = 0;
		};
		for(unsigned int h = 0; h<suce.payloadLen; h++){
			payload[h] = 0;
		};
		std::clock_t start;
		//std::clock_t current;
		unsigned char * p = NULL;   // default subcarrier allocation
		if (verbose) 
		printf("Using ofdmtxrx\n");
		printf("%d %d %d\n", suce.numSubcarriers, suce.CPLen, suce.taperLen);

		//Sets up transceiver object
		ofdmtxrx txcvr(suce.numSubcarriers, suce.CPLen, suce.taperLen, p, dsaCallback, (void*) &dsaCBs);
		txcvr.set_tx_freq(suce.frequency);
		txcvr.set_tx_rate(suce.bandwidth);
		txcvr.set_tx_gain_soft(suce.txgain_dB);
		txcvr.set_tx_gain_uhd(suce.uhd_txgain_dB);
    	txcvr.set_rx_freq(frequency);
   		txcvr.set_rx_rate(suce.bandwidth);
    	txcvr.set_rx_gain_uhd(uhd_rxgain);
		//txcvr.start_rx();
		float time = 0;	
		start = std::clock();
		while(true)
			{
			time = 0;
			start = std::clock();
			printf("SU transmitting\n");
			mess.number = secondarymsgnumber;
			mess.purpose = 't';
			write(dsaCBs.client, (const void*)&mess, sizeof(mess));
			secondarymsgnumber++;
			int z;
			//If it does not sense the primary user then the secondary user will transmit
			while(bfi.primaryon==0)
				{
				for(z=0; z<uninterruptedframes; z++){
					modulation_scheme ms = convertModScheme(suce.modScheme, &suce.bitsPerSym);
					fec_scheme fec0 = convertFECScheme(suce.innerFEC, verbose);
					fec_scheme fec1 = convertFECScheme(suce.outerFEC, verbose);
					usleep(1);
					txcvr.assemble_frame(header, payload, suce.payloadLen, ms, fec0, fec1);
					int isLastSymbol = 0;
					while(!isLastSymbol)
						{
						usleep(1);
						isLastSymbol = txcvr.write_symbol();
						if(usescenario)
						enactScenarioBasebandTx(txcvr.fgbuffer, txcvr.fgbuffer_len, suce, sc);
						txcvr.transmit_symbol();
						}
			   		txcvr.end_transmit_frame();
					usleep(100);
					if(adapt==1)
					postTxTasks(&suce, &msg.feed, verbose);
				}
			}
			time = 0;
			start = std::clock();
			//std::clock_t current;
			printf("SU sensing\n");

			//Once the primary user is detected the secondary user stops transmitting
			//and switches to sensing in a new while loop
			mess.number = secondarymsgnumber;
			mess.purpose = 'r';
			write(dsaCBs.client, (const void*)&mess, sizeof(mess));
			secondarymsgnumber++;
			while(bfi.primaryon==1)
				{
				time = 0;
				bfi.primaryon = 0;
				start = std::clock();
				std::clock_t current;

				//The while loop sets primaryon to 0 in the beginning. If the loop
				//finishes without a new primary transmission switching it to 1 then
				//the secondary user will assume it has stopped and resume transmitting
				//This while loop below will run for secondaryscantime seconds
				while(secondaryscantime > time)
					{
					current = std::clock();
					time = (current-start)/CLOCKS_PER_SEC;
					}			
				}
			}
		}

	if(secondary == 1 && dsaCBs.detectiontype == 't' && !receiver){
		struct broadcastfeedbackinfo bfi;
		mess.type = 'S';
		dsaCBs.usrptype = 'S';
		mess.msgreceived = 1;
		verbose = 0;
		if(broadcasting==1){
		bfi.primaryon = 0;
		bfi.user = 'S';
		bfi.client = dsaCBs.client;
		bfi.m_ptr = &msg;
		bfi.msgnumber = &secondarymsgnumber;
		pthread_create( &receiverfeedbackThread, NULL, feedbackThread, (void*) &bfi);
		}
		printf("secondary\n");

		//The secondary user has a payload of all zeroes
		for(unsigned int h = 0; h<8; h++){
			header[h] = 0;
		};
		for(unsigned int h = 0; h<suce.payloadLen; h++){
			payload[h] = 0;
		};
		//std::clock_t current;
		unsigned char * p = NULL;   // default subcarrier allocation
		if (verbose) 
		printf("Using ofdmtxrx\n");
		printf("%d %d %d\n", suce.numSubcarriers, suce.CPLen, suce.taperLen);

		//Sets up transceiver object
		ofdmtxrx txcvr(suce.numSubcarriers, suce.CPLen, suce.taperLen, p, dsaCallback, (void*) &dsaCBs);
		txcvr.set_tx_freq(suce.frequency);
		txcvr.set_tx_rate(suce.bandwidth);
		txcvr.set_tx_gain_soft(suce.txgain_dB);
		txcvr.set_tx_gain_uhd(suce.uhd_txgain_dB);
    	txcvr.set_rx_freq(frequency);
   		txcvr.set_rx_rate(suce.bandwidth);
    	txcvr.set_rx_gain_uhd(uhd_rxgain);
		//txcvr.start_rx();
		while(true)
			{
			printf("SU transmitting\n");
			mess.number = secondarymsgnumber;
			mess.purpose = 't';
			write(dsaCBs.client, (const void*)&mess, sizeof(mess));
			//printf("%c %c %d\n", mess.purpose, mess.type, mess.number);
			secondarymsgnumber++;
			int z;
			//If it does not sense the primary user then the secondary user will transmit
			while(bfi.energydetected==0)
				{
				for(z=0; z<uninterruptedframes; z++){
					modulation_scheme ms = convertModScheme(suce.modScheme, &suce.bitsPerSym);
					fec_scheme fec0 = convertFECScheme(suce.innerFEC, verbose);
					fec_scheme fec1 = convertFECScheme(suce.outerFEC, verbose);
					usleep(1);
					txcvr.assemble_frame(header, payload, suce.payloadLen, ms, fec0, fec1);
					int isLastSymbol = 0;
					while(!isLastSymbol)
						{
						usleep(1);
						isLastSymbol = txcvr.write_symbol();
						if(usescenario)
						enactScenarioBasebandTx(txcvr.fgbuffer, txcvr.fgbuffer_len, suce, sc);
						txcvr.transmit_symbol();
						}
			   		txcvr.end_transmit_frame();
					usleep(100);
					if(adapt==1)
					postTxTasks(&suce, &msg.feed, verbose);
				}
			}
			printf("SU sensing\n");

			//Once the primary user is detected the secondary user stops transmitting
			//and switches to sensing in a new while loop
			mess.number = secondarymsgnumber;
			mess.purpose = 'r';
			write(dsaCBs.client, (const void*)&mess, sizeof(mess));
			//printf("%c %c %d\n", mess.purpose, mess.type, mess.number);
			secondarymsgnumber++;
			while(bfi.energydetected==1)
				{
				z=0;
				/*bfi.primaryon = 0;
				start = std::clock();
				std::clock_t current;

				//The while loop sets primaryon to 0 in the beginning. If the loop
				//finishes without a new primary transmission switching it to 1 then
				//the secondary user will assume it has stopped and resume transmitting
				//This while loop below will run for secondaryscantime seconds
				while(secondaryscantime > time)
					{
					current = std::clock();
					time = (current-start)/CLOCKS_PER_SEC;
					}*/			
			}
		}
	}

	//If the secondary transmitter is using energy detection it will call the fftscan function to check for
	//spectrum holes
	if(secondary == 1 && dsaCBs.detectiontype == 'e' && !receiver){
		struct broadcastfeedbackinfo bfi;
		std::string args = "internal";
		//Creates usrp pointer that will be passed to fftscan function
		uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);

		int cantransmit = 0;
		int primaryoncounter;
		int primaryoffcounter;
		mess.type = 'S';
		dsaCBs.usrptype = 'S';
		mess.msgreceived = 1;
		verbose = 0;
		if(broadcasting==1){
		bfi.user = 'S';
		bfi.client = dsaCBs.client;
		bfi.m_ptr = &msg;
		bfi.msgnumber = &secondarymsgnumber;
		pthread_create( &receiverfeedbackThread, NULL, feedbackThread, (void*) &bfi);
		}
		printf("secondary\n");

		//The secondary user has a payload of all zeroes
		for(int h = 0; h<8; h++){
			header[h] = 0;
		};
		for(int h = 0; (unsigned int)h<suce.payloadLen; h++){
			payload[h] = 0;
		};
		unsigned char * p = NULL;   // default subcarrier allocation
		if (verbose) 
		printf("Using ofdmtxrx\n");
		printf("%d %d %d\n", suce.numSubcarriers, suce.CPLen, suce.taperLen);

		//Sets up transceiver object
		ofdmtxrx txcvr(suce.numSubcarriers, suce.CPLen, suce.taperLen, p, dsaCallback, (void*) &dsaCBs);
		txcvr.set_tx_freq(suce.frequency);
		txcvr.set_tx_rate(suce.bandwidth);
		txcvr.set_tx_gain_soft(suce.txgain_dB);
		txcvr.set_tx_gain_uhd(suce.uhd_txgain_dB);
    	txcvr.set_rx_freq(frequency);
   		txcvr.set_rx_rate(suce.bandwidth);
    	txcvr.set_rx_gain_uhd(uhd_rxgain);
		printf("\nMake sure no transmitters are transmitting before finding the noise floor\n");
		//txcvr.start_rx();
		float noisefloor = noise_floor(suce, usrp, fftinfo);
	
		printf("\nNoise floor found! Press any key to start secondary user");
		getchar();	
		cantransmit = 1;
		while(true)
			{
			printf("SU transmitting\n");
			mess.number = secondarymsgnumber;
			mess.purpose = 't';
			write(dsaCBs.client, (const void*)&mess, sizeof(mess));
			secondarymsgnumber++;
			int z;
			//If it does not sense the primary user then the secondary user will transmit
			while(cantransmit==1)
				{
				for(z=0; z<uninterruptedframes; z++)
					{
					modulation_scheme ms = convertModScheme(suce.modScheme, &suce.bitsPerSym);
					fec_scheme fec0 = convertFECScheme(suce.innerFEC, verbose);
					fec_scheme fec1 = convertFECScheme(suce.outerFEC, verbose);
					usleep(1);
					txcvr.assemble_frame(header, payload, suce.payloadLen, ms, fec0, fec1);
					int isLastSymbol = 0;

					while(!isLastSymbol)
						{
						usleep(1);;
						isLastSymbol = txcvr.write_symbol();
						if(usescenario)
						enactScenarioBasebandTx(txcvr.fgbuffer, txcvr.fgbuffer_len, suce, sc);
						txcvr.transmit_symbol();
					}

			   		txcvr.end_transmit_frame();
					usleep(100);
					if(adapt==1)
					postTxTasks(&puce, &msg.feed, verbose);
				}
				cantransmit = 0;
				primaryoncounter = 0;
				primaryoffcounter = 0;
			
				mess.number = secondarymsgnumber;
				mess.purpose = 'S';
				write(dsaCBs.client, (const void*)&mess, sizeof(mess));
				secondarymsgnumber++;

				//The secondary user will wait in this while loop and wait and see if any
				//primary users appear
				for(int h=0; h<fftinfo.testnumber; h++)
					{
					cantransmit = fftscan(suce, usrp, noisefloor, fftinfo);
					if(cantransmit==1){
						primaryoffcounter++;
					}
					else{
						primaryoncounter++;
					}	
				mess.number = secondarymsgnumber;
				mess.purpose = 's';
				write(dsaCBs.client, (const void*)&mess, sizeof(mess));
				secondarymsgnumber++;
				}
				
				if(primaryoffcounter > primaryoncounter)
					{
					cantransmit = 1;

				}
				else
					{
					cantransmit = 0;
				}
			}
			printf("SU sensing\n");

			//Once the primary user is detected the secondary user stops transmitting
			//and switches to sensing in a new while loop
			mess.number = secondarymsgnumber;
			mess.purpose = 'r';
			write(dsaCBs.client, (const void*)&mess, sizeof(mess));
			secondarymsgnumber++;
			while(cantransmit==0)
				{
				primaryoncounter = 0;
				primaryoffcounter = 0;
				//The while loop sets primaryon to 0 in the beginning. If the loop
				//finishes without a new primary transmission switching it to 1 then
				//the secondary user will assume it has stopped and resume transmitting
				//This while loop below will run for secondaryscantime seconds
				for(int h=0; h<fftinfo.testnumber; h++)
					{

					cantransmit = fftscan(suce, usrp, noisefloor, fftinfo);
					if(cantransmit==1){
						primaryoffcounter++;
					}
					else{
						primaryoncounter++;
					}
				}
				
				if(primaryoffcounter > primaryoncounter){
					cantransmit = 1;
				}
				else{
					cantransmit = 0;
			
				}
			}
		};
		return 0;
	}

	if(energy==1){
		std::string(args) = "";
		uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);
		struct message emsg;
		emsg.type = 's';

		//The energy detector sends 2 kinds of messages
		//'D' = Primary user detected
		//'d' = Primary user not detected
		emsg.purpose = 'f';
		emsg.number = 1;
		int cantransmit = 0;
		int primaryoncounter = 0;
		int primaryoffcounter = 0;
		float noisefloor;
		printf("\nMake sure the secondary transmitter is transmitting before finding the noise floor\n");
		printf("This insures that detection of the SU's energy won't cause a false detection\n");
		noisefloor = noise_floor(suce, usrp, fftinfo);
		printf("\nNoise floor found! Press any key to start energy detector\n");
		getchar();;
		while(true){
			primaryoffcounter = 0;
			primaryoncounter = 0;
			for(int h=0; h<fftinfo.testnumber; h++)
				{
				cantransmit = 0;
				cantransmit = fftscan(suce, usrp, noisefloor, fftinfo);
				if(cantransmit==1){
					primaryoffcounter++;
				}
				else{
					primaryoncounter++;
				}
			}		
			if(primaryoffcounter < primaryoncounter){
				emsg.purpose = 'D';				
				write(dsaCBs.client, &emsg, sizeof(emsg));
				emsg.number++;
			}
			if(primaryoffcounter > primaryoncounter){
				emsg.purpose = 'd';				
				write(dsaCBs.client, &emsg, sizeof(emsg));
				emsg.number++;
			}
			//printf("%c %d %d %d\n", emsg.purpose, emsg.number, primaryoffcounter, primaryoncounter);
		}
	}	
	//Primary receiver. Does nothing but use dsaCallback function to send feedback
	if(receiver == 1 && primary == 1){
		dsaCBs.usrptype = 'p';
		printf("receiver\n");
		unsigned char * p = NULL;   // default subcarrier allocation
		if (verbose) 
		printf("Using ofdmtxrx\n");;
		//Basic transceiver setup
		printf("%d %d %d\n", puce.numSubcarriers, puce.CPLen, puce.taperLen);
		ofdmtxrx txcvr(puce.numSubcarriers, puce.CPLen, puce.taperLen, p, dsaCallback, (void*) &dsaCBs);
		txcvr.set_tx_freq(puce.frequency);
		txcvr.set_tx_rate(puce.bandwidth);
		txcvr.set_tx_gain_soft(puce.txgain_dB);
		txcvr.set_tx_gain_uhd(puce.uhd_txgain_dB);
		txcvr.set_rx_freq(puce.frequency);
		txcvr.set_rx_rate(puce.bandwidth);
		txcvr.set_rx_gain_uhd(uhd_rxgain);
		txcvr.start_rx();

		//The receiver sits in this infinite while loop and does nothing but wait to receive
		//liquid frames that it will interpret with dsaCallback
		while(true){
			verbose=0;
		}
		return 0;

	}
	//Secondary Receiver. It does nothing but wait to receive liquid frames and call dsacallback
	//But when the the detection type in the master_dsa_file.txt is set to 'r'
	//that means the receivers do the sensing
	//The secondary receivers won't change their behavior, whenever they detect a primary frame
	//they will always send a TCP message to the secondary transmitter
	//but if the detection type is 'r' then the secondary transmitter will transmit and rest based on a variable
	//that changes whenever a message is received from a secondary receiver saying the primary user was detected
	//If the detection type is anything else then the secondary transmitter ignores this variable
	//So these receivers can technically sense the spectrum, but only through their callback function
	//For energy detection sensing the energy detector is used, because an energy detecting usrp object and
	//and liquid transceiver usrp object cannot both have their receivers on or overflows and problems will happen
	if(receiver == 1 && secondary == 1){
		dsaCBs.usrptype = 's';
		printf("receiver\n");
		unsigned char * p = NULL;   // default subcarrier allocation
		if (verbose) 
		printf("Using ofdmtxrx\n");

		//Basic transceiver setup
		printf("%d %d %d\n", suce.numSubcarriers, suce.CPLen, suce.taperLen);
		ofdmtxrx txcvr(suce.numSubcarriers, suce.CPLen, suce.taperLen, p, dsaCallback, (void*) &dsaCBs);
		txcvr.set_tx_freq(suce.frequency);
		txcvr.set_tx_rate(suce.bandwidth);
		txcvr.set_tx_gain_soft(suce.txgain_dB);
		txcvr.set_tx_gain_uhd(suce.uhd_txgain_dB);
		txcvr.set_rx_freq(suce.frequency);
		txcvr.set_rx_rate(suce.bandwidth);
		txcvr.set_rx_gain_uhd(uhd_rxgain);
		txcvr.start_rx();

		//The receiver sits in this infinite while loop and does nothing but wait to receive
		//liquid frames that it will interpret with dsaCallback
		while(true){
			verbose=0;
		}
		return 0;

	}
}

//If a receiver is being used but not DSA then a basic receiver is made. It does nothing
//but wait to receive transmissions and execute the dsaCallback function
//Currently doesn't serve a purpose
//It could later be adjusted so a non DSA radio can go through tests while broadcasting to multiple receivers
//with each receiver being initialized with this if statement
//and all of the feedback being interpreted by a TCP message reading thread on the transmitter
//Future work!!!
if(receiver == 1 && dsa != 1){
	ce = CreateCognitiveEngine();
	const char* str = "ce1.txt";
	readCEConfigFile(&ce, (char *)str, verbose);
	printf("receiver\n");
	unsigned char * p = NULL;   // default subcarrier allocation
	if (verbose) 
	printf("Using ofdmtxrx\n");

	//Basic transceiver setup
	printf("%d %d %d\n", ce.numSubcarriers, ce.CPLen, ce.taperLen);
	ofdmtxrx txcvr(ce.numSubcarriers, ce.CPLen, ce.taperLen, p, dsaCallback, (void*) &rxCBs);
	txcvr.set_tx_freq(ce.frequency);
	txcvr.set_tx_rate(ce.bandwidth);
	txcvr.set_tx_gain_soft(ce.txgain_dB);
	txcvr.set_tx_gain_uhd(ce.uhd_txgain_dB);
    txcvr.set_rx_freq(frequency);
    txcvr.set_rx_rate(bandwidth);
    txcvr.set_rx_gain_uhd(uhd_rxgain);
	txcvr.start_rx();

	//The receiver sits in this infinite while loop and does nothing but wait to receive
	//liquid frames that it will interpret with dsaCallback
	while(true){
		verbose=0;
	}
	return 0;

}

if(dsa && isController){

	//Acccumulator variables used to find percentages for byte and bit errors
	//There are 2 for each type of averaged feedback
	//Average Total Frame Feedback
	//Average Primary Frame Feedback
	//Average Primary Safe Frame Feedback (the SU is off)
	//Average Primary Collision Frame Feedback (the SU is on)
	//Average Secondary Frame Feedback
	//Average Secondary Safe Frame Feedback (the PU is off)
	//Average Secondary Collision Frame Feedback (the PU is on)
	float tfbbyte = 0.0;
	float tfbbit = 0.0;
	float pfbbyte = 0.0;
	float pfbbit = 0.0;
	float sfbbyte = 0.0;
	float sfbbit = 0.0;
	float psfbbyte = 0.0;
	float psfbbit = 0.0;
	float ssfbbyte = 0.0;
	float ssfbbit = 0.0;
	float pcfbbyte = 0.0;
	float pcfbbit = 0.0;
	float scfbbyte = 0.0;
	float scfbbit = 0.0;


	//These variables deal with checking how efficiently the secondary user utilizes the spectrum

	//The time intervals between when the controller checks if the spectrum is occupied or not
	float spectrumchecktime = 0.05;

	//Time variables used to check if a spectrumchecktime interval has passed
	std::clock_t spectrumstart;
	std::clock_t spectrumcurrent;

	//Accumulators that increment when the spectrum is empty, when it is being used, or whenever the spectrum is checked
	float spectrumempty = 0.0;
	float spectrumused = 0.0;
	float spectrumtotal = 0.0;
	float spectrumusedsecondary = 0.0;
	float spectrumusedprimary = 0.0;
	float spectrumunusedprimary = 0.0;
	float spectrumusedoverlap = 0.0;

	//Only becomes one once the primary user first starts transmitting
	//Makes sure spectrum efficiency measurements are started at the right time
	int primarystart = 0;



	fprintf(dataFile, "DSA CRTS\n");
    fprintf(dataFile,  "%-13s %-10s %-10s %-13s %-15s %-12s %-12s %-20s %-19s\n", "linetype","frameNum","evm (dB)","rssi (dB)","Byte Errors","Bit Errors", "Throughput", "Spectral Efficiency", "Average Goal Value");
    fflush(dataFile);

	//Set to 0 so that the rendezvous time won't be calculated for the first time the secondary user turns on, since
	//it should be turned on first before the primary user
	int canchecktime = 0;

	//Set to one whenever the primary user switches from on to off or off to on
	//Makes sure that evacuation time and rendezvous time are only calculated once per switch
	//Prevents problems from happening if secondary user turns off and on multiple times in one spectrum hole
	int primarychange = 0;

	//1 if the secondary user is currently sensing for the primary user
	//It is 1 if it is currently in sensing mode or if it is still in transmit mode but is paused
	//to check the spectrum before sending another burst
	int secondarysensing = 0;

	//Feedback structs calculated at end of DSA test

	//Average feedback of all feedback received
	struct feedbackStruct totalfb;

	//Average feedback of all primary feedback received
	struct feedbackStruct primaryfb;

	//Average feedback of all primary feedback received when secondary user was on
	struct feedbackStruct primarycollisionfb;

	//Average feedback of all secondary feedback received when primary user was on
	struct feedbackStruct secondarycollisionfb;

	//Average feedback of all secondary feedback received
	struct feedbackStruct secondaryfb;

	//Average feedback of all primary feedback received when secondary user was off
	struct feedbackStruct primarysafefb;

	//Average feedback of all secondary feedback received when primary user was off
	struct feedbackStruct secondarysafefb;

	//Accumulator variables that are incremented whenever a new feedback of their type is read in and added to their
	//respective feedback struct
	int tfb = 0;
	int pfb = 0;
	int pcfb = 0;
	int sfb = 0;
	int scfb = 0;
	int psfb = 0;
	int ssfb = 0;

	//Increments whenever a receiver cannot identify a header as primary or secondary
	int unknownheader = 0;
	
	//Time variables that show the start and end of the test cycles
	//Starting when the primary user first transmits and ending when the primary
	//user rests for its final cycle
	std::clock_t primarystarttime;
	std::clock_t primaryendtime;
	//Accumulator variables track how many feedback structs of each type have valid headers
	float headertfb = 0.0;
	float headerpfb = 0.0;
	float headersfb = 0.0;
	float headerpcfb = 0.0;
	float headerscfb = 0.0;
	float headerpsfb = 0.0;
	float headerssfb = 0.0;

	//Accumulator variables track how many feedback structs of each type have valid payloads
	float payloadtfb = 0.0;
	float payloadpfb = 0.0;
	float payloadsfb = 0.0;
	float payloadpcfb = 0.0;
	float payloadscfb = 0.0;
	float payloadpsfb = 0.0;
	float payloadssfb = 0.0;

	//The message number of the last received primary and secondary messages
	int latestprimary = 0;
	int latestsecondary = 0;

	//The total number of rendezvous times
	float rendcounter = 0.0;
	
	//The total number of evacuation times
	float evaccounter = 0.0;

	//Total number of false alarms
	float totalfalsealarm = 0.0;

	//Total number of missed spectrum holes
	float totalmissedhole = 0.0;

	//Total number of primary user cycles (one transmit burst and one rest interval)
	float totalcycles = 0.0;

	//Total number of successful spectrum hole identifications by the secondary user
	float success = 0.0;

	float falsealarmprob = 0.0;
	float probofdetection = 0.0;
	std::clock_t totalevacuationtime = 0;
	std::clock_t totalrendevoustime = 0;
	std::clock_t averageevacuationtime;
	std::clock_t averagerendevoustime;
	float evacuationtimelist[100];
	float rendezvoustimelist[100];

	//Before starting its message checking while loop the controller sets all of its feedback
	//structs to have variables of all 0's
	totalfb.header_valid = 0;
	totalfb.payload_valid = 0;
   	totalfb.payload_len = 0;
	totalfb.payloadByteErrors = 0;
   	totalfb.payloadBitErrors = 0;
	totalfb.iteration = 0;
   	totalfb.evm = 0.0;
	totalfb. rssi = 0.0;
	totalfb.cfo = 0.0;
	totalfb.block_flag = 0;
	secondarycollisionfb.header_valid = 0;
	secondarycollisionfb.payload_valid = 0;
   	secondarycollisionfb.payload_len = 0;
	secondarycollisionfb.payloadByteErrors = 0;
   	secondarycollisionfb.payloadBitErrors = 0;
	secondarycollisionfb.iteration = 0;
   	secondarycollisionfb.evm = 0.0;
	secondarycollisionfb. rssi = 0.0;
	secondarycollisionfb.cfo = 0.0;
	secondarycollisionfb.block_flag = 0;
	primaryfb.header_valid = 0;
	primaryfb.payload_valid = 0;
   	primaryfb.payload_len = 0;
	primaryfb.payloadByteErrors = 0;
   	primaryfb.payloadBitErrors = 0;
	primaryfb.iteration = 0;
   	primaryfb.evm = 0.0;
	primaryfb. rssi = 0.0;
	primaryfb.cfo = 0.0;
	primaryfb.block_flag = 0;
	primarycollisionfb.header_valid = 0;
	primarycollisionfb.payload_valid = 0;
   	primarycollisionfb.payload_len = 0;
	primarycollisionfb.payloadByteErrors = 0;
   	primarycollisionfb.payloadBitErrors = 0;
	primarycollisionfb.iteration = 0;
   	primarycollisionfb.evm = 0.0;
	primarycollisionfb. rssi = 0.0;
	primarycollisionfb.cfo = 0.0;
	primarycollisionfb.block_flag = 0;
	secondaryfb.header_valid = 0;
	secondaryfb.payload_valid = 0;
   	secondaryfb.payload_len = 0;
	secondaryfb.payloadByteErrors = 0;
   	secondaryfb.payloadBitErrors = 0;
	secondaryfb.iteration = 0;
   	secondaryfb.evm = 0.0;
	secondaryfb. rssi = 0.0;
	secondaryfb.cfo = 0.0;
	secondaryfb.block_flag = 0;
	primarysafefb.header_valid = 0;
	primarysafefb.payload_valid = 0;
   	primarysafefb.payload_len = 0;
	primarysafefb.payloadByteErrors = 0;
   	primarysafefb.payloadBitErrors = 0;
	primarysafefb.iteration = 0;
   	primarysafefb.evm = 0.0;
	primarysafefb. rssi = 0.0;
	primarysafefb.cfo = 0.0;
	primarysafefb.block_flag = 0;	
	secondarysafefb.header_valid = 0;
	secondarysafefb.payload_valid = 0;
   	secondarysafefb.payload_len = 0;
	secondarysafefb.payloadByteErrors = 0;
   	secondarysafefb.payloadBitErrors = 0;
	secondarysafefb.iteration = 0;
   	secondarysafefb.evm = 0.0;
	secondarysafefb. rssi = 0.0;
	secondarysafefb.cfo = 0.0;
	secondarysafefb.block_flag = 0;
		
	std::clock_t primaryofftime = 0;
	std::clock_t primaryontime = 0;
	std::clock_t secondaryofftime = 0;
	std::clock_t secondaryontime = 0;
	std::clock_t evacuationtime;
	std::clock_t rendevoustime;
	int primary = 0;
	//int secondary = 0;
	std::clock_t time = std::clock();
	int loop = 1;
	spectrumstart = std::clock();

	//This loop will iterate and read in new messages until the primary user finishes its  cycles and sends the
	//finish message with a purpose char of 'F'
	while(loop){
		spectrumcurrent = std::clock();

		//This if statement runs every spectrumchecktime interval and will see if the spectrum is currently being utilized
		//by the primary or secondary user
		if(spectrumchecktime<(float)((spectrumcurrent-spectrumstart)/CLOCKS_PER_SEC)){
			if(primarystart==1){
				if(primary==1 and secondarysensing == 0){
				spectrumused++;
				spectrumusedprimary++;
				spectrumusedsecondary++;
				spectrumusedoverlap++;
				}
				else{
					if(primary==1){
						spectrumused++;
						spectrumusedprimary++;
					}
					else{
						spectrumunusedprimary++;
						if(secondarysensing==0){
							spectrumused++;
							spectrumusedsecondary++;
						}
						else{
							spectrumempty++;
						}
					}
				}
				spectrumtotal++;
				
			}
			spectrumstart = std::clock();
		}
		if(msg.msgreceived == 1){
			if(msg.type == 'P'){
				if(latestprimary<msg.number){
					
					if(msg.purpose == 't'){
						if(primarystart==0){
							primarystarttime = std::clock();
							primarystart = 1;
						}
						primary = 1;
						primarychange = 1;
						latestprimary = msg.number;
						time = std::clock();
						primaryontime = time;
						printf("Primary user started transmitting at time %f seconds\n", ((float)time/CLOCKS_PER_SEC));
						fprintf(dataFile, "Primary user started transmitting at time %f seconds\n", ((float)time/CLOCKS_PER_SEC));
					}
					if(msg.purpose == 'r'){
						primary = 0;
						primarychange = 1;
						latestprimary = msg.number;
						time = std::clock();
						primaryofftime = time;
						++totalcycles;
						printf("Primary user stopped transmitting at time %f seconds\n", ((float)time/CLOCKS_PER_SEC));
						fprintf(dataFile, "Primary user stopped transmitting at time %f seconds\n", ((float)time/CLOCKS_PER_SEC));
					}
					if(msg.purpose == 'u'){
						unknownheader++;
					}
					if(msg.purpose == 'F'){
						primary = 0;
						latestprimary = msg.number;
						primaryendtime = std::clock();
						loop = 0;
						break;
					}
					if(msg.purpose == 'f'){
						latestprimary = msg.number;
						fprintf(dataFile, "%-13s %-10i %-10.2f %-13.2f %-15.2d %-12.2d %-12.2d %-20.2f %-19.2f\n", 
							"pudata:", msg.feed.iteration,  msg.feed.evm, msg.feed.rssi, msg.feed.payloadByteErrors,
							msg.feed.payloadBitErrors, 1, 1.0, 1.0);
						if(msg.feed.header_valid==1){
							headertfb++;
							headerpfb++;

							if(secondarysensing==0){
								headerpcfb++;
							}
							else{
								headerpsfb++;
							}
						}
						if(msg.feed.payload_valid==1){
							payloadtfb++;
							payloadpfb++;
							if(secondarysensing==0){
								payloadpcfb++;
							}
							else{
								payloadpsfb++;
							}
						}
						tfbbyte += msg.feed.payloadByteErrors;
						tfbbit += msg.feed.payloadBitErrors;
						totalfb = feedbackadder(totalfb, msg.feed);
						tfb++;
						primaryfb = feedbackadder(primaryfb, msg.feed);
						pfb++;
						pfbbyte += msg.feed.payloadByteErrors;
						pfbbit += msg.feed.payloadBitErrors;
						if(secondarysensing==0){
							primarycollisionfb = feedbackadder(primarycollisionfb, msg.feed);
							pcfb++;
							pcfbbyte += msg.feed.payloadByteErrors;
							pcfbbit += msg.feed.payloadBitErrors;
						}
						else{
							primarysafefb = feedbackadder(primarysafefb, msg.feed);
							psfb++;
							psfbbyte += msg.feed.payloadByteErrors;
							psfbbit += msg.feed.payloadBitErrors;
						}
					}
				}
			}
			if(msg.type == 'S'){
				//printf("Secondary %d %d\n", msg.number, latestsecondary);
				if(latestsecondary<msg.number){

					//The secondary user has started transmitting
					if(msg.purpose == 't'){
						secondary = 1;
						secondarysensing = 0;
						latestsecondary = msg.number;
						time = std::clock();
						secondaryontime = time;
						printf("Secondary user started transmitting at time %f seconds\n", ((float)time/CLOCKS_PER_SEC));
						fprintf(dataFile, "Secondary user started transmitting at time %f seconds\n", ((float)time/CLOCKS_PER_SEC));
						if(primary == 0 && canchecktime && primarychange){
							rendevoustime = secondaryontime - primaryofftime;
							printf("Rendezvous time = %f seconds\n", ((float)rendevoustime/CLOCKS_PER_SEC));
							fprintf(dataFile, "Rendezvous time = %f seconds\n", ((float)rendevoustime/CLOCKS_PER_SEC));
							rendezvoustimelist[(int)rendcounter] = ((float)rendevoustime/CLOCKS_PER_SEC);
							success++;
							rendcounter++;
							totalrendevoustime += rendevoustime;
						}
						else{
							canchecktime = 1;
						}
						primarychange = 0;
					}

					//Sent when a receiver tells the secondary user that it could't decode a header
					if(msg.purpose == 'u'){
						unknownheader++;
					}

					//Sent when the secondary user starts sensing (But not necessarily in the sensing while loop)
					if(msg.purpose == 'S'){
						secondarysensing=1;
					}

					//Sent when the secondary user is done sensing
					if(msg.purpose == 's'){
						secondarysensing=0;
					}

					//The secondary user has stopped transmitting
					if(msg.purpose == 'r'){
						secondary = 0;
						secondarysensing = 1;
						latestsecondary = msg.number;
						time = std::clock();
						secondaryofftime = time;
						printf("Secondary user stopped transmitting at time %f seconds\n", ((float)time/CLOCKS_PER_SEC));
						fprintf(dataFile, "Secondary user stopped transmitting at time %f seconds\n", ((float)time/CLOCKS_PER_SEC));
						if(primary==1 && primarychange){
							evacuationtime = secondaryofftime - primaryontime;
							printf("Evacuation time = %f seconds\n", ((float)evacuationtime/CLOCKS_PER_SEC));
							fprintf(dataFile, "Evacuation time = %f seconds\n", ((float)evacuationtime/CLOCKS_PER_SEC));
							evacuationtimelist[(int)evaccounter] = ((float)evacuationtime/CLOCKS_PER_SEC);
							evaccounter++;
							totalevacuationtime += evacuationtime;
						}
						if(primary == 0){
							printf("Wasted Spectrum Hole\n");
							fprintf(dataFile, "Wasted Spectrum Hole\n");
							++totalmissedhole;
						}
						primarychange = 0;
					}

					//The secondary user is sending feedback to the controller
					//The feedback is written to the data file and added to the proper accumulator feedback
					//structs where it will later be averaged together
					if(msg.purpose == 'f'){
						//feedbackStruct_print(&msg.feed);
						latestsecondary = msg.number;
						fprintf(dataFile, "%-13s %-10i %-10.2f %-13.2f %-15.2d %-12.2d %-12.2d %-20.2f %-19.2f\n", 
							"sudata:", msg.feed.iteration,  msg.feed.evm, msg.feed.rssi, msg.feed.payloadByteErrors,
							msg.feed.payloadBitErrors, 1, 1.0, 1.0);
						if(msg.feed.header_valid==1){
							headertfb++;
							headersfb++;
							if(primary==1){
								headerscfb++;
							}
							else{
								headerssfb++;
							}
						}
						if(msg.feed.payload_valid==1){
							payloadtfb++;
							payloadsfb++;
							if(primary==1){
								payloadscfb++;
							}
							else{
								payloadssfb++;
							}
						}
						tfbbyte += msg.feed.payloadByteErrors;
						tfbbit += msg.feed.payloadBitErrors;
						totalfb = feedbackadder(totalfb, msg.feed);
						totalfb = feedbackadder(totalfb, msg.feed);
						tfb++;
						sfbbyte += msg.feed.payloadByteErrors;
						sfbbit += msg.feed.payloadBitErrors;
						secondaryfb = feedbackadder(secondaryfb, msg.feed);
						sfb++;
						if(primary==1){
							secondarycollisionfb = feedbackadder(secondarycollisionfb, msg.feed);
							scfb++;
							scfbbyte += msg.feed.payloadByteErrors;
							scfbbit += msg.feed.payloadBitErrors;
						}
						else{
							secondarysafefb = feedbackadder(secondarysafefb, msg.feed);
							ssfb++;
							ssfbbyte += msg.feed.payloadByteErrors;
							ssfbbit += msg.feed.payloadBitErrors;
						}
					}
				}
			}
		msg.msgreceived = 0;
		}
	};

	//After the primary transmitter finishes its cycles and sends the finished message to the controller
	//the controller will stop iterating through the while loop and print out the data to the standard output
	//and the data file
	printf("Testing Complete\n");
	printf("Total Test Time: %f seconds\n", (float)((primaryendtime-primarystarttime)/CLOCKS_PER_SEC));
	fprintf(dataFile, "Total Test Time: %f seconds\n", (float)((primaryendtime-primarystarttime)/CLOCKS_PER_SEC));

	//What percentage of the time the spectrum was occupied
	printf("\nSpectrum Usage: %%%f\n", 100*(spectrumused/spectrumtotal));
	printf("Primary User Spectrum Usage: %%%f\n", 100*(spectrumusedprimary/spectrumtotal));
	printf("Secondary User Spectrum Usage: %%%f\n", 100*(spectrumusedsecondary/spectrumtotal));
	printf("Secondary User Free Spectrum Usage: %%%f\n", 100*((spectrumusedsecondary-spectrumusedoverlap)/spectrumunusedprimary));
	printf("Overlapped Spectrum Usage: %%%f\n", 100*(spectrumusedoverlap/spectrumtotal));
	fprintf(dataFile, "\nSpectrum Usage: %%%f\n", 100*(spectrumused/spectrumtotal));
	fprintf(dataFile, "Primary User Spectrum Usage: %%%f\n", 100*(spectrumusedprimary/spectrumtotal));
	fprintf(dataFile, "Secondary User Spectrum Usage: %%%f\n", 100*(spectrumusedsecondary/spectrumtotal));
	fprintf(dataFile, "Secondary User Free Spectrum Usage: %%%f\n", 100*((spectrumusedsecondary-spectrumusedoverlap)/spectrumunusedprimary));
	fprintf(dataFile, "Overlapped Spectrum Usage: %%%f\n", 100*(spectrumusedoverlap/spectrumtotal));
	printf("\n%d unidentifiable headers\n\n", unknownheader);
	fprintf(dataFile, "\n%d unidentifiable headers\n\n", unknownheader);
	falsealarmprob = totalfalsealarm/totalcycles;
	printf("Probability of False Alarm: %f\n", falsealarmprob);
	fprintf(dataFile, "Probability of False Alarm: %f\n", falsealarmprob);
	probofdetection = success/totalcycles;
	printf("Probability of Detection: %f\n", probofdetection);
	fprintf(dataFile, "Probability of Detection: %f\n", probofdetection);
	averageevacuationtime = totalevacuationtime/evaccounter;
	printf("Average Evacuation Time: %f seconds\n", ((float)averageevacuationtime)/CLOCKS_PER_SEC);
	fprintf(dataFile, "Average Evacuation Time: %f seconds\n", ((float)averageevacuationtime)/CLOCKS_PER_SEC);
	averagerendevoustime = totalrendevoustime/rendcounter;
	printf("Average Rendezvous Time: %f seconds\n", ((float)averagerendevoustime)/CLOCKS_PER_SEC);
	fprintf(dataFile, "Average Rendezvous Time: %f seconds\n", ((float)averagerendevoustime)/CLOCKS_PER_SEC);

	//The if statements prevent division by 0
	//The variables in the feedback struct are divided by the accumulator variables
	//to create average feedbacks for each type
	//These feedbacks will then be printed out and written to the data file
	if(tfb>0){
	totalfb.header_valid = (int)ceil((float)totalfb.header_valid/(float)tfb);
	totalfb.payload_valid = (int)ceil((float)totalfb.payload_valid/(float)tfb);
   	totalfb.payload_len = (int)ceil((float)totalfb.payload_len/(float)tfb);
	totalfb.payloadByteErrors = (int)ceil((float)totalfb.payloadByteErrors/(float)tfb);
   	totalfb.payloadBitErrors = (int)ceil((float)totalfb.payloadBitErrors/(float)tfb);
	totalfb.iteration /= tfb;
   	totalfb.evm /= tfb;
	totalfb. rssi /= tfb;
	totalfb.cfo /= tfb;
	totalfb.block_flag /= tfb;}
	if(scfb>0){
	secondarycollisionfb.header_valid = (int)ceil((float)secondarycollisionfb.header_valid/(float)scfb);
	secondarycollisionfb.payload_valid = (int)ceil((float)secondarycollisionfb.payload_valid/(float)scfb);
   	secondarycollisionfb.payload_len = (int)ceil((float)secondarycollisionfb.payload_len/(float)scfb);
	secondarycollisionfb.payloadByteErrors = (int)ceil((float)secondarycollisionfb.payloadByteErrors/(float)scfb);
   	secondarycollisionfb.payloadBitErrors = (int)ceil((float)secondarycollisionfb.payloadBitErrors/(float)scfb);
	secondarycollisionfb.iteration /= scfb;
   	secondarycollisionfb.evm /= scfb;
	secondarycollisionfb. rssi /= scfb;
	secondarycollisionfb.cfo /= scfb;
	secondarycollisionfb.block_flag /= scfb;}
	if(pfb>0){
	primaryfb.header_valid = (int)ceil((float)primaryfb.header_valid/(float)pfb);
	primaryfb.payload_valid = (int)ceil((float)primaryfb.payload_valid/(float)pfb);
   	primaryfb.payload_len = (int)ceil((float)primaryfb.payload_len/(float)pfb);
	primaryfb.payloadByteErrors = (int)ceil((float)primaryfb.payloadByteErrors/(float)pfb);
   	primaryfb.payloadBitErrors = (int)ceil((float)primaryfb.payloadBitErrors/(float)pfb);
	primaryfb.iteration /= pfb;
   	primaryfb.evm /= pfb;
	primaryfb. rssi /= pfb;
	primaryfb.cfo /= pfb;
	primaryfb.block_flag /= pfb;}
	if(pcfb>0){
	primarycollisionfb.header_valid = (int)ceil((float)primarycollisionfb.header_valid/(float)pcfb);
	primarycollisionfb.payload_valid = (int)ceil((float)primarycollisionfb.payload_valid/(float)pcfb);
   	primarycollisionfb.payload_len = (int)ceil((float)primarycollisionfb.payload_len/(float)pcfb);
	primarycollisionfb.payloadByteErrors = (int)ceil((float)primarycollisionfb.payloadByteErrors/(float)pcfb);
   	primarycollisionfb.payloadBitErrors = (int)ceil((float)primarycollisionfb.payloadBitErrors/(float)pcfb);
	primarycollisionfb.iteration /= pcfb;
   	primarycollisionfb.evm /= pcfb;
	primarycollisionfb. rssi /= pcfb;
	primarycollisionfb.cfo /= pcfb;
	primarycollisionfb.block_flag /= pcfb;}
	if(sfb>0){
	secondaryfb.header_valid = (int)ceil((float)secondaryfb.header_valid/(float)sfb);
	secondaryfb.payload_valid = (int)ceil((float)secondaryfb.payload_valid/(float)sfb);
   	secondaryfb.payload_len = (int)ceil((float)secondaryfb.payload_len/(float)sfb);
	secondaryfb.payloadByteErrors = (int)ceil((float)secondaryfb.payloadByteErrors/(float)sfb);
   	secondaryfb.payloadBitErrors = (int)ceil((float)secondaryfb.payloadBitErrors/(float)sfb);
	secondaryfb.iteration /= sfb;
   	secondaryfb.evm /= sfb;
	secondaryfb. rssi /= sfb;
	secondaryfb.cfo /= sfb;
	secondaryfb.block_flag /= sfb;}
	if(ssfb>0){
	secondarysafefb.header_valid = (int)ceil((float)secondarysafefb.header_valid/(float)ssfb);
	secondarysafefb.payload_valid = (int)ceil((float)secondarysafefb.payload_valid/(float)ssfb);
   	secondarysafefb.payload_len = (int)ceil((float)secondarysafefb.payload_len/(float)ssfb);
	secondarysafefb.payloadByteErrors = (int)ceil((float)secondarysafefb.payloadByteErrors/(float)ssfb);
   	secondarysafefb.payloadBitErrors = (int)ceil((float)secondarysafefb.payloadBitErrors/(float)ssfb);
	secondarysafefb.iteration /= ssfb;
   	secondarysafefb.evm /= ssfb;
	secondarysafefb. rssi /= ssfb;
	secondarysafefb.cfo /= ssfb;
	secondarysafefb.block_flag /= ssfb;}
	if(psfb>0){
	primarysafefb.header_valid = (int)ceil((float)primarysafefb.header_valid/(float)psfb);
	primarysafefb.payload_valid = (int)ceil((float)primarysafefb.payload_valid/(float)psfb);
   	primarysafefb.payload_len = (int)ceil((float)primarysafefb.payload_len/(float)psfb);
	primarysafefb.payloadByteErrors = (int)ceil((float)primarysafefb.payloadByteErrors/(float)psfb);
   	primarysafefb.payloadBitErrors = (int)ceil((float)primarysafefb.payloadBitErrors/(float)psfb);
	primarysafefb.iteration /= psfb;
   	primarysafefb.evm /= psfb;
	primarysafefb. rssi /= psfb;
	primarysafefb.cfo /= psfb;
	primarysafefb.block_flag /= psfb;}
	if(tfb<1)
	tfb=1;
	if(pfb<1)
	pfb=1;
	if(pcfb<1)
	pcfb=1;
	if(sfb<1)
	sfb=1;
	if(psfb<1)
	psfb=1;
	if(ssfb<1)
	ssfb=1;
	if(scfb<1)
	scfb=1;
    fprintf(dataFile, "%-13s %-10s %-10s %-13s %-15s %-12s %-12s %-20s %-19s\n", "linetype","frameNum","evm (dB)","rssi (dB)","Byte Errors","Bit Errors", "Throughput", "Spectral Efficiency", "Average Goal Value");
	printf("\n%d Total Frames\nAverage Total Frame Feedback\n\n", tfb);
	printf("Total Frame Valid Header Percentage: %%%f\n", headertfb/tfb * 100);
	printf("Total Frame Valid Payload Percentage: %%%f\n", payloadtfb/tfb * 100);
	printf("Total Frame Payload Byte Errors: %f\n", tfbbyte/tfb);
	printf("Total Frame Payload Bit Errors: %f\n", tfbbit/tfb);
	fprintf(dataFile, "\n%d Total Frames\nAverage Total Frame Feedback\n\n", tfb);
	fprintf(dataFile, "Total Frame Valid Header Percentage: %%%f\n", headertfb/tfb * 100);
	fprintf(dataFile, "Total Frame Valid Payload Percentage: %%%f\n", payloadtfb/tfb * 100);
	fprintf(dataFile, "Total Frame Payload Byte Errors: %f\n", tfbbyte/tfb);
	fprintf(dataFile, "Total Frame Payload Bit Errors: %f\n", tfbbit/tfb);

	fprintf(dataFile, "%-13s %-10i %-10.2f %-13.2f %-15.2d %-12.2d %-12.2d %-20.2f %-19.2f\n", 
		"totalframes:", totalfb.iteration,  totalfb.evm, totalfb.rssi, totalfb.payloadByteErrors,
		totalfb.payloadBitErrors, 1, 1.0, 1.0);
	feedbackStruct_print(&totalfb);
	printf("\n%d Primary Frames\nAverage Primary Frame Feedback\n\n", pfb);
	printf("Primary Frame Valid Header Percentage: %%%f\n", headerpfb/pfb * 100);
	printf("Primary Frame Valid Payload Percentage: %%%f\n", payloadpfb/pfb * 100);
	printf("Primary Frame Payload Byte Errors: %f\n", pfbbyte/pfb);
	printf("Primary Frame Payload Bit Errors: %f\n", pfbbit/pfb);
	fprintf(dataFile, "\n%d Primary Frames\nAverage Primary Frame Feedback\n\n", pfb);
	fprintf(dataFile, "Primary Frame Valid Header Percentage: %%%f\n", headerpfb/pfb * 100);
	fprintf(dataFile, "Primary Frame Valid Payload Percentage: %%%f\n", payloadpfb/pfb * 100);
	fprintf(dataFile, "Primary Frame Payload Byte Errors: %f\n", pfbbyte/pfb);
	fprintf(dataFile, "Primary Frame Payload Bit Errors: %f\n", pfbbit/pfb);

	fprintf(dataFile, "%-13s %-10i %-10.2f %-13.2f %-15.2d %-12.2d %-12.2d %-20.2f %-19.2f\n", 
		"puframes:", primaryfb.iteration,  primaryfb.evm, primaryfb.rssi, primaryfb.payloadByteErrors,
		primaryfb.payloadBitErrors, 1, 1.0, 1.0);
	feedbackStruct_print(&primaryfb);
	printf("\n%d Safe Primary Frames\nAverage Safe Primary Frame Feedback\n\n", psfb);
	printf("Safe Primary Frame Valid Header Percentage: %%%f\n", headerpsfb/psfb * 100);
	printf("Safe Primary Frame Valid Payload Percentage: %%%f\n", payloadpsfb/psfb * 100);
	printf("Safe Primary Frame Payload Byte Errors: %f\n", psfbbyte/psfb);
	printf("Safe Primary Frame Payload Bit Errors: %f\n", psfbbit/psfb);

	fprintf(dataFile, "\n%d Primary Frames\nAverage Primary Frame Feedback\n\n", psfb);
	fprintf(dataFile, "Safe Primary Frame Valid Header Percentage: %%%f\n", headerpsfb/psfb * 100);
	fprintf(dataFile, "Safe Primary Frame Valid Payload Percentage: %%%f\n", payloadpsfb/psfb * 100);
	fprintf(dataFile, "Safe Primary Frame Payload Byte Errors: %f\n", psfbbyte/psfb);
	fprintf(dataFile, "Safe Primary Frame Payload Bit Errors: %f\n", psfbbit/psfb);
	fprintf(dataFile, "\n%d Primary Frames\nAverage Primary Frame Feedback\n\n", psfb);
	fprintf(dataFile, "%-13s %-10i %-10.2f %-13.2f %-15.2d %-12.2d %-12.2d %-20.2f %-19.2f\n", 
		"puframes:", primarysafefb.iteration,  primarysafefb.evm, primarysafefb.rssi, primarysafefb.payloadByteErrors,
		primarysafefb.payloadBitErrors, 1, 1.0, 1.0);
	feedbackStruct_print(&primarysafefb);
	printf("\n%d Primary Collision Frames\nAverage Primary Collision Frame Feedback\n\n", pcfb);
	printf("Primary Collision Frame Valid Header Percentage: %%%f\n", headerpcfb/pcfb * 100);
	printf("Primary Collision Frame Valid Payload Percentage: %%%f\n", payloadpcfb/pcfb * 100);
	fprintf(dataFile, "Primary Collision Frame Valid Header Percentage: %%%f\n", headerpcfb/pcfb * 100);
	fprintf(dataFile, "Primary Collision Frame Valid Payload Percentage: %%%f\n", payloadpcfb/pcfb * 100);
	fprintf(dataFile, "\n%d Primary Collision Frames\nAverage Primary Collision Frame Feedback\n\n", pcfb);
	printf("Primary Collision Frame Payload Byte Errors: %f\n", pcfbbyte/pcfb);
	printf("Primary Collision Frame Payload Bit Errors: %f\n", pcfbbit/pcfb);
	fprintf(dataFile, "Primary Collision Frame Payload Byte Errors: %f\n", pcfbbyte/pcfb);
	fprintf(dataFile, "Primary Collision Frame Payload Bit Errors: %f\n", pcfbbit/pcfb);
	fprintf(dataFile,"%-13s %-10i %-10.2f %-13.2f %-15.2d %-12.2d %-12.2d %-20.2f %-19.2f\n", 
		"puCframes:", primarycollisionfb.iteration,  primarycollisionfb.evm, primarycollisionfb.rssi, primarycollisionfb.payloadByteErrors,
		primarycollisionfb.payloadBitErrors, 1, 1.0, 1.0);
	feedbackStruct_print(&primarycollisionfb);
	printf("\n%d Secondary Frames\nAverage Secondary Frame Feedback\n\n", sfb);
	printf("Secondary Frame Valid Header Percentage: %%%f\n", headersfb/sfb * 100);
	printf("Secondary Frame Valid Payload Percentage: %%%f\n", payloadsfb/sfb * 100);
	fprintf(dataFile, "\n%d Secondary Frames\nAverage Secondary Frame Feedback\n\n", sfb);
	fprintf(dataFile, "Secondary Frame Valid Header Percentage: %%%f\n", headersfb/sfb * 100);
	fprintf(dataFile, "Secondary Frame Valid Payload Percentage: %%%f\n", payloadsfb/sfb * 100);
	printf("Secondary Frame Payload Byte Errors: %f\n", sfbbyte/sfb);
	printf("Secondary Frame Payload Bit Errors: %f\n", sfbbit/sfb);
	fprintf(dataFile, "Secondary Frame Payload Byte Errors: %f\n", sfbbyte/sfb);
	fprintf(dataFile, "Secondary Frame Payload Bit Errors: %f\n", sfbbit/sfb);

	fprintf(dataFile, "%-13s %-10i %-10.2f %-13.2f %-15.2d %-12.2d %-12.2d %-20.2f %-19.2f\n", 
		"suframes:", secondaryfb.iteration,  secondaryfb.evm, secondaryfb.rssi, secondaryfb.payloadByteErrors,
		secondaryfb.payloadBitErrors, 1, 1.0, 1.0);
	feedbackStruct_print(&secondaryfb);
	printf("\n%d Safe Secondary Frames\nAverage Safe Secondary Frame Feedback\n\n", ssfb);
	printf("Safe Secondary Frame Valid Header Percentage: %%%f\n", headerssfb/ssfb * 100);
	printf("Safe Secondary Frame Valid Payload Percentage: %%%f\n", payloadssfb/ssfb * 100);
	fprintf(dataFile, "\n%d Safe Secondary Frames\nAverage Secondary Frame Feedback\n\n", ssfb);
	fprintf(dataFile, "Safe Secondary Frame Valid Header Percentage: %%%f\n", headerssfb/ssfb * 100);
	fprintf(dataFile, "Safe Secondary Frame Valid Payload Percentage: %%%f\n", payloadssfb/ssfb * 100);
	printf("Safe Secondary Frame Payload Byte Errors: %f\n", ssfbbyte/ssfb);
	printf("Safe Secondary Frame Payload Bit Errors: %f\n", ssfbbit/ssfb);
	fprintf(dataFile, "Safe Secondary Frame Payload Byte Errors: %f\n", ssfbbyte/ssfb);
	fprintf(dataFile, "Safe Secondary Frame Payload Bit Errors: %f\n", ssfbbit/ssfb);

	fprintf(dataFile, "%-13s %-10i %-10.2f %-13.2f %-15.2d %-12.2d %-12.2d %-20.2f %-19.2f\n", 
		"suframes:", secondarysafefb.iteration,  secondarysafefb.evm, secondarysafefb.rssi, secondarysafefb.payloadByteErrors,
		secondarysafefb.payloadBitErrors, 1, 1.0, 1.0);
	feedbackStruct_print(&secondarysafefb);
	printf("\n%d Secondary Collision Frames\nAverage Secondary Collision Frame Feedback\n\n", scfb);
	printf("Secondary Collision Frame Valid Header Percentage: %%%f\n", headerscfb/scfb * 100);
	printf("Secondary Collision Frame Valid Payload Percentage: %%%f\n", payloadscfb/scfb * 100);
	fprintf(dataFile, "\n%d Secondary Collision Frames\nAverage Secondary Collision Frame Feedback\n\n", scfb);
	fprintf(dataFile, "Secondary Collision Frame Valid Header Percentage: %%%f\n", headerscfb/scfb * 100);
	fprintf(dataFile, "Secondary Collision Frame Valid Payload Percentage: %%%f\n", payloadscfb/scfb * 100);
	printf("Secondary Collision Frame Payload Byte Errors: %f\n", scfbbyte/scfb);
	printf("Secondary Collision Frame Payload Bit Errors: %f\n", scfbbit/scfb);
	fprintf(dataFile, "Secondary Collision Frame Payload Byte Errors: %f\n", scfbbyte/scfb);
	fprintf(dataFile, "Secondary Collision Frame Payload Bit Errors: %f\n", scfbbit/scfb);

	fprintf(dataFile,"%-13s %-10i %-10.2f %-13.2f %-15.2d %-12.2d %-12.2d %-20.2f %-19.2f\n", 
		"suCframes:", secondarycollisionfb.iteration,  secondarycollisionfb.evm, secondarycollisionfb.rssi, secondarycollisionfb.payloadByteErrors,
		secondarycollisionfb.payloadBitErrors, 1, 1.0, 1.0);
	feedbackStruct_print(&secondarycollisionfb);

	//A list of evacuation times and rendezvous times are written to the data file
	fprintf(dataFile, "\n\nEvacuation Times\n\n");
	printf("\n\nEvacuation Times\n\n");
	int v;
	for(v=0; v<evaccounter; v++){
		fprintf(dataFile, "%f\n", evacuationtimelist[v]);
		printf("%f\n", evacuationtimelist[v]);
	}
	fprintf(dataFile, "\n\nRendezvous Times\n\n");
	printf("\n\nRendezvous Times\n\n");
	for(v=0; v<rendcounter; v++){
		fprintf(dataFile, "%f\n", rendezvoustimelist[v]);
		printf("%f\n", rendezvoustimelist[v]);
	}
	return 1;
}

if(tester==1){
	int w;
	int txbyte;
	int rxbyte;
	for(w=0; w<20; w++){
		txbyte = msequence_generate_symbol(tx_ms,8);
		rxbyte = msequence_generate_symbol(rx_ms,8);
		printf("%d %d\n", txbyte, rxbyte);
	}
	return 1;
}

//The node will transmit noise using the ce1.txt cognitive engine and scenario sc1.txt
//sc1.txt will be set to have a very low SNR
if(noise==1){
	verbose = 0;
	struct CognitiveEngine ce = CreateCognitiveEngine();
	const char* cestring = "ce1.txt";
	readCEConfigFile(&ce, (char *)cestring, 0);
	struct Scenario sc = CreateScenario();
	const char* scstring = "sc1.txt";
	readScConfigFile(&sc, (char *)scstring, 0);
    unsigned char * p = NULL;   // default subcarrier allocation
    if (verbose) 
        printf("Using ofdmtxrx\n");
    ofdmtxrx txcvr(ce.numSubcarriers, ce.CPLen, ce.taperLen, p, rxCallback, (void*) &rxCBs);

    txcvr.set_tx_freq(ce.frequency);
    txcvr.set_tx_rate(ce.bandwidth);
    txcvr.set_tx_gain_soft(ce.txgain_dB);
    txcvr.set_tx_gain_uhd(ce.uhd_txgain_dB);


    int i = 0;

	//The transmission of the interferer will have a header of all 23's and a random payload
	//Before transmission each frame will be heavily distorted by the sc1.txt scenario
    for (i=0; i<8; i++)
        header[i] = 23;

    for (i=0; i<(signed int)ce.payloadLen; i++)
        payload[i] = (unsigned char)msequence_generate_symbol(tx_ms,8);
    // Set Modulation Scheme
    if (verbose) printf("Modulation scheme: %s\n", ce.modScheme);
    modulation_scheme ms = convertModScheme(ce.modScheme, &ce.bitsPerSym);

    // Set inner forward error correction scheme
    if (verbose) printf("Inner FEC: ");
    fec_scheme fec0 = convertFECScheme(ce.innerFEC, verbose);

    // Set outer forward error correction scheme
    if (verbose) printf("Outer FEC: ");
    fec_scheme fec1 = convertFECScheme(ce.outerFEC, verbose);
	
	while(true){

        //txcvr.transmit_packet(header, payload, ce.payloadLen, ms, fec0, fec1);
        // Replace with txcvr methods that allow access to samples:
        txcvr.assemble_frame(header, payload, ce.payloadLen, ms, fec0, fec1);
        int isLastSymbol = 0;
        while(!isLastSymbol)
        {
            isLastSymbol = txcvr.write_symbol();
            enactScenarioBasebandTx(txcvr.fgbuffer, txcvr.fgbuffer_len, ce, sc);
            txcvr.transmit_symbol();
        }
        txcvr.end_transmit_frame();
	}	
}

return 0;
}
