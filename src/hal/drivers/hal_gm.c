#include <rtapi_pci.h>
#include <rtapi_io.h>

#include "rtapi.h"		// RTAPI realtime OS API
#include "rtapi_app.h"		// RTAPI realtime module decls
#include "hal.h"		// HAL public API decls
#include "gm.h"			// Hardware dependent defines
#include "rtapi_math.h"

// Module information.
MODULE_AUTHOR("Bence Kovacs");
MODULE_DESCRIPTION("Driver for General Mechatronics 6-Axis Motion Control Card for EMC HAL");
MODULE_LICENSE("GPL");

typedef struct { //encoder_t
    // Pins
	hal_bit_t			*reset;
	hal_s32_t			*counts;
	hal_float_t			*position;
	hal_float_t			*velocity;
	hal_s32_t			*rawcounts;
	hal_bit_t			*index_enable;
    
    // Parameters
	hal_bit_t			counter_mode;
	hal_bit_t			index_mode;
	hal_bit_t			index_invert;
	hal_u32_t			counts_per_rev;
	hal_float_t			position_scale;
	hal_float_t			min_speed_estimate;

    // Private data
	hal_s32_t			raw_offset;
	hal_s32_t			index_offset;
	hal_s32_t			last_index_latch;
	hal_bit_t			first_index;
	hal_bit_t			module_exist;
} encoder_t;

typedef struct { //switches_t
    // Pins.
	hal_bit_t			*home;
	hal_bit_t			*homeNot;

	hal_bit_t			*posLimSwIn;
	hal_bit_t			*posLimSwInNot;
	hal_bit_t			*negLimSwIn;
	hal_bit_t			*negLimSwInNot;    
} switches_t;

typedef struct { //estop_t
    // Pins.
	hal_bit_t			*in;
	hal_bit_t			*inNot;  
} estop_t;

typedef struct { //gpio_t
    // Pins.
	hal_bit_t			*in;
	hal_bit_t			*inNot;
	hal_bit_t			*out;
	hal_bit_t			isOut;
	hal_bit_t			invertOut;
} gpio_t;

typedef struct { //RS485_8output_t
    // Pins.
	hal_bit_t			*out_0;
	hal_bit_t			*out_1;
	hal_bit_t			*out_2;
	hal_bit_t			*out_3;
	hal_bit_t			*out_4;
	hal_bit_t			*out_5;
	hal_bit_t			*out_6;
	hal_bit_t			*out_7;
    // Parameters
	hal_bit_t			invertOut_0;
	hal_bit_t			invertOut_1;
	hal_bit_t			invertOut_2;
	hal_bit_t			invertOut_3;
	hal_bit_t			invertOut_4;
	hal_bit_t			invertOut_5;
	hal_bit_t			invertOut_6;
	hal_bit_t			invertOut_7;
} RS485_8output_t;

typedef struct { //RS485_8input_t
    // Pins.
	hal_bit_t			*in_0;
	hal_bit_t			*inNot_0;
	hal_bit_t			*in_1;
	hal_bit_t			*inNot_1;
	hal_bit_t			*in_2;
	hal_bit_t			*inNot_2;
	hal_bit_t			*in_3;
	hal_bit_t			*inNot_3;
	hal_bit_t			*in_4;
	hal_bit_t			*inNot_4;
	hal_bit_t			*in_5;
	hal_bit_t			*inNot_5;
	hal_bit_t			*in_6;
	hal_bit_t			*inNot_6;
	hal_bit_t			*in_7;
	hal_bit_t			*inNot_7;
} RS485_8input_t;

typedef struct { //RS485_DacAdc_t
    // Pins.
	hal_float_t			*DAC_0;
	hal_float_t			*DAC_1;
	hal_float_t			*DAC_2;
	hal_float_t			*DAC_3;

	hal_bit_t			*dac_0_enable;
	hal_bit_t			*dac_1_enable;
	hal_bit_t			*dac_2_enable;
	hal_bit_t			*dac_3_enable;

	hal_float_t			*ADC_0;
	hal_float_t			*ADC_1;
	hal_float_t			*ADC_2;
	hal_float_t			*ADC_3;
	hal_float_t			*ADC_4;
	hal_float_t			*ADC_5;
	hal_float_t			*ADC_6;
	hal_float_t			*ADC_7;

    // Parameters.
	hal_float_t			DAC_0_offset;
	hal_float_t			DAC_1_offset;
	hal_float_t			DAC_2_offset;
	hal_float_t			DAC_3_offset;
    
	hal_float_t			DAC_0_min;
	hal_float_t			DAC_1_min;
	hal_float_t			DAC_2_min;
	hal_float_t			DAC_3_min;
    
	hal_float_t			DAC_0_max;
	hal_float_t			DAC_1_max;
	hal_float_t			DAC_2_max;
	hal_float_t			DAC_3_max;
	
	hal_float_t			ADC_0_offset;
	hal_float_t			ADC_1_offset;
	hal_float_t			ADC_2_offset;
	hal_float_t			ADC_3_offset;
	hal_float_t			ADC_4_offset;
	hal_float_t			ADC_5_offset;
	hal_float_t			ADC_6_offset;
	hal_float_t			ADC_7_offset;
    
	hal_float_t			ADC_0_scale;
	hal_float_t			ADC_1_scale;
	hal_float_t			ADC_2_scale;
	hal_float_t			ADC_3_scale;
	hal_float_t			ADC_4_scale;
	hal_float_t			ADC_5_scale;
	hal_float_t			ADC_6_scale;
	hal_float_t			ADC_7_scale;
} RS485_DacAdc_t;

typedef struct { //RS485_TeachPad_t
    // Pins.
	//6 ADC channel
	hal_float_t			*ADC_0;
	hal_float_t			*ADC_1;
	hal_float_t			*ADC_2;
	hal_float_t			*ADC_3;
	hal_float_t			*ADC_4;
	hal_float_t			*ADC_5;
	//8 digital input
	hal_bit_t			*in_0;
	hal_bit_t			*inNot_0;
	hal_bit_t			*in_1;
	hal_bit_t			*inNot_1;
	hal_bit_t			*in_2;
	hal_bit_t			*inNot_2;
	hal_bit_t			*in_3;
	hal_bit_t			*inNot_3;
	hal_bit_t			*in_4;
	hal_bit_t			*inNot_4;
	hal_bit_t			*in_5;
	hal_bit_t			*inNot_5;
	hal_bit_t			*in_6;
	hal_bit_t			*inNot_6;
	hal_bit_t			*in_7;
	hal_bit_t			*inNot_7;	
	//encoder
	hal_bit_t			*enc_reset;
	hal_s32_t			*enc_counts;
	hal_float_t			*enc_position;
	hal_s32_t			*enc_rawcounts;

    // Parameters
	//6 ADC channels
	hal_float_t			ADC_0_offset;
	hal_float_t			ADC_1_offset;
	hal_float_t			ADC_2_offset;
	hal_float_t			ADC_3_offset;
	hal_float_t			ADC_4_offset;
	hal_float_t			ADC_5_offset;
	hal_float_t			ADC_0_scale;
	hal_float_t			ADC_1_scale;
	hal_float_t			ADC_2_scale;
	hal_float_t			ADC_3_scale;
	hal_float_t			ADC_4_scale;
	hal_float_t			ADC_5_scale;
	//encoder
	hal_float_t			enc_position_scale;
	
    // Private data
       //encoder
       hal_s32_t			enc_raw_offset;

} RS485_TeachPad_t;

typedef struct { //RS485_mgr_t
	hal_u32_t			ID[16];
	hal_u32_t			BYTES_TO_WRITE[16];
	hal_u32_t			BYTES_TO_READ[16];
} RS485_mgr_t;

typedef struct { //axisdac_t
    // Pins.
	hal_float_t				*value;
	hal_bit_t				*enable;
 
    // Parameters.
	hal_float_t				min;
	hal_float_t				max;
	hal_float_t				offset;
	
	hal_bit_t				invert_serial;
} axisdac_t;

typedef struct { //stepgen_t
    // Pins.
	hal_float_t		*position_cmd;
	hal_float_t		*velocity_cmd;
	hal_float_t		*position_fb;
	hal_s32_t		*count_fb;	
	hal_bit_t		*enable;
	
    // Parameters
	hal_u32_t		step_type; //0: StepDir, 1: UpDown, 2: Quadrature
	hal_bit_t		control_type; //0: position, 1: velocity	
	hal_u32_t		steplen;
	hal_u32_t		stepspace;
	hal_u32_t		dirdelay;
	hal_float_t		maxaccel;
	hal_float_t		maxvel;
	hal_bit_t		polarity_A;
	hal_bit_t		polarity_B;	
	hal_float_t		position_scale;
	
    //Saved Parameters
	hal_u32_t		curr_steplen;
	hal_u32_t		curr_stepspace;
	hal_u32_t		curr_dirdelay;
	hal_float_t		curr_maxaccel;
	hal_float_t		curr_maxvel;	
	hal_float_t		curr_position_scale;
	
    // Private data
	hal_u32_t		stepgen_fb_offset;
	hal_float_t		old_pos_cmd;
	hal_float_t		max_dv;
	hal_float_t		old_vel;
	hal_float_t		steprate_scale;
} stepgen_t;

typedef struct { //CardMgr_t
    // Pins.
	hal_bit_t		*cardEnable;
	hal_bit_t		*power_enable;
	hal_bit_t		*power_fault;
	hal_bit_t		*watchdog_expired;
	
    // Parameters	
	hal_bit_t		watchdog_enable;
	hal_u32_t		watchdog_timeout_ns;
    
    // Private data
	hal_u32_t		card_control_reg;
	hal_bit_t		disable;
	hal_u32_t		dbg_PCI_counter_last;
	hal_u32_t		cntr;
} cardMgr_t;

typedef struct { //CAN_GM_t
      //Pins
	hal_bit_t		*enable;
	hal_float_t		*position_cmd;
	hal_float_t		*position_fb;
	
      //Parameters
	hal_float_t		position_scale;

} CAN_GM_t;

typedef struct { //CANmsg_t
	hal_bit_t		Ext; //0: Standrad ID, 1: Extended ID
	hal_u32_t		ID;
	hal_u32_t		data[8];
	hal_u32_t		DLC;
	hal_bit_t		RTR;
}CANmsg_t;

typedef struct { //gm_device_t
    // Card relatad data
	card			*pCard;
	int			boardID;  //Sequential nr of cards,  0  -  MAX_GM_DEVICES-1
	int 			cardID;   //Version of the card and its modules

    // Driver related data
	switches_t		switches[6];
	gpio_t			gpio[32];
	estop_t			estop[2];
	
	RS485_mgr_t		RS485_mgr;
	RS485_8input_t		RS485_8input[16];
	RS485_8output_t		RS485_8output[16];
	RS485_DacAdc_t		RS485_DacAdc[16];
	RS485_TeachPad_t	RS485_TeachPad[16];
	
	CAN_GM_t		CAN_GM[6];
	
	stepgen_t		stepgen[6];
	hal_u32_t		stepgen_status;
	axisdac_t		axisdac[6];
	encoder_t		encoder[6];
	
	cardMgr_t		cardMgr;
		
	hal_u32_t		period_ns;
	hal_float_t		period_s;
	hal_float_t		rec_period_s;
} gm_device_t;
   
typedef struct { //gm_driver_t
    int					comp_id;
    gm_device_t				*device[MAX_GM_DEVICES];
} gm_driver_t;

static gm_driver_t				driver;

static int                      num_boards = 0;
static int                      failed_errno = 0; // errno of last failed registration

static struct
rtapi_pci_device_id gm_pci_tbl[] = {
    // GM PCI Card
    {
        .vendor = PLX_VENDOR_ID,
        .device = GM_DEVICE_ID,
        .subvendor = PLX_VENDOR_ID,
        .subdevice = GM_SUBDEVICE_ID_1,
    },
 
    {
        .vendor = PLX_VENDOR_ID,
        .device = GM_DEVICE_ID,
        .subvendor = PLX_VENDOR_ID,
        .subdevice = GM_SUBDEVICE_ID_2,
    },

    {0,},
};

//////////////////////////////////////////////////////////////////////////////
//                          Function prototypes                             //
//////////////////////////////////////////////////////////////////////////////

//Export Pins, Parameters and Functions
  static int ExportFunctions(void *arg, int comp_id, int boardId);
  static int ExportEncoder(void *arg, int comp_id, int version);
  static int ExportStepgen(void *arg, int comp_id, int version);
  static int ExportDAC(void *arg, int comp_id, int version);
  static int ExportRS485(void *arg, int comp_id, int version);
  static int ExportCAN(void *arg, int comp_id, int version);
  static int ExportMixed(void *arg, int comp_id);

//PCI driver misc functions
  static int gm_pci_probe(struct rtapi_pci_dev *dev, const struct rtapi_pci_device_id *id);
  static void gm_pci_remove(struct rtapi_pci_dev *dev);

//Methods exported to HAL
  static void read(void *arg, long period);
  static void write(void *arg, long period);
  static void RS485(void *arg, long period);

//Private methods
  //StepGens
  static void stepgen(void *arg, long period);
  static void stepgenControl(void *arg, long period, unsigned int i);
  static void stepgenCheckParameters(void *arg, long period, unsigned int channel);
  //RS485
  static unsigned int RS485_CheckChecksum(hal_u32_t* data, hal_u32_t length);
  static unsigned int RS485_CalcChecksum(hal_u32_t* data, hal_u32_t length);
  static void RS485_OrderDataRead(hal_u32_t* dataIn32, hal_u32_t* dataOut8, hal_u32_t length);
  static void RS485_OrderDataWrite(hal_u32_t* dataIn8, hal_u32_t* dataOut32, hal_u32_t length);
  //Encoders
  static void encoder(void *arg, long period); 
  //CAN
  static void GM_CAN_SERVO(void *arg);
  static void CAN_SendDataFrame(void *arg, CANmsg_t *Msg);
  static void CAN_ReceiveDataFrame(void *arg, CANmsg_t *Msg);
#ifdef CANOPEN
  static void CAN_Reset(void *arg);
  static void CAN_SetBaud(void *arg, hal_u32_t Baud);
#endif
  static int CAN_ReadStatus(void *arg, hal_u32_t *RxCnt, hal_u32_t *TxCnt);
  //Card management
  static void card_mgr(void *arg, long period);

//////////////////////////////////////////////////////////////////////////////
//                          PCI driver functions                            //
//////////////////////////////////////////////////////////////////////////////


static int
gm_pci_probe(struct rtapi_pci_dev *dev, const struct rtapi_pci_device_id *id)
{
        (void)id;
        int			error=0;
	card			*pCard = NULL;
	gm_device_t		*pDevice;


        if (num_boards >= MAX_GM_DEVICES) {
          rtapi_print_msg(RTAPI_MSG_ERR,"skipping AnyIO board at %s, this driver can only handle %d\n", rtapi_pci_name(dev), MAX_GM_DEVICES);
          return -EINVAL;
        }

        // NOTE: this enables the board's BARs -- this fixes the Arty bug
        if (rtapi_pci_enable_device(dev)) {
          rtapi_print_msg(RTAPI_MSG_ERR,"skipping AnyIO board at %s, failed to enable PCI device\n", rtapi_pci_name(dev));
          return failed_errno = -ENODEV;
        }

	// Allocate memory for device object.
	pDevice = hal_malloc(sizeof(gm_device_t));

	if (pDevice == 0) {
	  rtapi_print_msg(RTAPI_MSG_ERR, "General Mechatronics: ERROR: hal_malloc() failed.\n");
	  hal_exit(driver.comp_id);
	  return(-ENOMEM);
	}

	// Save pointer to device object.
	driver.device[num_boards] = pDevice;

	// Map card into memory.
	pCard = (card *)rtapi_pci_ioremap_bar(dev, 5);
	rtapi_print_msg(RTAPI_MSG_INFO, "General Mechatronics: Card address @ %p, Len = %d.\n", pCard, (int)rtapi_pci_resource_len(dev, 5));

	// Initialize device.
	pDevice->pCard = pCard;

	// Give board id for the card, increasing from 0
	pDevice->boardID = num_boards++;
	
	//Check card ID
	pDevice->cardID = pCard->cardID;
	rtapi_print_msg(RTAPI_MSG_INFO, "General Mechatronics: Card ID: 0x%X.\n", pDevice->cardID);
	
	if ( (pDevice->cardID & IDmask_card) != cardVersion1 ) {
	  rtapi_print_msg(RTAPI_MSG_ERR, "General Mechatronics: ERROR, unknown card detected.\nPlease, download the latest driver.\n");
	  hal_exit(driver.comp_id);
	  return(-ENODEV);
	}

	// Export and init pins, parameters, and functions
	rtapi_set_msg_level(RTAPI_MSG_WARN);
	pDevice->cardMgr.disable = 0; //Enable pointers of not presented modules will be referenced to this variable
	pDevice->period_ns	= 0; 
	
	error = ExportEncoder(pDevice, driver.comp_id, pDevice->cardID & IDmask_encoder);
	if(error == 0) error = ExportStepgen(pDevice, driver.comp_id, pDevice->cardID & IDmask_stepgen);
	if(error == 0) error = ExportDAC(pDevice, driver.comp_id, pDevice->cardID & IDmask_dac);
	if(error == 0) error = ExportRS485(pDevice, driver.comp_id, pDevice->cardID & IDmask_rs485);
	if(error == 0) error = ExportCAN(pDevice, driver.comp_id, pDevice->cardID & IDmask_can);
	if(error == 0) error = ExportMixed(pDevice, driver.comp_id);
	if(error == 0) error = ExportFunctions(pDevice, driver.comp_id, pDevice->boardID);
	
	pDevice->cardMgr.card_control_reg = 0;
	
	rtapi_set_msg_level(RTAPI_MSG_ALL);

	if(error){
	  rtapi_print_msg(RTAPI_MSG_ERR, "General Mechatronics: Error exporting pins and parameters.\n");
	  hal_exit(driver.comp_id);
	  return -EINVAL;
	}

	return 0;
}

static void
gm_pci_remove(struct rtapi_pci_dev *dev)
{
        int i;
        gm_device_t		*pDevice;

        for(i = 0; i < MAX_GM_DEVICES; i++){
			
        if((pDevice = driver.device[i]) != NULL)
        {
          // turn off all
          pDevice->pCard->card_control_reg = (hal_s32_t) 0;
				
          // Unmap card
          rtapi_iounmap((void *)(pDevice->pCard));
          rtapi_pci_disable_device(dev);
          rtapi_pci_set_drvdata(dev, NULL);
        }
    }
}


static struct
rtapi_pci_driver gm_pci_driver = {
	.name = "hal_gm",
	.id_table = gm_pci_tbl,
	.probe = gm_pci_probe,
	.remove = gm_pci_remove,
};

//////////////////////////////////////////////////////////////////////////////
//                     RTAPI main and exit functions                        //
//////////////////////////////////////////////////////////////////////////////

int
rtapi_app_main(void)
{
	int 			r = 0;
	int			msgLevel, i = 0;

	msgLevel = rtapi_get_msg_level();
	rtapi_set_msg_level(RTAPI_MSG_ALL);
	rtapi_print_msg(RTAPI_MSG_INFO, "General Mechatronics: Driver version 1.1.3 loading...\n");

	// Connect to the HAL.
	driver.comp_id = hal_init("hal_gm");
	if (driver.comp_id < 0) {
          rtapi_print_msg(RTAPI_MSG_ERR, "General Mechatronics: ERROR: hal_init() failed.\n");
          return(-EINVAL);
    	}

    	for(i = 0; i < MAX_GM_DEVICES; i++){
	  driver.device[i] = NULL;
    	}

	r = rtapi_pci_register_driver(&gm_pci_driver);

	if (r != 0) {
          rtapi_print_msg(RTAPI_MSG_ERR,"error registering PCI driver\n");
          hal_exit(driver.comp_id);
          return r;
        }

        if(failed_errno) {
          // at least one card registration failed
          hal_exit(driver.comp_id);
          rtapi_pci_unregister_driver(&gm_pci_driver);
          return failed_errno;
        }

        if(num_boards == 0) {
          // no cards were detected
          hal_exit(driver.comp_id);
          rtapi_pci_unregister_driver(&gm_pci_driver);
          return -ENODEV;
        }

    	if(num_boards == 0){
          // No card detected
	  rtapi_print_msg(RTAPI_MSG_WARN, "General Mechatronics: No General Mechatronics card detected :(. \n");
	  hal_exit(driver.comp_id);
	  return -ENODEV;
    	}

    	hal_ready(driver.comp_id);
    	rtapi_set_msg_level(msgLevel);

    	return(0);
}

void
rtapi_app_exit(void)
{
	rtapi_pci_unregister_driver(&gm_pci_driver);
    	hal_exit(driver.comp_id);
}

//////////////////////////////////////////////////////////////////////////////
//                 Export Pins, Parameters and Functions                    //
//////////////////////////////////////////////////////////////////////////////

static int
ExportEncoder(void *arg, int comp_id, int version)
{
	int	i, error=0, boardId;
    	gm_device_t	*device = (gm_device_t *)arg;	
	card	*pCard = device->pCard;
	boardId = device->boardID;
	
	
 	//Export pins and parameters for encoder
	switch (version)
	{
	  case notPresented:
	    for(i=0;i<6;i++)
	      {
		device->encoder[i].module_exist = 0;
	      }	  
	    rtapi_print_msg(RTAPI_MSG_WARN, "General Mechatronics: No encoder module available in this version of the Card.\n");
	    break;
	  case encoderVersion1:
	      for(i=0;i<6;i++)
	      {
	      //Export Pins
		if(error == 0) error = hal_pin_bit_newf(HAL_IN, &(device->encoder[i].reset), comp_id, "gm.%1d.encoder.%1d.reset", boardId, i);
		if(error == 0) error = hal_pin_s32_newf(HAL_OUT, &(device->encoder[i].counts), comp_id, "gm.%1d.encoder.%1d.counts", boardId, i);
		if(error == 0) error = hal_pin_float_newf(HAL_OUT, &(device->encoder[i].position), comp_id, "gm.%1d.encoder.%1d.position", boardId, i);
		if(error == 0) error = hal_pin_float_newf(HAL_OUT, &(device->encoder[i].velocity), comp_id, "gm.%1d.encoder.%1d.velocity", boardId, i);
		if(error == 0) error = hal_pin_s32_newf(HAL_OUT, &(device->encoder[i].rawcounts), comp_id, "gm.%1d.encoder.%1d.rawcounts", boardId, i);
		if(error == 0) error = hal_pin_bit_newf(HAL_IO, &(device->encoder[i].index_enable), comp_id, "gm.%1d.encoder.%1d.index-enable", boardId, i);
			
	      //Export Parameters
		if(error == 0) error = hal_param_bit_newf(HAL_RW, &(device->encoder[i].counter_mode), comp_id, "gm.%1d.encoder.%1d.counter-mode", boardId, i);
		if(error == 0) error = hal_param_bit_newf(HAL_RW, &(device->encoder[i].index_mode), comp_id, "gm.%1d.encoder.%1d.index-mode", boardId, i);
		if(error == 0) error = hal_param_bit_newf(HAL_RW, &(device->encoder[i].index_invert), comp_id, "gm.%1d.encoder.%1d.index-invert", boardId, i);
		if(error == 0) error = hal_param_u32_newf(HAL_RW, &(device->encoder[i].counts_per_rev), comp_id, "gm.%1d.encoder.%1d.counts-per-rev", boardId, i);
		if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->encoder[i].position_scale), comp_id, "gm.%1d.encoder.%1d.position-scale", boardId, i);
		if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->encoder[i].min_speed_estimate), comp_id, "gm.%1d.encoder.%1d.min-speed-estimate", boardId, i);
		
	      //Init parameters
		device->encoder[i].raw_offset = pCard->ENC_counter[i];
		device->encoder[i].index_offset = 0;
		device->encoder[i].last_index_latch = pCard->ENC_index_latch[i];
		device->encoder[i].first_index = 1;
		device->encoder[i].module_exist = 1;
	      }
	    break;
	  default:
	    rtapi_print_msg(RTAPI_MSG_ERR, "General Mechatronics: ERROR, unknown encoder version.\nPlease, download the latest driver.\n");
	}
	return error;
}

static int
ExportStepgen(void *arg, int comp_id, int version)
{
	int	i, error=0, boardId;
	gm_device_t	*device = (gm_device_t *)arg;
	card	*pCard = device->pCard;
	boardId = device->boardID;
		
 	//Export pins and parameters for step generator
	switch (version)
	{
	  case notPresented:
	    for(i=0;i<6;i++)
	      {
		device->stepgen[i].enable = &(device->cardMgr.disable); //Set enable pointers to a 0 value variable
		device->stepgen[i].position_cmd = &(device->stepgen[i].old_pos_cmd);
	      }
	      rtapi_print_msg(RTAPI_MSG_WARN, "General Mechatronics: No stepgen module available in this version of the Card.\n");
	  break;
	  case stepgenVersion1:
	   device->stepgen_status=0;
	   pCard->StepGen_status = 0;
	  //Export pins and parameters
	   for(i = 0; i < 6; i++)
	    {	
		//Export Pins
		if(error == 0) error = hal_pin_float_newf(HAL_IN, &(device->stepgen[i].position_cmd), comp_id, "gm.%1d.stepgen.%1d.position-cmd", boardId, i);
		if(error == 0) error = hal_pin_float_newf(HAL_OUT, &(device->stepgen[i].position_fb), comp_id, "gm.%1d.stepgen.%1d.position-fb", boardId, i);
		if(error == 0) error = hal_pin_float_newf(HAL_IN, &(device->stepgen[i].velocity_cmd), comp_id, "gm.%1d.stepgen.%1d.velocity-cmd", boardId, i);
		if(error == 0) error = hal_pin_s32_newf(HAL_OUT, &(device->stepgen[i].count_fb), comp_id, "gm.%1d.stepgen.%1d.count-fb", boardId, i);		
		if(error == 0) error = hal_pin_bit_newf(HAL_IN, &(device->stepgen[i].enable), comp_id, "gm.%1d.stepgen.%1d.enable", boardId, i);

		//Export Parameters.
		if(error == 0) error = hal_param_bit_newf(HAL_RW, &(device->stepgen[i].control_type), comp_id, "gm.%1d.stepgen.%1d.control-type", boardId, i); //0: StepDir, 1: UpDown, 2: Quadrature
		if(error == 0) error = hal_param_u32_newf(HAL_RW, &(device->stepgen[i].step_type), comp_id, "gm.%1d.stepgen.%1d.step-type", boardId, i); //0: position, 1: velocity		
		if(error == 0) error = hal_param_u32_newf(HAL_RW, &(device->stepgen[i].steplen), comp_id, "gm.%1d.stepgen.%1d.steplen", boardId, i);
		if(error == 0) error = hal_param_u32_newf(HAL_RW, &(device->stepgen[i].stepspace), comp_id, "gm.%1d.stepgen.%1d.stepspace", boardId, i);
		if(error == 0) error = hal_param_u32_newf(HAL_RW, &(device->stepgen[i].dirdelay), comp_id, "gm.%1d.stepgen.%1d.dirdelay", boardId, i);	
		if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->stepgen[i].maxaccel), comp_id, "gm.%1d.stepgen.%1d.maxaccel", boardId, i);
		if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->stepgen[i].maxvel), comp_id, "gm.%1d.stepgen.%1d.maxvel", boardId, i);
		if(error == 0) error = hal_param_bit_newf(HAL_RW, &(device->stepgen[i].polarity_A), comp_id, "gm.%1d.stepgen.%1d.invert-step1", boardId, i);
		if(error == 0) error = hal_param_bit_newf(HAL_RW, &(device->stepgen[i].polarity_B), comp_id, "gm.%1d.stepgen.%1d.invert-step2", boardId, i);
		if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->stepgen[i].position_scale), comp_id, "gm.%1d.stepgen.%1d.position-scale", boardId, i);
                if(error != 0) break;

		//Init parameters
		device->stepgen[i].curr_steplen		= 0;
		device->stepgen[i].curr_stepspace	= 0;
		device->stepgen[i].curr_dirdelay	= 0;
		device->stepgen[i].curr_maxaccel	= 0.0;
		device->stepgen[i].curr_maxvel		= 0.0;	
		device->stepgen[i].curr_position_scale 	= 1.0;
		device->stepgen[i].steprate_scale 	= 30 / 1000000000.0 * 4294967296.0;
		device->stepgen[i].stepgen_fb_offset 	= pCard->StepGen_fb[i];
	
		//Init FPGA registers
		pCard->StepGen_time_params[i] 	= 0;
		pCard->StepGen_steprate[i] 	= 0;
	    }
	  break;
	  default:
	    rtapi_print_msg(RTAPI_MSG_ERR, "General Mechatronics: ERROR, unknown stepgen version.\nPlease, download the latest driver.\n");
	    error = -1;
	}
return error;
}

static int
ExportDAC(void *arg, int comp_id, int version)
{
	int	i, error=0, boardId;
	gm_device_t	*device = (gm_device_t *)arg;
	card	*pCard = device->pCard;
	boardId = device->boardID;
		
	//Export pins and parameters for DAC
	switch (version)
	{
	  case notPresented:
	    for(i=0;i<6;i++)
	      {
		device->axisdac[i].enable = &(device->cardMgr.disable); //Set enable pointers to a 0 value variable
	      }
	      rtapi_print_msg(RTAPI_MSG_WARN, "General Mechatronics: No DAC module available in this version of the Card.\n");
	    break;
	  case dacVersion1:
	    for(i=0;i<6;i++)
	      {
		device->axisdac[i].enable = &(device->cardMgr.disable); //Set enable pointers to a 0 value variable
	      }
              rtapi_print_msg(RTAPI_MSG_WARN, "General Mechatronics: This card supports DAC ver.1 only, which is no longer produced. No DAC pins will be exported to HAL. If you need DAC, contact to bence.kovacs@generalmechatronics.com for firmware upgrade.\n");
	    break;
	  case dacVersion2:
	      for(i=0;i<6;i++)
	      {
		//Export Pins
		if(error == 0) error = hal_pin_bit_newf(HAL_IN, &(device->axisdac[i].enable), comp_id, "gm.%1d.dac.%1d.enable", boardId, i);
		if(error == 0) error = hal_pin_float_newf(HAL_IN, &(device->axisdac[i].value), comp_id, "gm.%1d.dac.%1d.value", boardId, i);

		//Export Parameters.
		if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->axisdac[i].min), comp_id, "gm.%1d.dac.%1d.low-limit", boardId, i);
		if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->axisdac[i].max), comp_id, "gm.%1d.dac.%1d.high-limit", boardId, i);
		if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->axisdac[i].offset), comp_id, "gm.%1d.dac.%1d.offset", boardId, i);
		if(error == 0) error = hal_param_bit_newf(HAL_RW, &(device->axisdac[i].invert_serial), comp_id, "gm.%1d.dac.%1d.invert-serial", boardId, i);
		
		//Init Parameters
		device->axisdac[i].max = 10;
		device->axisdac[i].min = -10;
		device->axisdac[i].offset = 0;
		device->axisdac[i].invert_serial = 0;
		
		//Init FPGA regs
		pCard->DAC_0 = 0x1FFF1FFF;
		pCard->DAC_1 = 0x1FFF1FFF;
		pCard->DAC_2 = 0x1FFF1FFF;
	      }
	    break;
	  default:
	    rtapi_print_msg(RTAPI_MSG_ERR, "General Mechatronics: ERROR, unknown axis DAC version.\nPlease, download the latest driver.\n");
	    error = -1;
	}
	return error;
}

static int
ExportRS485(void *arg, int comp_id, int version)
{
  
	int	i, error=0, boardId, temp;
	gm_device_t	*device = (gm_device_t *)arg;
	card	*pCard = device->pCard;
	boardId = device->boardID;
		
 	//Export pins and parameters for connected RS485 modules
	switch (version)
	{
	  case notPresented:
	    rtapi_print_msg(RTAPI_MSG_WARN, "General Mechatronics: No RS485 module available in this version of the Card.\n");
	    break;
	  case rs485Version1:
	    //READ IDs of connected modules
	    for(i=0; i<8; i++)
	    {
		temp=(hal_u32_t)pCard->moduleId[i];
		 
		if(((temp & 0xff)^0xaa) == ((temp & 0xff00)>>8)) 
		 {
		  device-> RS485_mgr.ID[2*i]=(temp >> 8) & 0xff;
		 }
		else device-> RS485_mgr.ID[2*i] = 0;
		 
		if(((temp & 0xff0000)^0xaa0000) == ((temp & 0xff000000)>>8))
		 {
		  device-> RS485_mgr.ID[2*i+1]=(temp & 0xff000000)>>24;
		 }
		else device-> RS485_mgr.ID[2*i+1]=0;
	    }
	
	    for(i = 0; i < 16; i++)  
	    {	
		switch (device-> RS485_mgr.ID[i])
		{
		 case 0:
		  break;	  
		 case RS485MODUL_ID_8INPUT:	    
			device-> RS485_mgr.BYTES_TO_WRITE[i]=0;
			device-> RS485_mgr.BYTES_TO_READ[i]=2;	//1 data byte + 1 Checksum
			
			if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->RS485_8input[i].in_0), comp_id, "gm.%1d.rs485.%02d.in-0", boardId, i);
			if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->RS485_8input[i].inNot_0), comp_id, "gm.%1d.rs485.%02d.in-not-0", boardId, i);
			if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->RS485_8input[i].in_1), comp_id, "gm.%1d.rs485.%02d.in-1", boardId, i);
			if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->RS485_8input[i].inNot_1), comp_id, "gm.%1d.rs485.%02d.in-not-1", boardId, i);
			if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->RS485_8input[i].in_2), comp_id, "gm.%1d.rs485.%02d.in-2", boardId, i);
			if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->RS485_8input[i].inNot_2), comp_id, "gm.%1d.rs485.%02d.in-not-2", boardId, i);
			if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->RS485_8input[i].in_3), comp_id, "gm.%1d.rs485.%02d.in-3", boardId, i);
			if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->RS485_8input[i].inNot_3), comp_id, "gm.%1d.rs485.%02d.in-not-3", boardId, i);
			if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->RS485_8input[i].in_4), comp_id, "gm.%1d.rs485.%02d.in-4", boardId, i);
			if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->RS485_8input[i].inNot_4), comp_id, "gm.%1d.rs485.%02d.in-not-4", boardId, i);
			if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->RS485_8input[i].in_5), comp_id, "gm.%1d.rs485.%02d.in-5", boardId, i);
			if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->RS485_8input[i].inNot_5), comp_id, "gm.%1d.rs485.%02d.in-not-5", boardId, i);
			if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->RS485_8input[i].in_6), comp_id, "gm.%1d.rs485.%02d.in-6", boardId, i);
			if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->RS485_8input[i].inNot_6), comp_id, "gm.%1d.rs485.%02d.in-not-6", boardId, i);
			if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->RS485_8input[i].in_7), comp_id, "gm.%1d.rs485.%02d.in-7", boardId, i);
			if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->RS485_8input[i].inNot_7), comp_id, "gm.%1d.rs485.%02d.in-not-7", boardId, i);	
		  break;		  
		  case RS485MODUL_ID_8OUTPUT:
			device-> RS485_mgr.BYTES_TO_WRITE[i]=2; // 1 data byte + 1 Checksum
			device-> RS485_mgr.BYTES_TO_READ[i]=0;
			
		    	if(error == 0) error = hal_pin_bit_newf(HAL_IN, &(device->RS485_8output[i].out_0), comp_id, "gm.%1d.rs485.%02d.relay-0", boardId, i);
			if(error == 0) error = hal_pin_bit_newf(HAL_IN, &(device->RS485_8output[i].out_1), comp_id, "gm.%1d.rs485.%02d.relay-1", boardId, i);
			if(error == 0) error = hal_pin_bit_newf(HAL_IN, &(device->RS485_8output[i].out_2), comp_id, "gm.%1d.rs485.%02d.relay-2", boardId, i);
			if(error == 0) error = hal_pin_bit_newf(HAL_IN, &(device->RS485_8output[i].out_3), comp_id, "gm.%1d.rs485.%02d.relay-3", boardId, i);
			if(error == 0) error = hal_pin_bit_newf(HAL_IN, &(device->RS485_8output[i].out_4), comp_id, "gm.%1d.rs485.%02d.relay-4", boardId, i);
			if(error == 0) error = hal_pin_bit_newf(HAL_IN, &(device->RS485_8output[i].out_5), comp_id, "gm.%1d.rs485.%02d.relay-5", boardId, i);
			if(error == 0) error = hal_pin_bit_newf(HAL_IN, &(device->RS485_8output[i].out_6), comp_id, "gm.%1d.rs485.%02d.relay-6", boardId, i);
			if(error == 0) error = hal_pin_bit_newf(HAL_IN, &(device->RS485_8output[i].out_7), comp_id, "gm.%1d.rs485.%02d.relay-7", boardId, i);
			
		    	if(error == 0) error = hal_param_bit_newf(HAL_RW, &(device->RS485_8output[i].invertOut_0), comp_id, "gm.%1d.rs485.%02d.invert-relay-0", boardId, i);
			if(error == 0) error = hal_param_bit_newf(HAL_RW, &(device->RS485_8output[i].invertOut_1), comp_id, "gm.%1d.rs485.%02d.invert-relay-1", boardId, i);
			if(error == 0) error = hal_param_bit_newf(HAL_RW, &(device->RS485_8output[i].invertOut_2), comp_id, "gm.%1d.rs485.%02d.invert-relay-2", boardId, i);
			if(error == 0) error = hal_param_bit_newf(HAL_RW, &(device->RS485_8output[i].invertOut_3), comp_id, "gm.%1d.rs485.%02d.invert-relay-3", boardId, i);
			if(error == 0) error = hal_param_bit_newf(HAL_RW, &(device->RS485_8output[i].invertOut_4), comp_id, "gm.%1d.rs485.%02d.invert-relay-4", boardId, i);
			if(error == 0) error = hal_param_bit_newf(HAL_RW, &(device->RS485_8output[i].invertOut_5), comp_id, "gm.%1d.rs485.%02d.invert-relay-5", boardId, i);
			if(error == 0) error = hal_param_bit_newf(HAL_RW, &(device->RS485_8output[i].invertOut_6), comp_id, "gm.%1d.rs485.%02d.invert-relay-6", boardId, i);
			if(error == 0) error = hal_param_bit_newf(HAL_RW, &(device->RS485_8output[i].invertOut_7), comp_id, "gm.%1d.rs485.%02d.invert-relay-7", boardId, i);

		  break;
		  case RS485MODUL_ID_DACADC:
			device-> RS485_mgr.BYTES_TO_WRITE[i]=5; // 8 data byte + 1 Checksum
			device-> RS485_mgr.BYTES_TO_READ[i]=9; 
		      
			if(error == 0) error = hal_pin_float_newf(HAL_IN, &(device->RS485_DacAdc[i].DAC_0), comp_id, "gm.%1d.rs485.%02d.dac-0", boardId, i);
			if(error == 0) error = hal_pin_float_newf(HAL_IN, &(device->RS485_DacAdc[i].DAC_1), comp_id, "gm.%1d.rs485.%02d.dac-1", boardId, i);
			if(error == 0) error = hal_pin_float_newf(HAL_IN, &(device->RS485_DacAdc[i].DAC_2), comp_id, "gm.%1d.rs485.%02d.dac-2", boardId, i);
			if(error == 0) error = hal_pin_float_newf(HAL_IN, &(device->RS485_DacAdc[i].DAC_3), comp_id, "gm.%1d.rs485.%02d.dac-3", boardId, i);
			if(error == 0) error = hal_pin_bit_newf(HAL_IN, &(device->RS485_DacAdc[i].dac_0_enable), comp_id, "gm.%1d.rs485.%02d.dac-enable-0", boardId, i);
			if(error == 0) error = hal_pin_bit_newf(HAL_IN, &(device->RS485_DacAdc[i].dac_1_enable), comp_id, "gm.%1d.rs485.%02d.dac-enable-1", boardId, i);
			if(error == 0) error = hal_pin_bit_newf(HAL_IN, &(device->RS485_DacAdc[i].dac_2_enable), comp_id, "gm.%1d.rs485.%02d.dac-enable-2", boardId, i);
			if(error == 0) error = hal_pin_bit_newf(HAL_IN, &(device->RS485_DacAdc[i].dac_3_enable), comp_id, "gm.%1d.rs485.%02d.dac-enable-3", boardId, i);
			if(error == 0) error = hal_pin_float_newf(HAL_OUT, &(device->RS485_DacAdc[i].ADC_0), comp_id, "gm.%1d.rs485.%02d.adc-0", boardId, i);
			if(error == 0) error = hal_pin_float_newf(HAL_OUT, &(device->RS485_DacAdc[i].ADC_1), comp_id, "gm.%1d.rs485.%02d.adc-1", boardId, i);      
			if(error == 0) error = hal_pin_float_newf(HAL_OUT, &(device->RS485_DacAdc[i].ADC_2), comp_id, "gm.%1d.rs485.%02d.adc-2", boardId, i);      
			if(error == 0) error = hal_pin_float_newf(HAL_OUT, &(device->RS485_DacAdc[i].ADC_3), comp_id, "gm.%1d.rs485.%02d.adc-3", boardId, i);        
			if(error == 0) error = hal_pin_float_newf(HAL_OUT, &(device->RS485_DacAdc[i].ADC_4), comp_id, "gm.%1d.rs485.%02d.adc-4", boardId, i);
			if(error == 0) error = hal_pin_float_newf(HAL_OUT, &(device->RS485_DacAdc[i].ADC_5), comp_id, "gm.%1d.rs485.%02d.adc-5", boardId, i);      
			if(error == 0) error = hal_pin_float_newf(HAL_OUT, &(device->RS485_DacAdc[i].ADC_6), comp_id, "gm.%1d.rs485.%02d.adc-6", boardId, i);      
			if(error == 0) error = hal_pin_float_newf(HAL_OUT, &(device->RS485_DacAdc[i].ADC_7), comp_id, "gm.%1d.rs485.%02d.adc-7", boardId, i);      
			if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->RS485_DacAdc[i].DAC_0_offset), comp_id, "gm.%1d.rs485.%02d.dac-offset-0", boardId, i);
			if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->RS485_DacAdc[i].DAC_1_offset), comp_id, "gm.%1d.rs485.%02d.dac-offset-1", boardId, i);
			if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->RS485_DacAdc[i].DAC_2_offset), comp_id, "gm.%1d.rs485.%02d.dac-offset-2", boardId, i);
			if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->RS485_DacAdc[i].DAC_3_offset), comp_id, "gm.%1d.rs485.%02d.dac-offset-3", boardId, i);	      
			if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->RS485_DacAdc[i].DAC_0_max), comp_id, "gm.%1d.rs485.%02d.dac-high-limit-0", boardId, i);
			if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->RS485_DacAdc[i].DAC_1_max), comp_id, "gm.%1d.rs485.%02d.dac-high-limit-1", boardId, i);
			if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->RS485_DacAdc[i].DAC_2_max), comp_id, "gm.%1d.rs485.%02d.dac-high-limit-2", boardId, i);
			if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->RS485_DacAdc[i].DAC_3_max), comp_id, "gm.%1d.rs485.%02d.dac-high-limit-3", boardId, i);
			if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->RS485_DacAdc[i].DAC_0_min), comp_id, "gm.%1d.rs485.%02d.dac-low-limit-0", boardId, i);
			if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->RS485_DacAdc[i].DAC_1_min), comp_id, "gm.%1d.rs485.%02d.dac-low-limit-1", boardId, i);
			if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->RS485_DacAdc[i].DAC_2_min), comp_id, "gm.%1d.rs485.%02d.dac-low-limit-2", boardId, i);
			if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->RS485_DacAdc[i].DAC_3_min), comp_id, "gm.%1d.rs485.%02d.dac-low-limit-3", boardId, i);
			if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->RS485_DacAdc[i].ADC_0_offset), comp_id, "gm.%1d.rs485.%02d.adc-offset-0", boardId, i);
			if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->RS485_DacAdc[i].ADC_1_offset), comp_id, "gm.%1d.rs485.%02d.adc-offset-1", boardId, i);
			if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->RS485_DacAdc[i].ADC_2_offset), comp_id, "gm.%1d.rs485.%02d.adc-offset-2", boardId, i);
			if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->RS485_DacAdc[i].ADC_3_offset), comp_id, "gm.%1d.rs485.%02d.adc-offset-3", boardId, i);
			if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->RS485_DacAdc[i].ADC_4_offset), comp_id, "gm.%1d.rs485.%02d.adc-offset-4", boardId, i);
			if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->RS485_DacAdc[i].ADC_5_offset), comp_id, "gm.%1d.rs485.%02d.adc-offset-5", boardId, i);
			if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->RS485_DacAdc[i].ADC_6_offset), comp_id, "gm.%1d.rs485.%02d.adc-offset-6", boardId, i);
			if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->RS485_DacAdc[i].ADC_7_offset), comp_id, "gm.%1d.rs485.%02d.adc-offset-7", boardId, i);
			if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->RS485_DacAdc[i].ADC_0_scale), comp_id, "gm.%1d.rs485.%02d.adc-scale-0", boardId, i);
			if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->RS485_DacAdc[i].ADC_1_scale), comp_id, "gm.%1d.rs485.%02d.adc-scale-1", boardId, i);
			if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->RS485_DacAdc[i].ADC_2_scale), comp_id, "gm.%1d.rs485.%02d.adc-scale-2", boardId, i);
			if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->RS485_DacAdc[i].ADC_3_scale), comp_id, "gm.%1d.rs485.%02d.adc-scale-3", boardId, i);
			if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->RS485_DacAdc[i].ADC_4_scale), comp_id, "gm.%1d.rs485.%02d.adc-scale-4", boardId, i);
			if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->RS485_DacAdc[i].ADC_5_scale), comp_id, "gm.%1d.rs485.%02d.adc-scale-5", boardId, i);
			if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->RS485_DacAdc[i].ADC_6_scale), comp_id, "gm.%1d.rs485.%02d.adc-scale-6", boardId, i);
			if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->RS485_DacAdc[i].ADC_7_scale), comp_id, "gm.%1d.rs485.%02d.adc-scale-7", boardId, i);	
			
			device->RS485_DacAdc[i].DAC_0_max = 10;
			device->RS485_DacAdc[i].DAC_0_min = -10;
			device->RS485_DacAdc[i].DAC_1_max = 10;
			device->RS485_DacAdc[i].DAC_1_min = -10;
			device->RS485_DacAdc[i].DAC_2_max = 10;
			device->RS485_DacAdc[i].DAC_2_min = -10;
			device->RS485_DacAdc[i].DAC_3_max = 10;
			device->RS485_DacAdc[i].DAC_3_min = -10;
			device->RS485_DacAdc[i].ADC_0_scale = 1;
			device->RS485_DacAdc[i].ADC_1_scale = 1;
			device->RS485_DacAdc[i].ADC_2_scale = 1;
			device->RS485_DacAdc[i].ADC_3_scale = 1;
			device->RS485_DacAdc[i].ADC_4_scale = 1;
			device->RS485_DacAdc[i].ADC_5_scale = 1;
			device->RS485_DacAdc[i].ADC_6_scale = 1;
			device->RS485_DacAdc[i].ADC_7_scale = 1;
		    break;		    
		    case RS485MODUL_ID_TEACHPAD:
			device-> RS485_mgr.BYTES_TO_WRITE[i]=0;
			device-> RS485_mgr.BYTES_TO_READ[i]=12;	//1 for 8 digit input, 6 for adc, 4 for encoder + 1 Checksum		
			
			if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->RS485_TeachPad[i].in_0), comp_id, "gm.%1d.rs485.%02d.in-0", boardId, i);
			if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->RS485_TeachPad[i].inNot_0), comp_id, "gm.%1d.rs485.%02d.in-not-0", boardId, i);
			if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->RS485_TeachPad[i].in_1), comp_id, "gm.%1d.rs485.%02d.in-1", boardId, i);
			if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->RS485_TeachPad[i].inNot_1), comp_id, "gm.%1d.rs485.%02d.in-not-1", boardId, i);
			if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->RS485_TeachPad[i].in_2), comp_id, "gm.%1d.rs485.%02d.in-2", boardId, i);
			if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->RS485_TeachPad[i].inNot_2), comp_id, "gm.%1d.rs485.%02d.in-not-2", boardId, i);
			if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->RS485_TeachPad[i].in_3), comp_id, "gm.%1d.rs485.%02d.in-3", boardId, i);
			if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->RS485_TeachPad[i].inNot_3), comp_id, "gm.%1d.rs485.%02d.in-not-3", boardId, i);
			if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->RS485_TeachPad[i].in_4), comp_id, "gm.%1d.rs485.%02d.in-4", boardId, i);
			if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->RS485_TeachPad[i].inNot_4), comp_id, "gm.%1d.rs485.%02d.in-not-4", boardId, i);
			if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->RS485_TeachPad[i].in_5), comp_id, "gm.%1d.rs485.%02d.in-5", boardId, i);
			if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->RS485_TeachPad[i].inNot_5), comp_id, "gm.%1d.rs485.%02d.in-not-5", boardId, i);
			if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->RS485_TeachPad[i].in_6), comp_id, "gm.%1d.rs485.%02d.in-6", boardId, i);
			if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->RS485_TeachPad[i].inNot_6), comp_id, "gm.%1d.rs485.%02d.in-not-6", boardId, i);
			if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->RS485_TeachPad[i].in_7), comp_id, "gm.%1d.rs485.%02d.in-7", boardId, i);
			if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->RS485_TeachPad[i].inNot_7), comp_id, "gm.%1d.rs485.%02d.in-not-7", boardId, i);	

			if(error == 0) error = hal_pin_float_newf(HAL_OUT, &(device->RS485_TeachPad[i].ADC_0), comp_id, "gm.%1d.rs485.%02d.adc-0", boardId, i);
			if(error == 0) error = hal_pin_float_newf(HAL_OUT, &(device->RS485_TeachPad[i].ADC_1), comp_id, "gm.%1d.rs485.%02d.adc-1", boardId, i);      
			if(error == 0) error = hal_pin_float_newf(HAL_OUT, &(device->RS485_TeachPad[i].ADC_2), comp_id, "gm.%1d.rs485.%02d.adc-2", boardId, i);      
			if(error == 0) error = hal_pin_float_newf(HAL_OUT, &(device->RS485_TeachPad[i].ADC_3), comp_id, "gm.%1d.rs485.%02d.adc-3", boardId, i);        
			if(error == 0) error = hal_pin_float_newf(HAL_OUT, &(device->RS485_TeachPad[i].ADC_4), comp_id, "gm.%1d.rs485.%02d.adc-4", boardId, i);
			if(error == 0) error = hal_pin_float_newf(HAL_OUT, &(device->RS485_TeachPad[i].ADC_5), comp_id, "gm.%1d.rs485.%02d.adc-5", boardId, i);  
			
			if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->RS485_TeachPad[i].ADC_0_offset), comp_id, "gm.%1d.rs485.%02d.adc-offset-0", boardId, i);
			if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->RS485_TeachPad[i].ADC_1_offset), comp_id, "gm.%1d.rs485.%02d.adc-offset-1", boardId, i);
			if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->RS485_TeachPad[i].ADC_2_offset), comp_id, "gm.%1d.rs485.%02d.adc-offset-2", boardId, i);
			if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->RS485_TeachPad[i].ADC_3_offset), comp_id, "gm.%1d.rs485.%02d.adc-offset-3", boardId, i);
			if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->RS485_TeachPad[i].ADC_4_offset), comp_id, "gm.%1d.rs485.%02d.adc-offset-4", boardId, i);
			if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->RS485_TeachPad[i].ADC_5_offset), comp_id, "gm.%1d.rs485.%02d.adc-offset-5", boardId, i);
			
			if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->RS485_TeachPad[i].ADC_0_scale), comp_id, "gm.%1d.rs485.%02d.adc-scale-0", boardId, i);
			if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->RS485_TeachPad[i].ADC_1_scale), comp_id, "gm.%1d.rs485.%02d.adc-scale-1", boardId, i);
			if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->RS485_TeachPad[i].ADC_2_scale), comp_id, "gm.%1d.rs485.%02d.adc-scale-2", boardId, i);
			if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->RS485_TeachPad[i].ADC_3_scale), comp_id, "gm.%1d.rs485.%02d.adc-scale-3", boardId, i);
			if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->RS485_TeachPad[i].ADC_4_scale), comp_id, "gm.%1d.rs485.%02d.adc-scale-4", boardId, i);
			if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->RS485_TeachPad[i].ADC_5_scale), comp_id, "gm.%1d.rs485.%02d.adc-scale-5", boardId, i);
			
			if(error == 0) error = hal_pin_bit_newf(HAL_IN, &(device->RS485_TeachPad[i].enc_reset), comp_id, "gm.%1d.rs485.%02d.enc-reset", boardId, i);
			if(error == 0) error = hal_pin_s32_newf(HAL_OUT, &(device->RS485_TeachPad[i].enc_counts), comp_id, "gm.%1d.rs485.%02d.enc-counts", boardId, i);
			if(error == 0) error = hal_pin_float_newf(HAL_OUT, &(device->RS485_TeachPad[i].enc_position), comp_id, "gm.%1d.rs485.%02d.enc-position", boardId, i);
			if(error == 0) error = hal_pin_s32_newf(HAL_OUT, &(device->RS485_TeachPad[i].enc_rawcounts), comp_id, "gm.%1d.rs485.%02d.enc-rawcounts", boardId, i);
			if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->RS485_TeachPad[i].enc_position_scale), comp_id, "gm.%1d.rs485.%02d.enc-position-scale", boardId, i);
			
			device->RS485_TeachPad[i].ADC_0_scale = 1;
			device->RS485_TeachPad[i].ADC_1_scale = 1;
			device->RS485_TeachPad[i].ADC_2_scale = 1;
			device->RS485_TeachPad[i].ADC_3_scale = 1;
			device->RS485_TeachPad[i].ADC_4_scale = 1;
			device->RS485_TeachPad[i].ADC_5_scale = 1;
			
			device->RS485_TeachPad[i].ADC_0_offset = 0;
			device->RS485_TeachPad[i].ADC_1_offset = 0;
			device->RS485_TeachPad[i].ADC_2_offset = 0;
			device->RS485_TeachPad[i].ADC_3_offset = 0;
			device->RS485_TeachPad[i].ADC_4_offset = 0;
			device->RS485_TeachPad[i].ADC_5_offset = 0;
			
			device->RS485_TeachPad[i].enc_raw_offset = 0;
			device->RS485_TeachPad[i].enc_position_scale=1;
		    break;
		    default:
			rtapi_print_msg(RTAPI_MSG_ERR, "General Mechatronics: ERROR, unknown rs485 module type.\nPlease, download the latest driver.\n");
		}
	    } 
	    break;
	  default:
	    rtapi_print_msg(RTAPI_MSG_ERR, "General Mechatronics: ERROR, unknown rs485 version.\nPlease, download the latest driver.\n");
	}
	return error;
}

static int
ExportCAN(void *arg, int comp_id, int version)
{
	int	i, error=0, boardId;
	gm_device_t	*device = (gm_device_t *)arg;
	boardId = device->boardID;
	
 	//Export pins and parameters for encoder
	switch (version)
	{
	  case notPresented:
	    for(i=0;i<6;i++)
	      {
		device->CAN_GM[i].enable = &(device->cardMgr.disable); //Set enable pointers to a 0 value variable
	      }	 
	    rtapi_print_msg(RTAPI_MSG_WARN, "General Mechatronics: No CAN module available in this version of the Card.\n");
	    break;
	  case canVersion1:
	    
	    //Export Pins and Parameters for CAN GM Servo Controllers
	    for(i=0;i<6;i++)
	    {
	       //Export Pins
		  if(error == 0) error = hal_pin_bit_newf(HAL_IN, &(device->CAN_GM[i].enable), comp_id, "gm.%1d.can-gm.%1d.enable", boardId, i);
		  if(error == 0) error = hal_pin_float_newf(HAL_IN, &(device->CAN_GM[i].position_cmd), comp_id, "gm.%1d.can-gm.%1d.position-cmd", boardId, i);
		  if(error == 0) error = hal_pin_float_newf(HAL_OUT, &(device->CAN_GM[i].position_fb), comp_id, "gm.%1d.can-gm.%1d.position-fb", boardId, i);
		  
	       //Export Parameters
		  if(error == 0) error = hal_param_float_newf(HAL_RW, &(device->CAN_GM[i].position_scale), comp_id, "gm.%1d.can-gm.%1d.position-scale", boardId, i);
	    }
	    
	    //Export Pins and Parameters for CANopen Servo Controllers
	      //In development...

	    break;
	  default:
	    rtapi_print_msg(RTAPI_MSG_ERR, "General Mechatronics: ERROR, unknown encoder version.\nPlease, download the latest driver.\n");
	}
	return error;
}

static int
ExportMixed(void *arg, int comp_id)
{
	int	i, j, error=0, boardId;
	gm_device_t	*device = (gm_device_t *)arg;
	boardId = device->boardID;

	//Homing and End switches pins and parameters
	for(i = 0; i < 6; i++)
	{	
		// Pins
		if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->switches[i].home), comp_id, "gm.%1d.axis.%1d.home-sw-in", boardId, i);
		if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->switches[i].homeNot), comp_id, "gm.%1d.axis.%1d.home-sw-in-not", boardId, i);
		if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->switches[i].posLimSwIn), comp_id, "gm.%1d.axis.%1d.pos-lim-sw-in", boardId, i);
		if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->switches[i].posLimSwInNot), comp_id, "gm.%1d.axis.%1d.pos-lim-sw-in-not", boardId, i);
		if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->switches[i].negLimSwIn), comp_id, "gm.%1d.axis.%1d.neg-lim-sw-in", boardId, i);
		if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->switches[i].negLimSwInNot), comp_id, "gm.%1d.axis.%1d.neg-lim-sw-in-not", boardId, i);
	}

	//Power bridge Fault and Error pins
	if(error == 0) error = hal_pin_bit_newf(HAL_IN, &(device->cardMgr.power_enable), comp_id, "gm.%1d.power-enable", boardId);
	if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->cardMgr.power_fault), comp_id, "gm.%1d.power-fault", boardId);
	
	//Watchdog pins and parameters
	if(error == 0) error = hal_param_bit_newf(HAL_RW, &(device->cardMgr.watchdog_enable), comp_id, "gm.%1d.watchdog-enable", boardId);
	if(error == 0) error = hal_param_u32_newf(HAL_RW, &(device->cardMgr.watchdog_timeout_ns), comp_id, "gm.%1d.watchdog-timeout-ns", boardId);
	if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->cardMgr.watchdog_expired), comp_id, "gm.%1d.watchdog-expired", boardId);
		      
	//Export pins and parameters for parallel IOs
	for(i=0;i<4;i++)
	{
		for(j=0;j<8;j++)
		{
			if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->gpio[i*8+j].in), comp_id, "gm.%1d.gpio.%1d.in-%1d", boardId, i, j);	
			if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->gpio[i*8+j].inNot), comp_id, "gm.%1d.gpio.%1d.in-not-%1d", boardId, i, j);		
			if(error == 0) error = hal_pin_bit_newf(HAL_IN, &(device->gpio[i*8+j].out), comp_id, "gm.%1d.gpio.%1d.out-%1d", boardId, i, j);
			if(error == 0) error = hal_param_bit_newf(HAL_RW, &(device->gpio[i*8+j].isOut), comp_id, "gm.%1d.gpio.%1d.is-out-%1d", boardId, i, j);
			if(error == 0) error = hal_param_bit_newf(HAL_RW, &(device->gpio[i*8+j].invertOut), comp_id, "gm.%1d.gpio.%1d.invert-out-%1d", boardId, i, j);
		}
	}

	//Export pins and parameters for estops
	for(i=0;i<2;i++)
	{
		if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->estop[i].in), comp_id, "gm.%1d.estop.%1d.in", boardId, i);			
		if(error == 0) error = hal_pin_bit_newf(HAL_OUT, &(device->estop[i].inNot), comp_id, "gm.%1d.estop.%1d.in-not", boardId, i);
	}
	
	device->cardMgr.cntr = 0; //Counter of executing card_mgr() function

	return error;
}

static int ExportFunctions(void *arg, int comp_id, int boardId)
{
	int error;
	gm_device_t	*device = (gm_device_t *)arg;

	error = hal_export_functf(write, device, 1, 0, comp_id, "gm.%d.write", boardId);

	if(error == 0)
	{
		error = hal_export_functf(read, device, 1, 0, comp_id, "gm.%d.read", boardId);
	}
	
	if(error == 0)
	{
		error = hal_export_functf(RS485, device, 1, 0, comp_id, "gm.%d.RS485", boardId);
	}

    	return error;
}

//////////////////////////////////////////////////////////////////////////////
//                        Read/Write HAL functions                          //
//////////////////////////////////////////////////////////////////////////////

static void
read(void *arg, long period)
{
    	gm_device_t	*device = (gm_device_t *)arg;
    	card	*pCard = device->pCard;
	unsigned int i;
	hal_u32_t temp;

      //basic card functionality: watchdog, switches, estop
	card_mgr(arg, period);

      //read parallel IOs
	temp=pCard->gpio;
    	for(i = 0; i < 32; i++)
	{
		*(device->gpio[i].in) = (hal_bit_t)((temp & ((unsigned int) 1 << i)) == 0 ? 0 : 1);
		*(device->gpio[i].inNot) = (hal_bit_t)((temp & ((unsigned int) 1 << i)) == 0 ? 1 : 0);
	}

      //Read Encoders
	encoder(arg, period);
}

static void
write(void *arg, long period)
{
	gm_device_t	*device = (gm_device_t *)arg;
	card	*pCard = device->pCard;

	int		i, temp1=0, temp2=0;
	hal_float_t	DAC[6];
	hal_u32_t	DAC_INTEGER[6];

	//Refresh DAC values
	  for(i=0;i<6;i++)
	  {
		if(*(device->axisdac[i].enable))
		{
			if( (*(device->axisdac[i].value) + device->axisdac[i].offset) > device->axisdac[i].max)
			{
				DAC[i] = device->axisdac[i].max;
			}
			else if (  (*(device->axisdac[i].value) + device->axisdac[i].offset) < device->axisdac[i].min)
			{
				DAC[i] = device->axisdac[i].min;
			}
			else
			{
				DAC[i] =   (*(device->axisdac[i].value) + device->axisdac[i].offset);
			}
		}
		else
		{
			DAC[i] = 0;
		}
		
		DAC[i] = (DAC[i] * 819.15) + 8191.5;
		if(DAC[i] < 0) DAC[i] = 0;
		else if (DAC[i] > 16383) DAC[i] = 16383;

		DAC_INTEGER[i] = (hal_u32_t)(DAC[i]);
		
		if(device->axisdac[i].invert_serial) DAC_INTEGER[i] |= 0x8000;
	  }
	  pCard->DAC_0 = ((DAC_INTEGER[1] & 0xFFFF) << 16) | (DAC_INTEGER[0] & 0xFFFF);
	  pCard->DAC_1 = ((DAC_INTEGER[3] & 0xFFFF) << 16) | (DAC_INTEGER[2] & 0xFFFF);
	  pCard->DAC_2 = ((DAC_INTEGER[5] & 0xFFFF) << 16) | (DAC_INTEGER[4] & 0xFFFF);
 
	//Run step generators
	  stepgen(arg, period);
	  
	//Handle GM CAN Servo Controllers
	  GM_CAN_SERVO(arg);
	  
	//Write parallel IOs
	  temp1 = 0;
	  for(i = 0; i < 32; i++)
	  {
		if(device->gpio[i].isOut) temp1 |= 0x0001 << i;			//write gpio mask: 1=output, 0=input
		if((*(device->gpio[i].out)) ^ (device->gpio[i].invertOut)) temp2 |= 0x0001 << i;	//write outputs
	  }
	  pCard->gpioDir=temp1;
	  pCard->gpio=temp2;
	  
}


//////////////////////////////////////////////////////////////////////////////
//                                   CAN bus                                //
//////////////////////////////////////////////////////////////////////////////

static void
GM_CAN_SERVO(void *arg)
{
	gm_device_t	*device = (gm_device_t *)arg;

	hal_u32_t	Rx_buf_cntr, Tx_buf_cntr;
	hal_u32_t	i, temp=0;
	hal_s32_t	posFb;
	CANmsg_t	CAN_msg;

	//Position references: ID 0x10 - 0x15
	//Position feed back: ID 0x20 - 0x25

      //Do not run, if none of the GM CAN channels are enabled
	for(i=0;i<6;i++){
	  if(*(device->CAN_GM[i].enable) == 1) temp++;
	}
	if(temp == 0) return;

      //Read Buffers status
	CAN_ReadStatus(arg, &Rx_buf_cntr, &Tx_buf_cntr);

      //Read feedbacks
	for(i=0;i<Rx_buf_cntr;i++)
	{
	  CAN_ReceiveDataFrame(arg, &CAN_msg);
	  
	  if((CAN_msg.ID >= 0x20) && (CAN_msg.ID <= 0x25))
	  {
	    if((device->CAN_GM[CAN_msg.ID - 0x20].position_scale<10e-6) && (device->CAN_GM[CAN_msg.ID - 0x20].position_scale>-10e-6))
	    {
	      device->CAN_GM[CAN_msg.ID - 0x20].position_scale = 1;
	    }
	     posFb = (CAN_msg.data[3] << 24) | (CAN_msg.data[2] << 16) | (CAN_msg.data[1] << 8) | CAN_msg.data[0];
	     *(device->CAN_GM[CAN_msg.ID - 0x20].position_fb) = (hal_float_t)posFb / device->CAN_GM[CAN_msg.ID - 0x20].position_scale;
	  }
	}

      //Send reference
	for(i=0;i<6;i++)
	{
	  if(*(device->CAN_GM[i].enable) == 1)
	  {
	    CAN_msg.RTR = 0; //Not a request frame
	    CAN_msg.Ext = 0; //Standard ID
	    CAN_msg.DLC = 4; //4 byte data
	    CAN_msg.ID = 0x10 + i;

	    if((device->CAN_GM[i].position_scale<10e-6) && (device->CAN_GM[i].position_scale>-10e-6))
	    {
	      device->CAN_GM[i].position_scale = 1;
	    }
	    temp = (hal_u32_t)(*(device->CAN_GM[i].position_cmd) * device->CAN_GM[i].position_scale);
	    CAN_msg.data[0] = temp & 0xFF;
	    CAN_msg.data[1] = (temp >> 8) & 0xFF;
	    CAN_msg.data[2] = (temp >> 16) & 0xFF;
	    CAN_msg.data[3] = (temp >> 24) & 0xFF;
            CAN_msg.data[4] = 0;
            CAN_msg.data[5] = 0;
            CAN_msg.data[6] = 0;
            CAN_msg.data[7] = 0;

	    CAN_SendDataFrame(arg, &CAN_msg);
	  }
	}  
}
static int
CAN_ReadStatus(void *arg, hal_u32_t *RxCnt, hal_u32_t *TxCnt)
{
	gm_device_t	*device = (gm_device_t *)arg;
	card	*pCard = device->pCard;
	
	hal_u32_t temp;  
	
	temp = pCard->CAN_status_reg;
	*TxCnt = temp & 0x3FF;
	*RxCnt = (temp >> 10) & 0x3FF;
	
	return (temp >> 20) & 0xFF; //return with MCP2515 IT reg
}

static void
CAN_ReceiveDataFrame(void *arg, CANmsg_t *Msg)
{
	gm_device_t	*device = (gm_device_t *)arg;
	card		*pCard = device->pCard;
	
	hal_u32_t	temp;
	
	temp = pCard->CAN_RX_buffer[0];
	if(temp & 0x80000000)
	{
	  Msg->Ext = 1;
	  Msg->ID = temp & 0x1FFFFFFF;
	}
	else
	{
	  Msg->Ext = 0;
	  Msg->ID = temp;
	}
	
	temp=pCard->CAN_RX_buffer[2];
	Msg->data[0] = (temp>>24) & 0xFF;
	Msg->data[1] = (temp>>16) & 0xFF;
	Msg->data[2] = (temp>>8) & 0xFF;
	Msg->data[3] = (temp) & 0xFF;

	temp=pCard->CAN_RX_buffer[1];
	Msg->data[4] = (temp>>24) & 0xFF;
	Msg->data[5] = (temp>>16) & 0xFF;
	Msg->data[6] = (temp>>8) & 0xFF;
	Msg->data[7] = (temp) & 0xFF;
	
	temp = pCard->CAN_RX_buffer[3];
	Msg->DLC = temp & 0xF;
	if(temp & 0x10) Msg->RTR = 1;
	else Msg->RTR = 0;	
}

static void
CAN_SendDataFrame(void *arg, CANmsg_t *Msg)
{
	gm_device_t	*device = (gm_device_t *)arg;
	card		*pCard = device->pCard;
	
	hal_u32_t ID;
	
	ID = Msg->ID;//MsgID;
	if(Msg->Ext) ID |= 0x80000000;
	pCard->CAN_TX_buffer[0]=ID;
	
	pCard->CAN_TX_buffer[2] = ((Msg->data[0] & 0xFF) << 24) |((Msg->data[1] & 0xFF) << 16) |((Msg->data[2] & 0xFF) << 8) |((Msg->data[3] & 0xFF) << 0);
	    
	if(Msg->DLC > 4) pCard->CAN_TX_buffer[1] = ((Msg->data[4] & 0xFF) << 24) |((Msg->data[5] & 0xFF) << 16) |((Msg->data[6] & 0xFF) << 8) |((Msg->data[7] & 0xFF) << 0);
 
	pCard->CAN_TX_buffer[3] = Msg->DLC;	  	  	
}

#ifdef CANOPEN
//Higher level protocols may need these functions
static void
CAN_Reset(void *arg)
{
	gm_device_t	*device = (gm_device_t *)arg;
	card		*pCard = device->pCard;
	
	pCard->CAN_TX_buffer[3] = 0x81;
}

static void
CAN_SetBaud(void *arg, hal_u32_t Baud)
{
  	gm_device_t		*device = (gm_device_t *)arg;
	card	*pCard = device->pCard;
	
	switch(Baud)
	{
	  case 125:
	    pCard->CAN_TX_buffer[2] = 0x01880700;
	    pCard->CAN_TX_buffer[3] = 0x82;
	    break;
	  
	  case 250:
	    pCard->CAN_TX_buffer[2] = 0x01880300;
	    pCard->CAN_TX_buffer[3] = 0x82;
	    break;
	    
	  case 500:
	    pCard->CAN_TX_buffer[2] = 0x01880100;
	    pCard->CAN_TX_buffer[3] = 0x82;
	    break;
	    
	  case 1000:
	    pCard->CAN_TX_buffer[2] = 0x01880000;
	    pCard->CAN_TX_buffer[3] = 0x82;
	    break;
	    
	  default:
	    rtapi_print_msg(RTAPI_MSG_ERR, "General Mechatronics:Not valid CAN Baud Rate. Supported: 125,250,500 and 1000 kBit/s.\n");  
	}
}
#endif

//////////////////////////////////////////////////////////////////////////////
//                        Card manage functions                             //
//////////////////////////////////////////////////////////////////////////////
static void
card_mgr(void *arg, long period)
{
	gm_device_t		*device = (gm_device_t *)arg;
    	card	*pCard = device->pCard;
	
	hal_u32_t	temp=0, i, j;
	
      //Read card status reg and process data
	temp = pCard->card_status_reg; //This resets watch dog timer
	
	*(device->cardMgr.watchdog_expired) = 	(hal_bit_t)((temp & (0x0001 << 0)) == 0 ? 0 : 1);
	*(device->cardMgr.power_fault) = 	(hal_bit_t)((temp & (0x0001 << 2)) == 0 ? 0 : 1);
	*(device->estop[0].in) = 		(hal_bit_t)((temp & (0x0001 << 3)) == 0 ? 0 : 1);
	*(device->estop[0].inNot) = 		(hal_bit_t)((temp & (0x0001 << 3)) == 0 ? 1 : 0);
	*(device->estop[1].in) = 		(hal_bit_t)((temp & (0x0001 << 4)) == 0 ? 0 : 1);
	*(device->estop[1].inNot) = 		(hal_bit_t)((temp & (0x0001 << 4)) == 0 ? 1 : 0);
	
    	for(i=5, j=0; i<11; i++,j++)
	{
		*(device->switches[j].home) = 		(hal_bit_t)((temp & (0x0001 << i)) == 0 ? 0 : 1);
		*(device->switches[j].homeNot) = 	(hal_bit_t)((temp & (0x0001 << i)) == 0 ? 1 : 0);
	}
	for(j=0;i<17;i++,j++){
		*(device->switches[j].posLimSwIn) = 	(hal_bit_t)((temp & (0x0001 << i)) == 0 ? 0 : 1);
		*(device->switches[j].posLimSwInNot) = 	(hal_bit_t)((temp & (0x0001 << i)) == 0 ? 1 : 0);
	}
	for(j=0;i<23;i++,j++){
		*(device->switches[j].negLimSwIn) = 	(hal_bit_t)((temp & (0x0001 << i)) == 0 ? 0 : 1);
		*(device->switches[j].negLimSwInNot) = 	(hal_bit_t)((temp & (0x0001 << i)) == 0 ? 1 : 0);
	}
	
	
      //Check if change happened in control reg and write control reg if well
	 //  ... Estop_1 | Estop_0 | Pwr_fault | Bus_err | Wdt_err  //Card status read resets wdt
	temp = 1; //EMC run
	if(*(device->cardMgr.power_enable)) temp |= (0x0001 << 1); //power enable
	
	if(device->cardMgr.watchdog_enable) //watchdog timeout in ns*256 unit. 0 if watchdog is disabled
	{
	  if(device->cardMgr.watchdog_timeout_ns < 256) temp |= 0x100;
	  else temp |= (device->cardMgr.watchdog_timeout_ns & 0xFFFFFF00);
	}
	
	if(temp != device->cardMgr.card_control_reg)
	{
	  device->cardMgr.card_control_reg = temp;
	  pCard->card_control_reg = temp;
	}
	
      //First 16 execution of this function measure PCI clk frequency. Result is printed to dmesg.
	if(device->cardMgr.cntr < 17)
	{
	  if(device->cardMgr.cntr == 0) device->cardMgr.dbg_PCI_counter_last = pCard->PCI_clk_counter;
	  else if(device->cardMgr.cntr == 16)
	  {
	      temp = rtapi_get_msg_level();
	      rtapi_set_msg_level(RTAPI_MSG_ALL);
	      rtapi_print_msg(RTAPI_MSG_INFO, "General Mechatronics: PCI clk frequency is %d khz.\n",
		      (int)((hal_float_t)(pCard->PCI_clk_counter - device->cardMgr.dbg_PCI_counter_last)/period*62500));	//Calculate frequency
	      rtapi_set_msg_level(temp);
	  }
	  device->cardMgr.cntr++;
	}	
}

//////////////////////////////////////////////////////////////////////////////
//                               Encoder                                    //
//////////////////////////////////////////////////////////////////////////////
static void
encoder(void *arg, long period)
{
    (void)period;
    gm_device_t		*device = (gm_device_t *)arg;
    card	*pCard = device->pCard;

    int		i;
	hal_s32_t	temp1 = 0, temp2;
	hal_float_t	vel;

	//Update parameters
	for(i=0; i<6; i++)
	{
	   if(device->encoder[i].index_invert == 1) temp1 |= (0x1 << i);
	   if(device->encoder[i].counter_mode == 1) temp1 |= (0x1 << (i+6));
	}
	pCard->ENC_control_reg = temp1;
	
	
	//Read encoders
	for(i = 0; i < 6; i++)
	  if(device->encoder[i].module_exist)
	  {
		temp1 = pCard->ENC_counter[i];
		temp2 = pCard->ENC_index_latch[i];
		
		if(*(device->encoder[i].reset) == 1) //If encoder in reset state
		{
		  device->encoder[i].index_offset = temp1;
		}
		else if(*(device->encoder[i].index_enable) == 1) //If not in reset and index is enabled
		{
		  if (temp2 != device->encoder[i].last_index_latch) //If index pulse come
		  {
		    if(device->encoder[i].index_mode == 0)  //reset counter at index
		    {
		      device->encoder[i].index_offset = temp2;  
		      *(device->encoder[i].index_enable) = 0; //disable index
		    }
		    else	//round counter at index
		    {
		      if(device->encoder[i].first_index == 1)  //Check if not first index
		      {
			device->encoder[i].first_index = 0;
		      }
		      else
		      {
			if(temp2 > (device->encoder[i].last_index_latch + (hal_s32_t)(device->encoder[i].counts_per_rev/4)))
			{
			  device->encoder[i].index_offset -=  device->encoder[i].last_index_latch + device->encoder[i].counts_per_rev - temp2;
			}
			else if(temp2 < (device->encoder[i].last_index_latch - (hal_s32_t)(device->encoder[i].counts_per_rev/4)))
			{
			  device->encoder[i].index_offset -=  device->encoder[i].last_index_latch - device->encoder[i].counts_per_rev - temp2;
			}
			else
			{
			  device->encoder[i].index_offset -=  device->encoder[i].last_index_latch - temp2;
			}
		      }
		    }
		  }
		}
		device->encoder[i].last_index_latch = temp2;
		
		*(device->encoder[i].rawcounts) =  temp1 - device->encoder[i].raw_offset;
		*(device->encoder[i].counts) = *(device->encoder[i].rawcounts) - device->encoder[i].index_offset;
		
		if((device->encoder[i].position_scale < 0.000001) && (device->encoder[i].position_scale > -0.000001))  device->encoder[i].position_scale = 1; //Don't like to divide by 0
		*(device->encoder[i].position) = (hal_float_t) *(device->encoder[i].counts) / device->encoder[i].position_scale;
		
		vel = (hal_float_t) pCard->ENC_period[i];
		if(vel == 0) vel = 1;
		vel = 33333333 / ( vel * device->encoder[i].position_scale); //velocity in position units / s
		
		if(fabs(vel) > device->encoder[i].min_speed_estimate)
		{
		  *(device->encoder[i].velocity) =  vel;
		}
		else
		{
		  *(device->encoder[i].velocity) = 0;
		}
	  }
}

//////////////////////////////////////////////////////////////////////////////
//                               Stepgen                                    //
//////////////////////////////////////////////////////////////////////////////
static void
stepgen(void *arg, long period)
{
    	gm_device_t	*device = (gm_device_t *)arg;
    	card		*pCard = device->pCard;

    	int		i;
	
      //Update stepgen status with enable bits
	for(i=0;i<6;i++)
	{
	  if(*(device->stepgen[i].enable) == 1) device->stepgen_status |= (0x1 << i);	//six stepgens share one status reg, 5 bits for each.
	  else
          {
            device->stepgen_status &= ~(0x1 << i);			//LS bits of 5 bits are the enable bits
            pCard->StepGen_steprate[i] = 0;
          }
	}

      //Check parameter changes, if enabled
	for(i=0;i<6;i++)
	{
	  if(*(device->stepgen[i].enable) == 1)
	  {
	    stepgenCheckParameters(arg, period, i);
	  }
	  
	}
      //Update fpga with status register (enable and parameter bits)
	pCard->StepGen_status = device->stepgen_status;
	 	
      //Run steppers, if enabled
	for(i=0;i<6;i++)
	{
	  if(*(device->stepgen[i].enable) == 1)
	  {
	    stepgenControl(arg, period, i);
	  }
	}

      //update old pos_cmd
	for(i=0;i<6;i++)
	{
	  device->stepgen[i].old_pos_cmd = *(device->stepgen[i].position_cmd);
	}
}

static void
stepgenCheckParameters(void *arg, long period, unsigned int channel)
{
      gm_device_t		*device = (gm_device_t *)arg;
      card	*pCard = device->pCard;
      
      hal_u32_t 	temp1, temp2;
      hal_float_t 	min_period, max_vel;
      
      //If period changed : recalc period related parameters
      if(device->period_ns != period)
      {
	device->period_s = period * 0.000000001;
	device->rec_period_s = 1.0/device->period_s;	
      }
      
      //If position scale changed : update steprate_scale
      if(device->stepgen[channel].curr_position_scale != device->stepgen[channel].position_scale)
      {	
	//30 ns is circley time of CLK, 1/10e9 is ns -sec conversion, 42.. is 2^32 because steprate is 32 bit
	device->stepgen[channel].steprate_scale = device->stepgen[channel].position_scale * 30.0 / 1000000000.0 * 4294967296.0;
      }
            
      //If steplen, stepspace, position_scale or max_vel changed  :  update max_vel
      if((device->stepgen[channel].steplen != device->stepgen[channel].curr_steplen) || (device->stepgen[channel].stepspace != device->stepgen[channel].curr_stepspace) || 
	  (device->stepgen[channel].maxvel != device->stepgen[channel].curr_maxvel) || (device->stepgen[channel].curr_position_scale != device->stepgen[channel].position_scale))
      {
	min_period = (device->stepgen[channel].steplen + device->stepgen[channel].stepspace) * 0.000000001;
	max_vel = 1/((hal_float_t)min_period * fabs(device->stepgen[channel].position_scale));
	
	if(device->stepgen[channel].maxvel <= 0)
	{
	  device->stepgen[channel].maxvel = max_vel;
	}
	else
	{
	  if(max_vel < device->stepgen[channel].maxvel) //if stepgen given velocity is higher, then what is possible with step timing parameters
	  {
	    device->stepgen[channel].maxvel = max_vel;
	    rtapi_print_msg(RTAPI_MSG_ERR, "GM: stepgen.%d.maxvel can not be reached with given 'steplen' and 'stepspace' parameters.\n", channel);
	  }
	}
      }
      
      //If steplen or dirdelay changed : update FPGA time parameter regs
      if((device->stepgen[channel].steplen != device->stepgen[channel].curr_steplen) || (device->stepgen[channel].dirdelay != device->stepgen[channel].curr_dirdelay))
      {	
	//Init time constants, send them to PCI 
	temp1= (device->stepgen[channel].steplen <= 1900000) ? (device->stepgen[channel].steplen/30) : 63333;
	temp2= (device->stepgen[channel].dirdelay <= 1900000) ? (device->stepgen[channel].dirdelay/30) : 63333;
	
	if((device->stepgen[channel].steplen > 1900000) || (device->stepgen[channel].dirdelay > 1900000))
	{
	  rtapi_print_msg(RTAPI_MSG_ERR, "GM: stepgen: 'steplen' and 'dirdelay' must be lower than 1 900 000 ns.\n");
	}
	pCard->StepGen_time_params[channel] = (temp1 << 16) | (temp2 & 0xFFFF);
      }
      
      //If enable, step_type or polarity bits changed : update fpga status reg
      if (*(device->stepgen[channel].enable) == 1) device->stepgen_status |= (0x1 << channel);	//Bit 0-5 is the enable bit
	else device->stepgen_status &= ~(0x1 << channel);
      if (device->stepgen[channel].step_type == 1) device->stepgen_status |= (0x1 << (channel + 6));	//Bits 6-17 are the step_mode bits
	else device->stepgen_status &= ~(0x1 << (channel + 6));
      if (device->stepgen[channel].step_type == 2) device->stepgen_status |= (0x1 << (channel + 12));	
	else device->stepgen_status &= ~(0x1 << (channel + 12));
      if (device->stepgen[channel].polarity_A == 1) device->stepgen_status |= (0x1 << (channel + 18));	//18-23. bit is polarity of channel A
	else device->stepgen_status &= ~(0x1 << (channel + 18));
      if (device->stepgen[channel].polarity_B == 1) device->stepgen_status |= (0x1 << (channel + 24));	//24-29. bit is polarity of channel B
	else device->stepgen_status &= ~(0x1 << (channel + 24));
	
      pCard->StepGen_status = device->stepgen_status;
      
      //If max_vel, max_accel or period changed : calc max_dv
      if((device->stepgen[channel].maxvel != device->stepgen[channel].curr_maxvel) || (device->stepgen[channel].maxaccel != device->stepgen[channel].curr_maxaccel) || (device->period_ns != period))
      {
	if(device->stepgen[channel].maxaccel <= 1e-20)
	{
	  device->stepgen[channel].maxaccel = device->stepgen[channel].maxvel * device->rec_period_s;
	  device->stepgen[channel].max_dv = device->stepgen[channel].maxvel;
	}
	else
	{
	  device->stepgen[channel].max_dv =  device->stepgen[channel].maxaccel * device->period_s; //max velocity change in position_unit/period^2
	}
	//vel = freq/pos_scale
	//pos=counts/pos_scale
      }
      
      //Update current values
      device->period_ns = period;
      device->stepgen[channel].curr_position_scale = device->stepgen[channel].position_scale;
      device->stepgen[channel].curr_stepspace = device->stepgen[channel].stepspace;
      device->stepgen[channel].curr_maxvel = device->stepgen[channel].maxvel;
      device->stepgen[channel].curr_maxaccel = device->stepgen[channel].maxaccel;
      device->stepgen[channel].curr_steplen = device->stepgen[channel].steplen;
      device->stepgen[channel].curr_dirdelay= device->stepgen[channel].dirdelay;
      
}

static void
stepgenControl(void *arg, long period, unsigned int channel)
{
    (void)period;
    gm_device_t		*device = (gm_device_t *)arg;
    card	*pCard = device->pCard;
	
	hal_s32_t stepgen_fb, stepgen_fb_int, last_count_fb_LS16_bits, last_count_fb_MS16_bits, last_count_fb;	
	hal_float_t	ref_vel = 0, match_acc, match_time, avg_v, est_out, est_cmd, est_err, dp;
	
     //read and count feedbacks
	stepgen_fb = pCard->StepGen_fb[channel];	//pCard->StepGen_Fb[channel] is 16.16 bit fixed point feedback in [step] unit
	stepgen_fb -= device->stepgen[channel].stepgen_fb_offset;
	stepgen_fb_int= stepgen_fb >> 16;		//get integer part of step feedback
	
	last_count_fb = *(device->stepgen[channel].count_fb);
	last_count_fb_LS16_bits = last_count_fb & 0xFFFF;
	last_count_fb_MS16_bits = last_count_fb & 0xFFFF0000;
	
      //Check for 16 bit overflow of stepgen_fb
	if(stepgen_fb_int > last_count_fb_LS16_bits + 32768) //16 bit step counter down overflow  
	{
	  *(device->stepgen[channel].count_fb) = (last_count_fb_MS16_bits + stepgen_fb_int - 65536);
	}
	else if (stepgen_fb_int + 32768 < last_count_fb_LS16_bits) //16 bit step counter up overflow
	{
	   *(device->stepgen[channel].count_fb) = (last_count_fb_MS16_bits + stepgen_fb_int + 65536);
	}
	else //no overflow
	{
	  *(device->stepgen[channel].count_fb) =   (last_count_fb_MS16_bits + stepgen_fb_int);
	}

      //save old position and get new one
	*(device->stepgen[channel].position_fb) = (*(device->stepgen[channel].count_fb) + ((hal_float_t)(stepgen_fb & 0xFFFF))/65536)/device->stepgen[channel].position_scale;	//[pos_unit]
	
      //velocity control is easy
	if(device->stepgen[channel].control_type == 1)
	{
	  ref_vel = *(device->stepgen[channel].velocity_cmd); 
	}
	
      //Position control is more difficult
	/*Position control based on John Kasunich's stepgen hal component.*/
	else if(device->stepgen[channel].control_type == 0)
	{
	//Reference velocity:
	  ref_vel =  (*(device->stepgen[channel].position_cmd) - device->stepgen[channel].old_pos_cmd) * device->rec_period_s;
	  
	  if(ref_vel > device->stepgen[channel].old_vel)
	  {
	    match_acc = device->stepgen[channel].maxaccel;
	  }
	  else
	  {
	   match_acc =  -device->stepgen[channel].maxaccel;
	  }
	  
	  match_time = (ref_vel - device->stepgen[channel].old_vel) / match_acc;
	  
	  avg_v = (ref_vel + device->stepgen[channel].old_vel) * 0.5;
	  
	  est_out = *(device->stepgen[channel].position_fb) + avg_v * match_time;;
	  est_cmd =  *(device->stepgen[channel].position_cmd) + ref_vel * (match_time - 1.5 * device->period_s);	  
	  
	  est_err = est_out - est_cmd;
	  
	  //If we can match velocity in one period	   
	  if(match_time < device->period_s)
	  {
	    if(fabs(est_err) > 0.0001) //position correction
	    {
		ref_vel -= 0.5 * est_err * device->rec_period_s;
	    }
	  }
	  
	  //If we need more periods to reach, then ramp
	  else
	  {
	    //position difference in case of ramping in the opposite direction
	    dp = -2.0 * match_acc * device->period_s * match_time; 	//sum of velocity change

	    //decide which way to ramp
	    if(fabs(est_err + dp*2.0) < fabs(est_err))
	    {
		match_acc = -match_acc;
	    }
	    ref_vel = device->stepgen[channel].old_vel + match_acc * device->period_s;
	  }
  
	}
      //Check max velocity, max acceleration and output baudrate
      
	//Check max velocity
	if(ref_vel > device->stepgen[channel].maxvel) ref_vel = device->stepgen[channel].maxvel;
	else if(ref_vel < -device->stepgen[channel].maxvel) ref_vel = -device->stepgen[channel].maxvel;
	  
	//Check max acceleration
	if((device->stepgen[channel].old_vel-ref_vel) > device->stepgen[channel].max_dv) 
	{
	 ref_vel=device->stepgen[channel].old_vel-device->stepgen[channel].max_dv;
	}
	else if((device->stepgen[channel].old_vel-ref_vel) < -device->stepgen[channel].max_dv)
	{
	 ref_vel=device->stepgen[channel].old_vel+device->stepgen[channel].max_dv;
	}
	//Save old velocity
	device->stepgen[channel].old_vel=ref_vel; 
	//Set steprate
	pCard->StepGen_steprate[channel] = (hal_s32_t)(ref_vel * device->stepgen[channel].steprate_scale);
	
}

//////////////////////////////////////////////////////////////////////////////
//                               RS485                                      //
//////////////////////////////////////////////////////////////////////////////
static void
RS485(void *arg, long period)
{
    (void)period;
	gm_device_t		*device = (gm_device_t *)arg;
	card	*pCard = device->pCard;
	
	unsigned int i, j;
	hal_float_t temp;
        hal_u32_t temp_u32;
        bool data_wr = 0;
        static hal_bit_t failed=0;
	
	//for write function
	hal_u32_t RS485DataIn8[32], RS485DataOut32[8];
	//for read function
	hal_u32_t RS485DataIn32[8], RS485DataOut8[32];

       //Check modules if any of it failed:
       //READ IDs of correctly connected modules and compare it with saved ID-s.
	for(i=0; i<8; i++)
	{
	temp_u32=(hal_u32_t)pCard->moduleId[i];
		 
	  if(((temp_u32 & 0xff)^0xaa) == ((temp_u32 & 0xff00)>>8)) 
	  {
	    if((device-> RS485_mgr.ID[2*i]) != ((temp_u32 >> 8) & 0xff)) 
            {
              //RS485 module fell off, error
              if(failed == 0) //Msg only first time, do not put 100 error msg
              {
                failed=1;
                rtapi_print_msg(RTAPI_MSG_ERR, "GM: ERROR: RS485 module ID:%2d failed.\n", 2*i);
              }
              *(device->cardMgr.power_fault) = 1;
            }
	  }
		 
	  if(((temp_u32 & 0xff0000)^0xaa0000) == ((temp_u32 & 0xff000000)>>8))
	  {
	    if((device-> RS485_mgr.ID[2*i+1]) != ((temp_u32 & 0xff000000)>>24))
            {
              //RS485 module fell off, error
              if(failed == 0) //Msg only first time, do not put 100 error msg
              {
                failed=1; 
                rtapi_print_msg(RTAPI_MSG_ERR, "GM: ERROR: RS485 module ID:%2d failed.\n", 2*i+1);
              }
              *(device->cardMgr.power_fault) = 1;
            }
	  }
        }
  
       //read RS485-s
	for(i=0;i<16;i++) if((device-> RS485_mgr.ID[i] != 0) && (device-> RS485_mgr.BYTES_TO_READ[i] != 0)) //If the module is presented and not write only
	{
	      //Block ram address lookahead support
		if(i != 0) *(&(pCard->serialModulesDataIn[i-1][7])); 
		else *(&(pCard->serialModulesDataOut[15][7]));
	      //Read bytes to RS485DataIn32 array
		for(j=0; j<8; j++) RS485DataIn32[j]= (hal_u32_t)pCard->serialModulesDataIn[i][j];
		
	      //Order data to RS485DataOut8 buffer
		RS485_OrderDataRead(RS485DataIn32, RS485DataOut8, device-> RS485_mgr.BYTES_TO_READ[i]);
	      //Process data if Checksum is OK
		if(RS485_CheckChecksum(RS485DataOut8,  device-> RS485_mgr.BYTES_TO_READ[i]) == 0)
		{
		  switch (device-> RS485_mgr.ID[i])
		  {
		    case RS485MODUL_ID_8INPUT:
			*(device->RS485_8input[i].in_0) = ((hal_bit_t)(RS485DataOut8[0] & 0x1) ? 1 : 0);
			*(device->RS485_8input[i].inNot_0) = (hal_bit_t)(RS485DataOut8[0] & 0x1) ? 0 : 1;
			*(device->RS485_8input[i].in_1) = (hal_bit_t)((RS485DataOut8[0] >> 1) & 0x1) ? 1 : 0;
			*(device->RS485_8input[i].inNot_1) = (hal_bit_t)((RS485DataOut8[0] >> 1) & 0x1) ? 0 : 1;
			*(device->RS485_8input[i].in_2) = (hal_bit_t)((RS485DataOut8[0] >> 2) & 0x1) ? 1 : 0;
			*(device->RS485_8input[i].inNot_2) = (hal_bit_t)((RS485DataOut8[0] >> 2) & 0x1) ? 0 : 1;
			*(device->RS485_8input[i].in_3) = (hal_bit_t)((RS485DataOut8[0] >> 3) & 0x1) ? 1 : 0;
			*(device->RS485_8input[i].inNot_3) = (hal_bit_t)((RS485DataOut8[0] >> 3) & 0x1) ? 0 : 1;
			*(device->RS485_8input[i].in_4) = (hal_bit_t)((RS485DataOut8[0] >> 4) & 0x1) ? 1 : 0;
			*(device->RS485_8input[i].inNot_4) = (hal_bit_t)((RS485DataOut8[0] >> 4) & 0x1) ? 0 : 1;
			*(device->RS485_8input[i].in_5) = (hal_bit_t)((RS485DataOut8[0] >> 5) & 0x1) ? 1 : 0;
			*(device->RS485_8input[i].inNot_5) = (hal_bit_t)((RS485DataOut8[0] >> 5) & 0x1) ? 0 : 1;
			*(device->RS485_8input[i].in_6) = (hal_bit_t)((RS485DataOut8[0] >> 6) & 0x1) ? 1 : 0;
			*(device->RS485_8input[i].inNot_6) = (hal_bit_t)((RS485DataOut8[0] >> 6) & 0x1) ? 0 : 1;
			*(device->RS485_8input[i].in_7) = (hal_bit_t)((RS485DataOut8[0] >> 7) & 0x1) ? 1 : 0;
			*(device->RS485_8input[i].inNot_7) = (hal_bit_t)((RS485DataOut8[0] >> 7) & 0x1) ? 0 : 1;
		      break;
		      
		   case RS485MODUL_ID_DACADC:
			*(device->RS485_DacAdc[i].ADC_0) = (((hal_float_t)RS485DataOut8[0])/25.5-5) * device->RS485_DacAdc[i].ADC_0_scale - device->RS485_DacAdc[i].ADC_0_offset;
			*(device->RS485_DacAdc[i].ADC_1) = (((hal_float_t)RS485DataOut8[1])/25.5-5) * device->RS485_DacAdc[i].ADC_1_scale - device->RS485_DacAdc[i].ADC_1_offset;
			*(device->RS485_DacAdc[i].ADC_2) = (((hal_float_t)RS485DataOut8[2])/25.5-5) * device->RS485_DacAdc[i].ADC_2_scale - device->RS485_DacAdc[i].ADC_2_offset;
			*(device->RS485_DacAdc[i].ADC_3) = (((hal_float_t)RS485DataOut8[3])/25.5-5) * device->RS485_DacAdc[i].ADC_3_scale - device->RS485_DacAdc[i].ADC_3_offset;
			*(device->RS485_DacAdc[i].ADC_4) = (((hal_float_t)RS485DataOut8[4])/25.5-5) * device->RS485_DacAdc[i].ADC_4_scale - device->RS485_DacAdc[i].ADC_4_offset;
			*(device->RS485_DacAdc[i].ADC_5) = (((hal_float_t)RS485DataOut8[5])/25.5-5) * device->RS485_DacAdc[i].ADC_5_scale - device->RS485_DacAdc[i].ADC_5_offset;
			*(device->RS485_DacAdc[i].ADC_6) = (((hal_float_t)RS485DataOut8[6])/25.5-5) * device->RS485_DacAdc[i].ADC_6_scale - device->RS485_DacAdc[i].ADC_6_offset;
			*(device->RS485_DacAdc[i].ADC_7) = (((hal_float_t)RS485DataOut8[7])/25.5-5) * device->RS485_DacAdc[i].ADC_7_scale - device->RS485_DacAdc[i].ADC_7_offset;
			
		      break;
		      
		   case RS485MODUL_ID_TEACHPAD:

			*(device->RS485_TeachPad[i].in_0) = ((hal_bit_t)(RS485DataOut8[0] & 0x1) ? 1 : 0);
			*(device->RS485_TeachPad[i].inNot_0) = (hal_bit_t)(RS485DataOut8[0] & 0x1) ? 0 : 1;
			*(device->RS485_TeachPad[i].in_1) = (hal_bit_t)((RS485DataOut8[0] >> 1) & 0x1) ? 1 : 0;
			*(device->RS485_TeachPad[i].inNot_1) = (hal_bit_t)((RS485DataOut8[0] >> 1) & 0x1) ? 0 : 1;
			*(device->RS485_TeachPad[i].in_2) = (hal_bit_t)((RS485DataOut8[0] >> 2) & 0x1) ? 1 : 0;
			*(device->RS485_TeachPad[i].inNot_2) = (hal_bit_t)((RS485DataOut8[0] >> 2) & 0x1) ? 0 : 1;
			*(device->RS485_TeachPad[i].in_3) = (hal_bit_t)((RS485DataOut8[0] >> 3) & 0x1) ? 1 : 0;
			*(device->RS485_TeachPad[i].inNot_3) = (hal_bit_t)((RS485DataOut8[0] >> 3) & 0x1) ? 0 : 1;
			*(device->RS485_TeachPad[i].in_4) = (hal_bit_t)((RS485DataOut8[0] >> 4) & 0x1) ? 1 : 0;
			*(device->RS485_TeachPad[i].inNot_4) = (hal_bit_t)((RS485DataOut8[0] >> 4) & 0x1) ? 0 : 1;
			*(device->RS485_TeachPad[i].in_5) = (hal_bit_t)((RS485DataOut8[0] >> 5) & 0x1) ? 1 : 0;
			*(device->RS485_TeachPad[i].inNot_5) = (hal_bit_t)((RS485DataOut8[0] >> 5) & 0x1) ? 0 : 1;
			*(device->RS485_TeachPad[i].in_6) = (hal_bit_t)((RS485DataOut8[0] >> 6) & 0x1) ? 1 : 0;
			*(device->RS485_TeachPad[i].inNot_6) = (hal_bit_t)((RS485DataOut8[0] >> 6) & 0x1) ? 0 : 1;
			*(device->RS485_TeachPad[i].in_7) = (hal_bit_t)((RS485DataOut8[0] >> 7) & 0x1) ? 1 : 0;
			*(device->RS485_TeachPad[i].inNot_7) = (hal_bit_t)((RS485DataOut8[0] >> 7) & 0x1) ? 0 : 1;

			*(device->RS485_TeachPad[i].ADC_0) = (hal_float_t)RS485DataOut8[1]/51.2 * device->RS485_TeachPad[i].ADC_0_scale - device->RS485_TeachPad[i].ADC_0_offset;
			*(device->RS485_TeachPad[i].ADC_1) = (hal_float_t)RS485DataOut8[2]/51.2 * device->RS485_TeachPad[i].ADC_1_scale - device->RS485_TeachPad[i].ADC_1_offset;
			*(device->RS485_TeachPad[i].ADC_2) = (hal_float_t)RS485DataOut8[3]/51.2 * device->RS485_TeachPad[i].ADC_2_scale - device->RS485_TeachPad[i].ADC_2_offset;
			*(device->RS485_TeachPad[i].ADC_3) = (hal_float_t)RS485DataOut8[4]/51.2 * device->RS485_TeachPad[i].ADC_3_scale - device->RS485_TeachPad[i].ADC_3_offset;
			*(device->RS485_TeachPad[i].ADC_4) = (hal_float_t)RS485DataOut8[5]/51.2 * device->RS485_TeachPad[i].ADC_4_scale - device->RS485_TeachPad[i].ADC_4_offset;
			*(device->RS485_TeachPad[i].ADC_5) = (hal_float_t)RS485DataOut8[6]/51.2 * device->RS485_TeachPad[i].ADC_5_scale - device->RS485_TeachPad[i].ADC_5_offset;
			
			*(device->RS485_TeachPad[i].enc_rawcounts)= (RS485DataOut8[7] & 0xff) | ((RS485DataOut8[8] & 0xff) << 8) | ((RS485DataOut8[9] & 0xff) << 16) | (RS485DataOut8[10] << 24);
			if(*(device->RS485_TeachPad[i].enc_reset))
			{
			  device->RS485_TeachPad[i].enc_raw_offset = *(device->RS485_TeachPad[i].enc_rawcounts);
			}
			*(device->RS485_TeachPad[i].enc_counts) = *(device->RS485_TeachPad[i].enc_rawcounts) - device->RS485_TeachPad[i].enc_raw_offset;
			
			if((device->RS485_TeachPad[i].enc_position_scale < 0.000001) && (device->RS485_TeachPad[i].enc_position_scale > -0.000001)) device->RS485_TeachPad[i].enc_position_scale=1; //don't divide by 0
			
			*(device->RS485_TeachPad[i].enc_position) = *(device->RS485_TeachPad[i].enc_counts) / device->RS485_TeachPad[i].enc_position_scale;
			
		    default:
		      break;     
		  }
		    
		}
	}

      //Write serial IOs
	for(i=0;i<16;i++) if((device-> RS485_mgr.ID[i] != 0) && (device-> RS485_mgr.BYTES_TO_WRITE[i] != 0)) //If the module is presented and not read only
	{
	  
		switch (device-> RS485_mgr.ID[i])
		{
		    case RS485MODUL_ID_8OUTPUT:
		      RS485DataIn8[0]=((*(device->RS485_8output[i].out_7) ^ (device->RS485_8output[i].invertOut_7)) << 7) |
				      ((*(device->RS485_8output[i].out_6) ^ (device->RS485_8output[i].invertOut_6)) << 6) |
				      ((*(device->RS485_8output[i].out_5) ^ (device->RS485_8output[i].invertOut_5)) << 5) |
				      ((*(device->RS485_8output[i].out_4) ^ (device->RS485_8output[i].invertOut_4)) << 4) |
				      ((*(device->RS485_8output[i].out_3) ^ (device->RS485_8output[i].invertOut_3)) << 3) |
				      ((*(device->RS485_8output[i].out_2) ^ (device->RS485_8output[i].invertOut_2)) << 2) |
				      ((*(device->RS485_8output[i].out_1) ^ (device->RS485_8output[i].invertOut_1)) << 1) |
				      ((*(device->RS485_8output[i].out_0) ^ (device->RS485_8output[i].invertOut_0)) << 0);
		    break;

		    case RS485MODUL_ID_DACADC:
		     //DAC 0 
		      if(*(device->RS485_DacAdc[i].dac_0_enable))
		      {
			temp = *(device->RS485_DacAdc[i].DAC_0)+ device->RS485_DacAdc[i].DAC_0_offset;
			
			if(temp > device->RS485_DacAdc[i].DAC_0_max) {  temp = device->RS485_DacAdc[i].DAC_0_max; }
			else if(temp < device->RS485_DacAdc[i].DAC_0_min) { temp = device->RS485_DacAdc[i].DAC_0_min; }
			
			temp = (temp + 10)*12.8 + 0.5;
		      }
		      else temp=128;
		      
		      if(temp>255) temp=255;
		      else if(temp <0) temp=0;
		      RS485DataIn8[0]= (hal_u32_t)temp;
		    //DAC 1
		      if(*(device->RS485_DacAdc[i].dac_1_enable))
		      {
			temp = *(device->RS485_DacAdc[i].DAC_1)+ device->RS485_DacAdc[i].DAC_1_offset;
			
			if(temp > device->RS485_DacAdc[i].DAC_1_max) {  temp = device->RS485_DacAdc[i].DAC_1_max; }
			else if(temp < device->RS485_DacAdc[i].DAC_1_min) { temp = device->RS485_DacAdc[i].DAC_1_min; }
			
			temp = (temp + 10)*12.8 + 0.5;
                      }
                      else temp=128;

		      if(temp>255) temp=255;
		      else if(temp <0) temp=0;
		      RS485DataIn8[1]= (hal_u32_t)temp;
		      
		    //DAC 2
		      if(*(device->RS485_DacAdc[i].dac_2_enable))
		      {
			temp = *(device->RS485_DacAdc[i].DAC_2)+ device->RS485_DacAdc[i].DAC_2_offset;
			
			if(temp > device->RS485_DacAdc[i].DAC_2_max) {  temp = device->RS485_DacAdc[i].DAC_2_max; }
			else if(temp < device->RS485_DacAdc[i].DAC_2_min) { temp = device->RS485_DacAdc[i].DAC_2_min; }
			
			temp = (temp + 10)*12.8 + 0.5;
                      }
                      else temp=128;

		      if(temp>255) temp=255;
		      else if(temp <0) temp=0;
		      RS485DataIn8[2]= (hal_u32_t)temp;
		      
		    //DAC 3
		      if(*(device->RS485_DacAdc[i].dac_3_enable))
		      {
			temp = *(device->RS485_DacAdc[i].DAC_3)+ device->RS485_DacAdc[i].DAC_3_offset;
			
			if(temp > device->RS485_DacAdc[i].DAC_3_max) {  temp = device->RS485_DacAdc[i].DAC_3_max; }
			else if(temp < device->RS485_DacAdc[i].DAC_3_min) { temp = device->RS485_DacAdc[i].DAC_3_min; }
			
			temp = (temp + 10)*12.8 + 0.5;
                      }
                      else temp=128;

		      if(temp>255) temp=255;
		      else if(temp <0) temp=0;
		      RS485DataIn8[3]= (hal_u32_t)temp;

		    break;		    
		    		      
		    default:
		    break;     
		  }
		  
		//Calc Checksum
		RS485DataIn8[device-> RS485_mgr.BYTES_TO_WRITE[i]-1]=RS485_CalcChecksum(RS485DataIn8, device-> RS485_mgr.BYTES_TO_WRITE[i] - 1);
		//Order 8 bit data to send to 32 bit PCI
		RS485_OrderDataWrite(RS485DataIn8, RS485DataOut32, device-> RS485_mgr.BYTES_TO_WRITE[i]); //order 8 bit bytes to 32 bit words
	      	//Send data      	      
		 for(j=7;(7-j) < ((unsigned int)(device-> RS485_mgr.BYTES_TO_WRITE[i]-1)/4) + 1;j--)
		 {
		    *(&(pCard->serialModulesDataOut[i][j])) = RS485DataOut32[j];
		 }
                 data_wr=1;
	  }
          //Ping RS485 BUS if no RS485 module with output is connected
          if(data_wr == 0) *(&(pCard->serialModulesDataOut[0][0])) = 0;
}


static void
RS485_OrderDataRead(hal_u32_t* dataIn32, hal_u32_t* dataOut8, hal_u32_t length)
{
	int i, j;
	//Order data (received to dataIn in reverse order and shifted) to dataOut[0]-dataOut[length-1]
	if(length > 0 && length < 32)
	for(i=length-1; i>=0; i--)
	{
	   j = length-1-i;
	   dataOut8[i]= (dataIn32[(unsigned int)(j/4)] >> ((j%4)*8)) & 0xff;	
	}
 }
 
static void
RS485_OrderDataWrite(hal_u32_t* dataIn8, hal_u32_t* dataOut32, hal_u32_t length)
{
	unsigned i, j;
	/* Byte order: 
	      RS485DataOut32[0]=0x28293031;
	      RS485DataOut32[1]=0x24252627;
	      RS485DataOut32[2]=0x20212223;
	      RS485DataOut32[3]=0x16171819;
	      
	      RS485DataOut32[4]=0x12131415;
	      RS485DataOut32[5]=0x08091011;
	      RS485DataOut32[6]=0x04050607;
	      RS485DataOut32[7]=0x00010203;
	*/
	
	for(i=0;i<length;i+=4)
	{
	  j=7-(i >> 2); //To which word to write
	  dataOut32[j]=(dataIn8[i+3] & 0xff) | ((dataIn8[i+2] & 0xff) << 8) | ((dataIn8[i+1] & 0xff) << 16) | (dataIn8[i] << 24);
	}
 }
 
static unsigned int
RS485_CheckChecksum(hal_u32_t* data, hal_u32_t length)
{
      unsigned int i=0, tempChecksum=0;
      
      if(length > 0 && length < 32)
      for(i=0; i < length-1; i++)
      {
	   tempChecksum += data[i];
      }
      if((data[i] ^ 0xaa) == (tempChecksum & 0xff)) return 0;

      return -1;
      
}

static unsigned int
RS485_CalcChecksum(hal_u32_t* data, hal_u32_t length)
{
      unsigned int i, tempChecksum=0;
      
      if(length > 0 && length < 32)
      for(i=0; i < length; i++)
      {
	   tempChecksum += data[i];
      }
     return (tempChecksum & 0xff) ^ 0xaa;
      
}

