//
//    Copyright (C) 2011 Andy Pugh, 2016 Boris Skegin
//
//    This program is free software; you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation; either version 2 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERinstTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program; if not, write to the Free Software
//    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
//

#include <rtapi_slab.h>
#include "rtapi.h"
#include "rtapi_string.h"
#include "rtapi_math.h"
#include "hal.h"
#include "hostmot2.h"

// PktUART specific error codes
#include "pktuart_errno.h"


#define MaxTrFrames     (16) // Send counts are written to 16 deep FIFO, burst mode



int hm2_pktuart_parse_md(hostmot2_t *hm2, int md_index) 
{
    // All this function actually does is allocate memory
    // and give the uart modules names. 
    
    
    // 
    // some standard sanity checks
    //
    
    int i, r = -EINVAL;
    hm2_module_descriptor_t *md = &hm2->md[md_index];
    static int last_gtag = -1;

    //The PktUART declares a TX and RX module separately
    //And Rx can currently be v1 or v2
    if (md->gtag == HM2_GTAG_PKTUART_TX
        && !hm2_md_is_consistent(hm2,md_index, 0, 4, 4, 0x000F)
        && !hm2_md_is_consistent(hm2,md_index, 1, 4, 4, 0x000F)
        && !hm2_md_is_consistent(hm2,md_index, 2, 4, 4, 0x000F))  {
        HM2_ERR("Unsupported or inconsistent PktUART TX module (version %i)"
                "not loading driver \n", md->version);
        return -EINVAL;
    }
    if (md->gtag == HM2_GTAG_PKTUART_RX
        && !hm2_md_is_consistent(hm2, md_index, 0, 4, 4, 0x000F)
        && !hm2_md_is_consistent(hm2, md_index, 1, 4, 4, 0x000F)
        && !hm2_md_is_consistent(hm2, md_index, 2, 4, 4, 0x000F)) {
        HM2_ERR("Unsupported or inconsistent PktUART RX module (version %i)"
                "not loading driver \n", md->version);
        return -EINVAL;
    }
    
    if (hm2->pktuart.num_instances > 1 && last_gtag == md->gtag) {
        HM2_ERR(
                "found duplicate Module Descriptor for %s (inconsistent "
                "firmware), not loading driver %i %i\n",
                hm2_get_general_function_name(md->gtag), md->gtag, last_gtag
                );
        return -EINVAL;
    }
    last_gtag = md->gtag;

    if (hm2->config.num_pktuarts > md->instances) {
        HM2_ERR(
                "config defines %d pktuarts, but only %d are available, "
                "not loading driver\n",
                hm2->config.num_pktuarts,
                md->instances
                );
        return -EINVAL;
    }
    
    if (hm2->config.num_pktuarts == 0) {
        return 0;
    }

    // 
    // looks good, start, or continue, initializing
    // 

    if (hm2->pktuart.num_instances == 0){
        if (hm2->config.num_pktuarts == -1) {
            hm2->pktuart.num_instances = md->instances;
        } else {
            hm2->pktuart.num_instances = hm2->config.num_pktuarts;
        }
        
        hm2->pktuart.instance = (hm2_pktuart_instance_t *)hal_malloc(hm2->pktuart.num_instances 
                                                               * sizeof(hm2_pktuart_instance_t));
        if (hm2->pktuart.instance == NULL) {
            HM2_ERR("out of memory!\n");
            r = -ENOMEM;
            goto fail0;
        }
    }

    // Register automatic updating of the Rx and Tx mode (status) registers
    if (md->gtag == HM2_GTAG_PKTUART_RX){
        hm2->pktuart.rx_version = md->version;
        r = hm2_register_tram_read_region(hm2, md->base_address + 3 * md->register_stride,
                                 (hm2->pktuart.num_instances * sizeof(rtapi_u32)),
                                 &hm2->pktuart.rx_status_reg);
        if (r < 0) {
            HM2_ERR("error registering tram read region for  PktUART Rx status(%d)\n", r);
            goto fail0;
        }
    } else if (md->gtag == HM2_GTAG_PKTUART_TX) {
        hm2->pktuart.tx_version = md->version;
        r = hm2_register_tram_read_region(hm2, md->base_address + 3 * md->register_stride,
                                 (hm2->pktuart.num_instances * sizeof(rtapi_u32)),
                                 &hm2->pktuart.tx_status_reg);
        if (r < 0) {
            HM2_ERR("error registering tram read region for  PktUART Tx status(%d)\n", r);
            goto fail0;
        }
    }

    for (i = 0 ; i < hm2->pktuart.num_instances ; i++){
        hm2_pktuart_instance_t *inst = &hm2->pktuart.instance[i];
        // For the time being we assume that all PktUARTS come on pairs
        if (inst->clock_freq == 0){
            inst->clock_freq = md->clock_freq;
            r = rtapi_snprintf(inst->name, sizeof(inst->name), "%s.pktuart.%01d", hm2->llio->name, i);
            HM2_PRINT("created PktUART Interface function %s.\n", inst->name);
        }
        if (md->gtag == HM2_GTAG_PKTUART_TX){
            inst->tx_addr = md->base_address + i * md->instance_stride;
            inst->tx_fifo_count_addr = (md->base_address 
                                        + md->register_stride 
                                        + i * md->instance_stride);
            inst->tx_bitrate_addr = (md->base_address 
                                     + 2 * md->register_stride
                                     + i * md->instance_stride);
            inst->tx_mode_addr = (md->base_address 
                                  + 3 * md->register_stride
                                  +i * md->instance_stride);  
        }
        else if (md->gtag == HM2_GTAG_PKTUART_RX){
            inst->rx_addr = md->base_address + i * md->instance_stride;
            inst->rx_fifo_count_addr = (md->base_address 
                                        + md->register_stride 
                                        + i * md->instance_stride);
            inst->rx_bitrate_addr = (md->base_address 
                                     + 2 * md->register_stride
                                     + i * md->instance_stride);
            inst->rx_mode_addr = (md->base_address 
                                  + 3 * md->register_stride 
                                  +i * md->instance_stride);    
        }
        else{
            HM2_ERR("Something very weird happened");
            goto fail0;
        }
    }

    return hm2->pktuart.num_instances;
fail0:
    return r;
}

EXPORT_SYMBOL_GPL(hm2_pktuart_setup_rx);
int hm2_pktuart_setup_rx(char *name, unsigned int bitrate, unsigned int filter_hz, unsigned int parity, int frame_delay, bool rx_enable, bool rx_mask){
    hostmot2_t *hm2;
    hm2_pktuart_instance_t *inst = 0;
    rtapi_u32 buff1, buff2;
    int i;
    int r = 0;
    rtapi_u32 filter;

    i = hm2_get_pktuart(&hm2, name);
    if (i < 0){
        HM2_ERR_NO_LL("Can not find PktUART instance %s.\n", name);
        return -EINVAL;
    }

    inst = &hm2->pktuart.instance[i];

/*
PktUART V2+ RX regmap
Bit 30         BadPop Error (read data FIFO with no data) RO
Bits 29..22    RX data digital filter bits (7..0) (in ClockLow periods)
               Should be set to 1/2 bit time
               (or max=65535 if it cannot be set long enough)
Bit  21	       FrameBuffer has data RO
Bits 20..16    Frames received RO
Bit  17        Parity enable WO
Bit  18        Odd Parity  WO  (1=odd, 0=even)
Bits 15..8     InterFrame delay in bit times RW
Bit  7	       RXBusy (serial state machine busy)
Bit  6	       RXMask RO
Bit  5         Parity Error RW
Bit  4	       RCFIFO Error RW
Bit  3	       RXEnable (must be set to receive packets) RW
Bit  2	       RXMask Enable (enables input data masking when transmitting) RW
Bit  1	       Overrun error (no stop bit when expected) (sticky) RW
Bit  0	       False Start bit error (sticky) RW*/

    filter = inst->clock_freq / filter_hz;
    if (hm2->pktuart.rx_version >= 2){
        if (filter > 0xFFFF) filter = 0xFFFF;
        buff1 = (rtapi_u32)((bitrate * 16777216.0)/inst->clock_freq); //24 bits in v2+
        buff1 |= (rtapi_u32)((filter & 0xFF00) << 16);
    } else {
        if (filter > 0xFF) filter = 0xFF;
        buff1 = (rtapi_u32)((bitrate * 1048576.0)/inst->clock_freq); //20 bits in v0 & v1
    }
    buff2  = (rtapi_u32)((filter & 0xFF) << 22);
    buff2 |= (rtapi_u32)(rx_mask << 2);
    buff2 |= (rtapi_u32)(rx_enable << 3);
    buff2 |= (rtapi_u32)((frame_delay & 0xFF) << 8);
    if (parity >  0) buff2 |= 0x20000;
    if (parity == 1) buff2 |= 0x40000;

    if (buff1 != inst->rx_bitrate){
        inst->rx_bitrate = buff1;
        r += hm2->llio->write(hm2->llio, inst->rx_bitrate_addr, &buff1, sizeof(rtapi_u32));
    }
    if (buff2 != inst->rx_mode){
        inst->rx_mode = buff2;
        r += hm2->llio->write(hm2->llio, inst->rx_mode_addr, &buff2, sizeof(rtapi_u32));
    }

    if (r < 0) {
        HM2_ERR("PktUART: hm2->llio->write failure %s setting up RX\n", name);
        return -1;
    }

    return 0;
}

EXPORT_SYMBOL_GPL(hm2_pktuart_setup_tx);
int hm2_pktuart_setup_tx(char *name, unsigned int bitrate, unsigned int parity, int frame_delay, bool drive_enable, bool drive_auto, int enable_delay){
    hostmot2_t *hm2;
    hm2_pktuart_instance_t *inst = 0;
    rtapi_u32 buff1, buff2;
    int i;
    int r = 0;

    i = hm2_get_pktuart(&hm2, name);
    if (i < 0){
        HM2_ERR_NO_LL("Can not find PktUART instance %s.\n", name);
        return -EINVAL;
    }
     inst = &hm2->pktuart.instance[i];

/*
PktUART V2+ TX regmap
Bit  21	    FrameBuffer Has Data RO
Bits 20..16 Frames to send  RO
Bit  17	    Parity enable WO
Bit  18	    Odd Parity  WO  (1=odd, 0=even)
Bits 15..8  InterFrame delay in bit times RW
Bit  7      Send busy RO
Bit  6      Drive Enable bit (enables external RS-422/485 Driver when set) RW
Bit  5      Drive enable Auto (Automatic external drive enable) RW
            Drive Enable Auto has priority over Drive Enable (bit 6 is a no-op if bit 5 is set)
Bit  4      SCFIFO Error RO
Bits 3..0   Drive enable delay (delay from asserting drive enable
            to start of data transmit. In CLock Low periods RW
            Drive enable delay is important to avoid start bit timing errors
            at high baud rates in RS-485 (half duplex) modes
*/

    if (hm2->pktuart.tx_version >= 2){
        buff1 = (rtapi_u32)((bitrate * 16777216.0)/inst->clock_freq); //24 bits in v2+
    } else {
        buff1 = (rtapi_u32)((bitrate * 1048576.0)/inst->clock_freq); //20 bits in v0 & v1
    }

    buff2  = (rtapi_u32)(enable_delay & 0x0F);
    if (drive_auto)   buff2 |= 0x00020;
    if (drive_enable) buff2 |= 0x00040;
    if (parity >  0)  buff2 |= 0x20000;
    if (parity == 1)  buff2 |= 0x40000;
    buff2 |= (rtapi_u32)((frame_delay & 0xFF) << 8);

    if (buff1 != inst->tx_bitrate){
        inst->tx_bitrate = buff1;
        r += hm2->llio->write(hm2->llio, inst->tx_bitrate_addr, &buff1, sizeof(rtapi_u32));
    }
    if (buff2 != inst->tx_mode){
        inst->tx_mode = buff2;
        r += hm2->llio->write(hm2->llio, inst->tx_mode_addr, &buff2, sizeof(rtapi_u32));
    }

    if (r < 0) {
        HM2_ERR("PktUART: hm2->llio->write failure %s setting up TX\n", name);
        return -1;
    }

    return 0;
}

EXPORT_SYMBOL_GPL(hm2_pktuart_reset);
void hm2_pktuart_reset(char *name){
    hostmot2_t *hm2;
    hm2_pktuart_instance_t *inst = 0;
    rtapi_u32 buff;
    int i;
    i = hm2_get_pktuart(&hm2, name);
    if (i < 0){
        HM2_ERR_NO_LL("Can not find PktUART instance %s.\n", name);
        return;
    }
    inst = &hm2->pktuart.instance[i];
    buff = 0x80010000;
    hm2->llio->write(hm2->llio, inst->tx_mode_addr, &buff, sizeof(rtapi_u32)); // clear sends, data FIFO and count register
    hm2->llio->write(hm2->llio, inst->rx_mode_addr, &buff, sizeof(rtapi_u32)); // clear receives, data FIFO and count register
}


EXPORT_SYMBOL_GPL(hm2_pktuart_setup);
// use -1 for bitrate, tx_mode and rx_mode to leave the mode unchanged
// use 1 for txclear or rxclear to issue a clear command for Tx or Rx registers
int hm2_pktuart_setup(char *name, unsigned int bitrate, rtapi_s32 tx_mode, rtapi_s32 rx_mode, int txclear, int rxclear){
    hostmot2_t *hm2;
    hm2_pktuart_instance_t *inst = 0;
    rtapi_u32 buff;
    int i;
    int r = 0;

    i = hm2_get_pktuart(&hm2, name);
    if (i < 0){
        HM2_ERR_NO_LL("Can not find PktUART instance %s.\n", name);
        return -EINVAL;
    }
    inst = &hm2->pktuart.instance[i];

    if (bitrate > 0){
        if (hm2->pktuart.tx_version >= 2){
            buff = (rtapi_u32)((bitrate * 16777216.0)/inst->clock_freq); //24 bits in v2+
        } else {
            buff = (rtapi_u32)((bitrate * 1048576.0)/inst->clock_freq); //20 bits in v0 & v1
        } 
        if (buff != inst->tx_bitrate){
            inst->tx_bitrate = buff;
            r += hm2->llio->write(hm2->llio, inst->tx_bitrate_addr, &buff, sizeof(rtapi_u32));
        }
        if (hm2->pktuart.rx_version >= 2){
            buff = (rtapi_u32)((bitrate * 16777216.0)/inst->clock_freq); //24 bits in v2+
        } else {
            buff = (rtapi_u32)((bitrate * 1048576.0)/inst->clock_freq); //20 bits in v0 & v1
        } 
        if (buff != inst->rx_bitrate){
            inst->rx_bitrate = buff;
            r += hm2->llio->write(hm2->llio, inst->rx_bitrate_addr, &buff, sizeof(rtapi_u32));
        }
    }

    /*  http://freeby.mesanet.com/regmap
      The PktUARTxMode register is used for setting and checking the
      PktUARTx's operation mode, timing and status:
      Bit  21          FrameBuffer Has Data
      Bits 20..16      Frames to send (input and output ports can overlap with an FPGA)
      Bit  17          Parity enable WO
      Bit  18          Odd Parity  WO  (1=odd, 0=even)
      Bits 15..8       InterFrame delay in bit times
      Bit  7           Transmit Logic active, not an error
      Bit  6           Drive Enable bit (enables external RS-422/485 Driver when set)
      Bit  5           Drive enable Auto (Automatic external drive enable)
      Bit  4           unused
      Bits 3..0        Drive enable delay (delay from asserting drive enable 
                       to start of data transmit). In CLock Low periods
    */
    if (tx_mode >= 0) {
        buff = ((rtapi_u32)tx_mode) & 0x3ffff;
        r += hm2->llio->write(hm2->llio, inst->tx_mode_addr, &buff, sizeof(rtapi_u32));
    }

    /* http://freeby.mesanet.com/regmap
      The PktUARTrMode register is used for setting and checking the PktUARTr's 
      operation mode, timing, and status
      Bits 29..22      RX data digital filter (in ClockLow periods)
                       Should be set to 1/2 bit time
                       (or max=255 if it cannot be set long enough)
      Bit  21          FrameBuffer has data
      Bit  17          Parity enable WO
      Bit  18          Odd Parity  WO  (1=odd, 0=even)
      Bits 20..16      Frames received
      Bits 15..8       InterFrame delay in bit times
      Bit  7           Receive Logic active, not an error
      Bit  6           RXMask
      Bit  5           Unused
      Bit  4           RCFIFO Error
      Bit  3           RXEnable (must be set to receive packets) 
      Bit  2           RXMask Enable (enables input data masking when transmitting)
      Bit  1           Overrun error (no stop bit when expected) (sticky)
      Bit  0           False Start bit error (sticky)
    */
    if (rx_mode >= 0) {
        buff = ((rtapi_u32)rx_mode) & 0x3FFFFFFF;
        r += hm2->llio->write(hm2->llio, inst->rx_mode_addr, &buff, sizeof(rtapi_u32));
    }

    /* http://freeby.mesanet.com/regmap
    The PktUARTx/PktUARTr mode register has a special data command that clears the PktUARTx/PktUARTr
    Clearing aborts any sends/receives in process, clears the data FIFO and
    clears the send count FIFO. To issue a clear command, you write 0x80010000
    to the PktUARTx/PktUARTr mode register.
     */
     buff = 0x80010000;
     if (txclear==1)
         r += hm2->llio->write(hm2->llio, inst->tx_mode_addr, &buff, sizeof(rtapi_u32)); // clear sends, data FIFO and count register
     if (rxclear==1)
         r += hm2->llio->write(hm2->llio, inst->rx_mode_addr, &buff, sizeof(rtapi_u32)); // clear receives, data FIFO and count register

    if (r < 0) {
        HM2_ERR("PktUART: hm2->llio->write failure %s\n", name);
        return -1;
    }

    return 0;
}


EXPORT_SYMBOL_GPL(hm2_pktuart_send);
int hm2_pktuart_send(char *name, const unsigned char data[], rtapi_u8 *num_frames, const rtapi_u16 frame_sizes[])
{
    hostmot2_t *hm2;
    rtapi_u32 buff;
    int c = 0;
    int r = 0;
    rtapi_u16 count = 0;
    int inst;
/*
       we work with nframes as a local copy of num_frames,
       so that we can return the num_frames sent out
       in case of SCFIFO error.
*/
    rtapi_u8 nframes;
    
    inst = hm2_get_pktuart(&hm2, name);
    if (inst < 0){
        HM2_ERR_NO_LL("Can not find PktUART instance %s.\n", name);
        return -EINVAL;
    }
    if (hm2->pktuart.instance[inst].tx_bitrate == 0){
        HM2_ERR("%s has not been configured.\n", name);
        return -EINVAL;
    }

    /* http://freeby.mesanet.com/regmap
       Send counts are written to 16 deep FIFO allowing up to 16 packets to be 
       sent in a burst (subject to data FIFO depth limits).
    */
    // Test if num_frames <= MaxTrFrames
    if ((*num_frames) > MaxTrFrames){
        nframes = MaxTrFrames;
    } else{
        nframes = *num_frames;
    }

    *num_frames = 0;

    rtapi_u8 i;
    for (i = 0; i < nframes; i++){
        count = count + frame_sizes[i];
        while (c < count - 3){
               buff = (data[c] + 
                      (data[c+1] << 8) +
                      (data[c+2] << 16) +
                      (data[c+3] << 24));
               r = hm2->llio->queue_write(hm2->llio, hm2->pktuart.instance[inst].tx_addr,
                             &buff, sizeof(rtapi_u32));
               if (r < 0) {
                   HM2_ERR("%s send: hm2->llio->queue_write failure\n", name);
                   return -1;
                  }
              c = c + 4;
        }


        // Now write the last bytes with bytes number < 4
        switch(count - c){
                 case 0:
                      break;
                 case 1:
                      buff = data[c];
                      r = hm2->llio->queue_write(hm2->llio, hm2->pktuart.instance[inst].tx_addr,
                                     &buff, sizeof(rtapi_u32));
                      if (r < 0){
                         HM2_ERR("%s send: hm2->llio->queue_write failure\n", name);
                         return -1;
                      }
                      break;
                 case 2:
                     buff = (data[c] +
                            (data[c+1] << 8));
                     r = hm2->llio->queue_write(hm2->llio, hm2->pktuart.instance[inst].tx_addr,
                                     &buff, sizeof(rtapi_u32));
                     if (r < 0){
                         HM2_ERR("%s send: hm2->llio->queue_write failure\n", name);
                         return -1;
                     }
                     break;
                 case 3:
                     buff = (data[c] +
                           (data[c+1] << 8) +
                           (data[c+2] << 16));
                     r = hm2->llio->queue_write(hm2->llio, hm2->pktuart.instance[inst].tx_addr,
                                     &buff, sizeof(rtapi_u32));
                     if (r < 0){
                         HM2_ERR("%s send: hm2->llio->queue_write failure\n", name);
                         return -1;
                     }
                     break;
              default:
                   HM2_ERR("%s send error in buffer parsing: count = %i, i = %i\n", name, count, c);
                   return -1;
        } // end switch
        (*num_frames)++;
        c = count;
    } // for loop

    // Write the number of bytes to be sent to PktUARTx sendcount register
    for (i = 0; i < nframes; i++){
        buff = (rtapi_u32) frame_sizes[i];
        r = hm2->llio->queue_write(hm2->llio, hm2->pktuart.instance[inst].tx_fifo_count_addr,
                                     &buff, sizeof(rtapi_u32));
        // Check for Send Count FIFO error
        r = hm2->llio->queue_read(hm2->llio, hm2->pktuart.instance[inst].tx_mode_addr,
                                     &buff, sizeof(rtapi_u32));
        if ((buff >> 4) & 0x01) {
            HM2_ERR_NO_LL("%s: SCFFIFO error\n", name);
            return -HM2_PKTUART_TxSCFIFOError;
        }
        if (r < 0){
            HM2_ERR("%s send: hm2->llio->queue_write failure\n", name);
            return -1;
        }
    }
    return count;
}

/* The function hm2_pktuart_read performs reads/writes outside of the normal
 * thread cycles. This is especially a problem with the Ethernet and p-port
 * connected cards where the reads and writes should be packeted.
 * This function has been left in place for backwards compatibility, but it
 * is recommended to use the hm2_pktuart_queue_* functions instead */

EXPORT_SYMBOL_GPL(hm2_pktuart_read);
int hm2_pktuart_read(char *name, unsigned char data[], rtapi_u8 *num_frames, rtapi_u16 *max_frame_length, rtapi_u16 frame_sizes[])
{
    hostmot2_t *hm2;
    int r, c;
    int bytes_total = 0; // total amount of bytes read
    rtapi_u16 countp; // packets count
    rtapi_u16 countb; // bytes count for the oldest packet received
    int inst;
    rtapi_u32 buff;
    rtapi_u16 data_size=(*num_frames)*(*max_frame_length);
    
    inst = hm2_get_pktuart(&hm2, name);
    
    if (inst < 0){ 
        HM2_ERR_NO_LL("Can not find PktUART instance %s.\n", name);
        *num_frames=0;  
        return -EINVAL;
    }
    if (hm2->pktuart.instance[inst].rx_bitrate == 0 ) {
        HM2_ERR("%s has not been configured.\n", name);
        *num_frames=0;  
        return -EINVAL;
    }


    // First poll the mode register for a non zero frames received count 
    // (mode register bits 20..16)
    r = hm2->llio->read(hm2->llio, hm2->pktuart.instance[inst].rx_mode_addr,
                        &buff, sizeof(rtapi_u32));
    if (r < 0) {
        HM2_ERR("%s read: hm2->llio->queue_read failure\n", name);
                return -1; // make the error message more detailed
    }
    if (buff & (0x1 << 21)){
        countp = (buff >> 16)  & 0x1f;
    } else {
        countp = 0;
    }
    HM2_INFO("hm2_pktuart: buffer = %08x\n", buff);
    HM2_INFO("hm2_pktuart: %i frames received\n", countp);

    // We expect to read at least 1 frame. 
    // If there is no complete frame yet in the buffer,
    // we'll deal with this by checking error bits.
    *num_frames = 0;

    // Bit 7 set does not really indicate any error condition,
    // but very probably means that the cycle time of the thread,
    // which you attach this function to, is not appropriate.       
    if ((buff >> 7) & 0x1){
        HM2_INFO("%s: Buffer error (RX idle but data in RX data FIFO)\n", name);
    }

    // Now check the error bits
    if ((buff >> 1) & 0x1){
        HM2_ERR_NO_LL("%s: Overrun error, no stop bit\n", name);
        return -HM2_PKTUART_RxOverrunError;
    }
    if (buff & 0x1){
        HM2_ERR_NO_LL("%s: False Start bit error\n", name);
        return -HM2_PKTUART_RxStartbitError;
    }

    // RCFIFO Error will get sticky if it is a consequence of either Overrun or False Start bit error?
    if ((buff >> 4) & 0x1){
        HM2_ERR_NO_LL("%s: RCFIFO Error\n", name);
        return -HM2_PKTUART_RxRCFIFOError;
    }

    if (countp==0){ 
        HM2_INFO_NO_LL("%s: no new frames \n", name);
        return 0;       // return zero bytes and zero frames
    }

    rtapi_u16 i=0;
    while ( i < countp ) {
          buff=0;
       /* The receive count register is a FIFO that contains the byte counts 
          of received packets. Since it is a FIFO it must only be read once after it 
          has be determined that there are packets available to read. */
          r = hm2->llio->read(hm2->llio, hm2->pktuart.instance[inst].rx_fifo_count_addr,
                        &buff, sizeof(rtapi_u32));

          countb = buff & 0x3ff; // PktUARTr  receive count register Bits 9..0 : bytes in receive packet

          if ((buff >> 14) & 0x1) {
              HM2_ERR_NO_LL("%s has False Start bit error in this packet.\n", name);
              return -HM2_PKTUART_RxPacketStartbitError;
          }

          if ((buff >> 15) & 0x1) {
              HM2_ERR_NO_LL("%s has Overrun error in this packet\n", name);
              return -HM2_PKTUART_RxPacketOverrrunError;
          }

           // a packet is completely received, but its byte count is zero
           // is very improbable, however we intercept this error too
          if (countb==0) {
              HM2_ERR_NO_LL("%s: packet %d has %d bytes.\n", name, countp+1, countb);
              return -HM2_PKTUART_RxPacketSizeZero; 
          }

          if (( bytes_total+countb)> data_size) {
               HM2_ERR_NO_LL("%s: bytes available %d are more than data array size %d\n", name, bytes_total+countb, data_size);
               return -HM2_PKTUART_RxArraySizeError ;
               countb = data_size - bytes_total;
          }

          (*num_frames)++; // increment num_frames to be returned at the end
          c = 0;
          buff = 0;
          frame_sizes[i]=countb;

          while (c < countb - 3){
                r = hm2->llio->read(hm2->llio, hm2->pktuart.instance[inst].rx_addr,
                            &buff, sizeof(rtapi_u32));

                if (r < 0) {
                   HM2_ERR("%s read: hm2->llio->queue_read failure\n", name);
                   return r;
                }
                 
                 data[bytes_total+c] = (buff & 0x000000FF); // i*frame_sizes[i]
                 data[bytes_total+c+1] = (buff & 0x0000FF00) >> 8;
                 data[bytes_total+c+2] = (buff & 0x00FF0000) >> 16;
                 data[bytes_total+c+3] = (buff & 0xFF000000) >> 24;
                 c = c + 4;
                 
            }

          switch(countb - c){
                 case 0: 
                      break;
                 case 1:
                      r = hm2->llio->read(hm2->llio, hm2->pktuart.instance[inst].rx_addr,
                                &buff, sizeof(rtapi_u32));
                      data[bytes_total+c]   = (buff & 0x000000FF);
                      break;
                 case 2:
                      r = hm2->llio->read(hm2->llio, hm2->pktuart.instance[inst].rx_addr,
                                &buff, sizeof(rtapi_u32));
                      data[bytes_total+c]   = (buff & 0x000000FF);
                      data[bytes_total+c+1] = (buff & 0x0000FF00) >> 8;
                      break;
                 case 3:
                      r = hm2->llio->read(hm2->llio, hm2->pktuart.instance[inst].rx_addr,
                                &buff, sizeof(rtapi_u32));
                      data[bytes_total+c]   = (buff & 0x000000FF);
                      data[bytes_total+c+1] = (buff & 0x0000FF00) >> 8;
                      data[bytes_total+c+2] = (buff & 0x00FF0000) >> 16;
                      break;
                default:
                     HM2_ERR_NO_LL("PktUART READ: Error in buffer parsing.\n");
                     return -EINVAL;
          }
        if (r < 0) {
            HM2_ERR("%s read: hm2->llio->queue_write failure\n", name);
            return -1;
        }

       bytes_total = bytes_total + countb;

       i++; // one frame/datagram read
    }// frame loop


    return bytes_total;
}

/* This function queues sufficient reads to get the available frame sizes
 * data is loaded into the *fsizes array on the next read cycle.
 * fsizes should be u32 x 16
 * FIXME: decide how to work out that the data has all been transferred
 */
int hm2_pktuart_queue_get_frame_sizes(char *name, rtapi_u32 fsizes[]){
    // queue as many reads of the FIFO as there are frames
    hostmot2_t *hm2;
    int inst;
    int j;
    int r;

    inst = hm2_get_pktuart(&hm2, name);

    if (inst < 0){
        HM2_ERR_NO_LL("Can not find PktUART instance %s.\n", name);
        return -EINVAL;
    }

    if (hm2->pktuart.instance[inst].rx_bitrate == 0 ) {
        HM2_ERR("%s has not been configured.\n", name);
        return -EINVAL;
    }

    for (j = 0; j < (int)((hm2->pktuart.rx_status_reg[inst] >> 16) & 0x1F); j++ ){
        rtapi_print_msg(RTAPI_MSG_INFO, "j = %i\n", j);
        r = hm2->llio->queue_read(hm2->llio, hm2->pktuart.instance[inst].rx_fifo_count_addr,
                     &fsizes[j], sizeof(rtapi_u32));
        if (r < 0){
            HM2_ERR("Unable to queue Rx FIFO count read");
        }
    }
    return j - 1;
}
EXPORT_SYMBOL_GPL(hm2_pktuart_queue_get_frame_sizes);

/* This function queues sufficient reads to extract the available data.
 * It does no error checking and does not explicitly check if there is
 * data to read (it will just queue zero reads in that case)
 * The using function should use the hm2_pktuart_get_rx_status functions
 * to determine whether data exists and error status.
 * The function queues enough reads to read the given number of 32 bit
 * frames, which should have been previously read by
 * hm2_pktuart_queue_get_frame_sizes returns the number of frame reads queued.
 */

int hm2_pktuart_queue_read_data(char *name, rtapi_u32 data[], int bytes) {
    hostmot2_t *hm2;
    int r;
    int i;
    int inst;

    inst = hm2_get_pktuart(&hm2, name);

    if (inst < 0){
        HM2_ERR_NO_LL("Can not find PktUART instance %s.\n", name);
        return -EINVAL;
    }
    if (hm2->pktuart.instance[inst].rx_bitrate == 0 ) {
        HM2_ERR("%s has not been configured.\n", name);
        return -EINVAL;
    }

/* queue enough reads to get the whole frame. Data will be transferred
 * to data[] next thread cycle, direct from FPGA, no serial latency    */
    for (i = 0; i < ((bytes % 4 == 0)? bytes / 4 : bytes / 4 + 1 ); i++ ){
        r = hm2->llio->queue_read(hm2->llio, hm2->pktuart.instance[inst].rx_addr,
                     &data[i], sizeof(rtapi_u32));
        if (r < 0){
            HM2_ERR("Unable to queue Rx FIFO read");
        }
    }
    return i - 1;
}
EXPORT_SYMBOL_GPL(hm2_pktuart_queue_read_data);


rtapi_u32 hm2_pktuart_get_rx_status(char *name){
    hostmot2_t *hm2;
    int i = hm2_get_pktuart(&hm2, name);
    return hm2->pktuart.rx_status_reg[i];
}
EXPORT_SYMBOL_GPL(hm2_pktuart_get_rx_status);


rtapi_u32 hm2_pktuart_get_tx_status(char *name){
    hostmot2_t *hm2;
    int i = hm2_get_pktuart(&hm2, name);
    return hm2->pktuart.tx_status_reg[i];
}
EXPORT_SYMBOL_GPL(hm2_pktuart_get_tx_status);


int hm2_pktuart_get_clock(char* name){
    hostmot2_t *hm2;
    int i = hm2_get_pktuart(&hm2, name);
    hm2_pktuart_instance_t inst = hm2->pktuart.instance[i];
    return inst.clock_freq;
}
EXPORT_SYMBOL_GPL(hm2_pktuart_get_clock);


int hm2_pktuart_get_version(char* name){
    hostmot2_t *hm2;
    hm2_get_pktuart(&hm2, name);
    return hm2->pktuart.tx_version + 16 * hm2->pktuart.rx_version  ;
}
EXPORT_SYMBOL_GPL(hm2_pktuart_get_version);

void hm2_pktuart_print_module(hostmot2_t *hm2){
    int i;
    HM2_PRINT("PktUART: %d\n", hm2->pktuart.num_instances);
    if (hm2->pktuart.num_instances <= 0) return;
    HM2_PRINT("    version: %d\n", hm2->pktuart.version);
    HM2_PRINT("    channel configurations\n");
    for (i = 0; i < hm2->pktuart.num_instances; i ++) {
        HM2_PRINT("    clock_frequency: %d Hz (%s MHz)\n", 
                  hm2->pktuart.instance[i].clock_freq, 
                  hm2_hz_to_mhz(hm2->pktuart.instance[i].clock_freq));
        HM2_PRINT("    instance %d:\n", i);
        HM2_PRINT("    HAL name = %s\n", hm2->pktuart.instance[i].name);
    }
}

// The following standard Hostmot2 functions are not currently used by pktuart. 

void hm2_pktuart_cleanup(hostmot2_t *hm2)
{
    (void)hm2;
}

void hm2_pktuart_write(hostmot2_t *hm2)
{
    (void)hm2;
}

