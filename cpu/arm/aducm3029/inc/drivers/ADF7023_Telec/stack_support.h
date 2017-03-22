/** \file stack_support.h
 *******************************************************************************
 ** \brief Provides different structure definitions required for MAC
 **
 ** \cond STD_FILE_HEADER
 **
 ** COPYRIGHT(c) 2010-11 Procubed Technology Solutions Pvt Ltd. 
 ** All rights reserved.
 **
 ** THIS SOFTWARE IS PROVIDED BY "AS IS" AND ALL WARRANTIES OF ANY KIND,
 ** INCLUDING THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR USE,
 ** ARE EXPRESSLY DISCLAIMED.  THE DEVELOPER SHALL NOT BE LIABLE FOR ANY
 ** DAMAGES WHATSOEVER RESULTING FROM THE USE OF THIS SOFTWARE. THIS SOFTWARE
 ** MAY NOT BE USED IN PRODUCTS INTENDED FOR USE IN IMPLANTATION OR OTHER
 ** DIRECT LIFE SUPPORT APPLICATIONS WHERE MALFUNCTION MAY RESULT IN THE DIRECT
 ** PHYSICAL HARM OR INJURY TO PERSONS. ALL SUCH IS USE IS EXPRESSLY PROHIBITED.
 **
 *******************************************************************************
 **  \endcond
 */

#ifndef STACK_SUPPORT_H
#define STACK_SUPPORT_H

#ifdef __cplusplus
extern "C" {
#endif

/*
** =============================================================================
** Public Macro definitions
** =============================================================================
*/
/**
 * \defgroup mac_interface stack_support
 */

#define START_REQ_RESERVE 				30

#define MCPS_DATA_REQ_RESERVE 			30

#define ASSOC_REQ_RESERVE 				40

#define ASSOC_RESP_RESERVE 				40

#define DISASSOC_REQ_RESERVE 			40

#define BEACON_REQ_RESERVE 				40

#define POLL_REQUEST_RESERVE 			40

#define ORPHAN_RESP_RESERVE 			40

#define SCAN_REQ_RESERVE    			30	

#define MAX_PANDESC_COUNT		 		5

#define PANID_CONFLICT_RESERVE    		60

/**
 ** \defgroup mac_defs  stack_support Definitions
 ** \ingroup mac_interface
 */

/*@{*/
/*
** =============================================================================
** Public Structures, Unions & enums Type Definitions
** =============================================================================
**/

/**
 *******************************************************************************
 ** \struct security_params
 ** Structure to indicate security features
 *******************************************************************************
 **/
typedef struct security_params
{
    uint8_t security_level;
    uint8_t key_id_mode;
    uint8_t key_identifier[9];
} security_params_t;



/**
 *******************************************************************************
 ** \struct mac_address_struct
 ** Structure to indicate address of Source and Destination
 *******************************************************************************
 **/
typedef struct mac_address_struct
{
    uint8_t address_mode;         /* addressing mode */
    ushort pan_id;                /* pan id (if present)*/
    union
    {
        ushort short_address;     /* short address */
        uint8_t ieee_address[8];    /* IEEE address */
    } address;
} mac_address_t;

typedef struct
{
    mac_address_t address;
    ulong channel;
    uchar page;
    uchar sf_spec[2];
    uchar gts_permit;
    uchar link_quality;
    uchar timestamp[3];
    uchar security;
	security_params_t sec_params;
} pandesc_t;


/**
 *******************************************************************************
 ** \struct mlme_set_req_tag
 ** Structure for the parameters to issue MLME_Set_Request API
 *******************************************************************************
 **/
typedef struct mlme_set_req_tag
{
    struct mlme_set_req_tag           *next;
    uint8_t                           Id;
    uint8_t                           PIBAttribute;		
    uint8_t                           PIBAttributeIndex;
    uint16_t                          PIBAttributeLength;	
    uint8_t                           PIBAttributeValue[1];
} mlme_set_req_t;

 /**
 *******************************************************************************
 ** \struct mlme_reset_req_tag
 ** Structure for the parameters to issue MLME_Reset_Request API
 *******************************************************************************
 **/
 
typedef struct mlme_reset_req_tag
{
    struct mlme_reset_req_tag         *next;
    uint8_t                           Id;
    uchar	                          SetDefaultPIB;
} mlme_reset_req_t;

/**
 *******************************************************************************
 ** \struct mlme_get_req_tag
 ** Structure for the parameters to issue MLME_Get_Request API
 *******************************************************************************
 **/
 
typedef struct mlme_get_req_tag
{
    struct mlme_get_req_tag           *next;
    uint8_t                           Id;
    uint8_t                           PIBAttribute;
    uint8_t                           PIBAttributeIndex;
} mlme_get_req_t;

/**
 *******************************************************************************
 ** \struct mcps_purge_req_tag
 ** Structure for the parameters to issue MCPS_Purge_Request API
 *******************************************************************************
 **/
 
typedef struct mcps_purge_req_tag
{
    struct mcps_purge_req_tag         *next;
    uint8_t                           Id;
    uchar	                          msduHandle;
} mcps_purge_req_t;

/**
 *******************************************************************************
 ** \struct mlme_start_req_tag
 ** Structure for the parameters to issue MLME_Start_Request API
 *******************************************************************************
 **/
typedef struct mlme_start_req_tag
{
    uint16_t                           PANId; 		
    uint8_t                            LogicalChannel;
    uint8_t                            ChannelPage;
    uint32_t                           startTime;		
    uint8_t                            BeaconOrder;	
    uint8_t                            SuperframeOrder; 
    uint8_t                            PANCoordinator;	
    bool                               BatteryLifeExtension; 
    uint8_t                            CoordRealignment;	
    uint8_t                            DSMESuperframeSpecification;
    uint8_t                            BeaconBitMap;
    uint8_t                            hoppingDescriptor;    
    uint8_t                            EnhancedBeaconOrder;
    uint8_t                            OffsetTimeSlot;
    uint16_t                           NBPANEnhancedBeaconOrder;
    security_params_t                  CRSecstuff;
    security_params_t                  BCNSecstuff;
    uint16_t                           MPM_EB_IEListLen; 
    uint8_t                            reserved[START_REQ_RESERVE];
    uint8_t                            msdu[1];   
} mlme_start_req_t;

/**
 *******************************************************************************
 ** \struct mcps_data_req_tag
 ** Structure for the parameters to issue MCPS_Data_Request API
 *******************************************************************************
 **/
typedef struct mcps_data_req_tag
{
    mac_address_t                       srcAddr; 
    mac_address_t                       dstAddr;		
    uint8_t                             msduHandle; 
    uint8_t                             txOptions;
    ushort	                            txChannel;
    bool                                PPDUCoding; 
    uint8_t                             FCSLength;
    bool                                modeSwitch;
    uint8_t                             newModeSUNPage;
    uint8_t                             modeSwitchParameterEntry;
    uint8_t                             frameCtrlOptions;
    bool                                sendMultiPurpose;
    security_params_t                   secstuff;
    uint16_t                            msduLength;
    uint16_t                            headerIElistLen;
    uint16_t                            payloadIElistLen;
    uint8_t                             reserved[MCPS_DATA_REQ_RESERVE]; 
    uint8_t                             msdu[1];
} mcps_data_req_t;

/**
 *******************************************************************************
 ** \struct mlme_assoc_req_tag
 ** Structure for the parameters to issue MLME_ASSOCIATE_Request API
 *******************************************************************************
 **/
typedef struct mlme_assoc_req_tag
{
    uint8_t                             logicalChannel;
    uint8_t                             channelPage;
    mac_address_t                       coordAddr;
    uint8_t                             capabilityInfo;
    uint8_t                             LowLatencyNwkInfo;
    uint16_t                            channelOffset;
    uint8_t                             hoppingSequenceID;
    security_params_t                   secstuff;
    uint8_t                             reserved[ASSOC_REQ_RESERVE];
    uint8_t                             payload[1];
} mlme_assoc_req_t;

/**
 *******************************************************************************
 ** \struct mlme_assoc_resp_tag
 ** Structure for the parameters to issue MLME_ASSOCIATE_Response API
 *******************************************************************************
 **/
typedef struct mlme_assoc_resp_tag
{
    uint8_t                             DeviceExtAddress[8];
    uint16_t                            AssocShortAddress;
    uint8_t                             Status;
    uint8_t                             LowLatencyNwkInfo;
    uint16_t                            channelOffset;
    uint16_t                            channelHoppingSeqLength;
    uint8_t                             channelHoppingSeq[5];
    security_params_t                   Secstuff;
    uint8_t                             reserved[ASSOC_RESP_RESERVE];
    uint8_t                             payload[1];
} mlme_assoc_resp_t;

/**
 *******************************************************************************
 ** \struct mlme_disassoc_req_tag
 ** Structure for the parameters to issue MLME_DISASSOCIATE_Request API
 *******************************************************************************
 **/
typedef struct mlme_disassoc_req_tag
{
    mac_address_t	                    DeviceAddr;
    uint8_t                             DisassociateReason;
    bool                                TxDirect;	
    security_params_t                   Secstuff; 
   	uint8_t                             reserved[DISASSOC_REQ_RESERVE];
   	uint8_t                             payload[1];
} mlme_disassoc_req_t;

/**
 *******************************************************************************
 ** \struct mlme_beacon_req_tag
 ** Structure for the parameters to issue MLME_BEACON_Request API
 *******************************************************************************
 **/
typedef struct mlme_beacon_req_tag
{
    uint8_t                             bcnType;
    uint8_t                             channel;
    uint8_t                             channelPage;
    uint8_t                             superFrameOrder;
    mac_address_t	                    DstAddr;
    bool                                BSNSuppression;
    security_params_t                   Secstuff;
    uint16_t                            headerIElistLen;
    uint16_t                            payloadIElistLen;
    uint8_t                             reserved[BEACON_REQ_RESERVE];    
    uint8_t                             msdu[1];
} mlme_beacon_req_t;

/**
 *******************************************************************************
 ** \struct mlme_poll_req_tag
 ** Structure for the parameters to issue MLME_POLL_Request API
 *******************************************************************************
 **/
typedef struct mlme_poll_req_tag
{
    mac_address_t                       CoordAddress;	      
    security_params_t                   Secstuff;
    uint8_t                             reserved[POLL_REQUEST_RESERVE];
    uint8_t                             payload[1];
} mlme_poll_req_t;

/**
 *******************************************************************************
 ** \struct mlme_panid_conflict_tag
 ** Structure for the parameters to issue PANID Conflict notification.
 *******************************************************************************
 **/
typedef struct mlme_panid_conflict_tag
{
	uint8_t                             reserved[PANID_CONFLICT_RESERVE];
	uint8_t                             payload[1];
} mlme_panid_conflict_t;

/**
 *******************************************************************************
 ** \struct mlme_orphan_resp_tag
 ** Structure for the parameters to issue MLME_ORPHAN_Response API
 *******************************************************************************
 **/
typedef struct mlme_orphan_resp_tag
{
    uint8_t	                            OrphanAddress[8];	
    ushort 	                            ShortAddress;	
    uint8_t                             AssociatedMember;	
    security_params_t                   Secstuff;
    uint8_t                             reserved[ORPHAN_RESP_RESERVE];
    uint8_t                             payload[1];
} mlme_orphan_resp_t;

/**
 *******************************************************************************
 ** \struct mlme_scan_req_tag
 ** Structure for the parameters to issue MLME_SCAN_Request API
 *******************************************************************************
 **/
typedef struct mlme_scan_req_tag
{
    uint8_t	                            scanType;		
    uint64_t                            scanChannels;	
    uint8_t	                            scanDuration;
    uint8_t                             channelPage;
    bool                                linkQualityScan;
    uint8_t                             frameControlOptions;	
    uint8_t                             mpm_scanduration_bpan;
    uint16_t                            mpm_scanduration_nbpan;
    security_params_t                   Secstuff;
    uint16_t                            headerIElistLen;
    uint16_t                            payloadIElistLen;
    uint8_t                             reserved[SCAN_REQ_RESERVE];
    uint8_t                             payload[1];
} mlme_scan_req_t;


 /**
 *******************************************************************************
 ** \struct mlme_start_conf_tag
 ** Structure for the parameters of the MLME_Start_Confirm callback
 *******************************************************************************
 **/
typedef struct mlme_start_conf_tag
{
    struct mlme_start_conf_tag         *next;
    uint8_t                            Id;
    uint8_t                            status;
} mlme_start_conf_t;

 /**
 *******************************************************************************
 ** \struct mcps_data_conf_tag
 ** Structure for the parameters of the MCPS_Data_Confirm callback
 *******************************************************************************
 **/
typedef struct mcps_data_conf_tag
{
    struct mcps_data_conf_tag          *next;
    uint8_t                            Id;
    uint8_t                            msduHandle; 
    uint8_t                            status;
    uint8_t                            NumBackoffs;
    uint32_t                           Timestamp;
} mcps_data_conf_t;


/**
 *******************************************************************************
 ** \struct mlme_set_conf_tag
 ** Structure for the parameters of the MLME_Set_Confirm callback
 *******************************************************************************
 **/
typedef struct mlme_set_conf_tag
{
    struct mlme_set_conf_tag            *next;
    uint8_t                             Id;
    uint8_t                             status;
    uint8_t                             PIBAttribute;
    uint8_t                             PIBAttributeIndex;
}mlme_set_conf_t;
/**
 *******************************************************************************
 ** \struct mlme_reset_conf_tag
 ** Structure for the parameters of the MLME_Reset_Confirm callback
 *******************************************************************************
 **/
typedef struct mlme_reset_conf_tag
{
    struct mlme_reset_conf_tag          *next;
    uint8_t                             Id;
    uint8_t                             status;
}mlme_reset_conf_t;

/**
 *******************************************************************************
 ** \struct mlme_get_conf_tag
 ** Structure for the parameters of the MLME_Get_Confirm callback
 *******************************************************************************
 **/
typedef struct mlme_get_conf_tag
{
    struct mlme_get_conf_tag            *next;
    uint8_t                             Id;
    uint8_t                             status;
    uint8_t                             PIBAttribute;
    uint8_t                             PIBAttributeIndex;
    uint16_t                            PIBAttributeLength;
    uint8_t                             PIBAttributeValue[1];
 } mlme_get_conf_t;
 
/**
 *******************************************************************************
 ** \struct mcps_purge_conf_tag
 ** Structure for the parameters of the MCPS_Purge_Confirm callback
 *******************************************************************************
 **/
typedef struct mcps_purge_conf_tag
{
    struct mcps_purge_conf_tag          *next;
    uint8_t                             Id;
    uint8_t                             msdu_handle; 
    uint8_t                             status; 
} mcps_purge_conf_t;

/**
 *******************************************************************************
 ** \struct mlme_beacon_conf_tag
 ** Structure for the parameters of the MLME_beacon_Confirm callback
 *******************************************************************************
 **/
typedef struct mlme_beacon_conf_tag
{
    struct mlme_beacon_conf_tag         *next;
    uint8_t                             Id;
    uchar                               status;
} mlme_beacon_conf_t;

/**
 *******************************************************************************
 ** \struct mlme_poll_conf_tag
 ** Structure for the parameters of the MLME_Poll_Confirm callback
 *******************************************************************************
 **/
typedef struct mlme_poll_conf_tag
{
    struct mlme_poll_conf_tag           *next;
    uint8_t                             Id;
    uint8_t                             status;
} mlme_poll_conf_t;

/**
 *******************************************************************************
 ** \struct mlme_assoc_conf_tag
 ** Structure for the parameters of the MLME association Confirm callback
 *******************************************************************************
 **/
typedef struct mlme_assoc_conf_tag
{
    struct mlme_assoc_conf_tag          *next;
    uint8_t                             Id;
    uint16_t                            short_address;
    uint8_t                             status;
    uint8_t                             LowLatencyNwkInfo;
    uint16                              channelOffset;
    uint8_t                             HoppingSequenceLength;
    uint8_t                             HoppingSequence;
    security_params_t                   sec_params;
} mlme_assoc_conf_t;

/**
 *******************************************************************************
 ** \struct mlme_disassoc_conf_tag
 ** Structure for the parameters of the MLME disassociation Confirm callback
 *******************************************************************************
 **/
typedef struct mlme_disassoc_conf_tag
{
    struct mlme_disassoc_conf_tag       *next;
    uint8_t                             Id;
    uint8_t                             status;
    mac_address_t                       Deviceaddr;
} mlme_disassoc_conf_t;

/**
 *******************************************************************************
 ** \struct mlme_scan_conf_tag
 ** Structure for the parameters of the MLME scan Confirm callback
 *******************************************************************************
 **/
typedef struct mlme_scan_conf_tag
{
    struct mlme_scan_conf_tag           *next;
    uint8_t                             Id;
    uint8_t                             status;
    uint8_t                             ScanType;
    uint8_t                             ChannelPage;
    uint64_t                            unscannedChannels;
    uint8_t                             ResultListSize;
    uint8_t                             *ResultList;
} mlme_scan_conf_t;

/**
 *******************************************************************************
 ** \struct mcps_data_ind_tag
 ** Structure for the parameters of the MLME data indication callback
 *******************************************************************************
 **/
typedef struct mcps_data_ind_tag
{
    mac_address_t                       Srcaddr;
    mac_address_t                       Dstaddr;
    uint8_t                             mpduLinkQuality;
    uint8_t                             DSN;
    uint32_t                            Timestamp;
    security_params_t                   Sec;
    uint16_t                            msduLength;
    uint8_t                             *pMsdu;
} mcps_data_ind_t;

/**
 *******************************************************************************
 ** \struct mlme_assoc_ind_tag
 ** Structure for the parameters of the MLME association indication callback.
 *******************************************************************************
 **/
typedef struct mlme_assoc_ind_tag
{
    uint8_t                             Child_64_bit_addr[8];
    uint8_t                             CapabilityInformation;
    uint8_t                             LowLatencyNwkInfo;
    uint16_t                            channelOffset;
    uint8_t                             HoppingSequenceID;
    security_params_t                   sec_params;
} mlme_assoc_ind_t;

/**
 *******************************************************************************
 ** \struct mlme_disassoc_ind_tag
 ** Structure for the parameters of the MLME dissociation indication callback.
 *******************************************************************************
 **/
typedef struct mlme_disassoc_ind_tag
{
    uint8_t                             DeviceAddress[8];
    uint8_t                             DisassociateReason;
    security_params_t                   sec_params;
} mlme_disassoc_ind_t;

/**
 *******************************************************************************
 ** \struct mlme_comm_status_ind_tag
 ** Structure for the parameters of the MLME comm status indication callback.
 *******************************************************************************
 **/
typedef struct mlme_comm_status_ind_tag
{
    mac_address_t                       Srcaddr;
    mac_address_t                       Dstaddr;
    uint8_t                             status;
    security_params_t                   sec_param;
} mlme_comm_status_ind_t;

/**
 *******************************************************************************
 ** \struct mlme_beacon_notify_ind_tag
 ** Structure for the parameters of the MLME beacon nofify indication callback.
 *******************************************************************************
 **/
typedef struct mlme_beacon_notify_ind_tag
{
    uint8_t                             bsn;
    pandesc_t                           pandesc;
    uint8_t                             pendAddrSpec;
    uint8_t                             *pendaddrlist;
    uint8_t                             ebsn;
    uint8_t                             beaconType;
    uint8_t                             payloadIEPresent;
    uint16_t                            sdulen;
    uint8_t                             *pSdu;
} mlme_beacon_notify_ind_t;

/**
 *******************************************************************************
 ** \struct mlme_beacon_req_ind_tag
 ** Structure for the parameters of the MLME beacon request indication callback.
 *******************************************************************************
 **/
typedef struct mlme_beacon_req_ind_tag
{
    uchar                               bcn_type;
    mac_address_t                       src_addr;
    ushort                              dest_pan_id;
    ushort                              headerIEListlen;
    ushort                              payloadIEListlen;
    uint8_t                             *pPayload;
} mlme_beacon_req_ind_t;

/**
 *******************************************************************************
 ** \struct mlme_orphan_ind_tag
 ** Structure for the parameters of the MLME orphan indication callback.
 *******************************************************************************
 **/
typedef struct mlme_orphan_ind_tag
{
    uint8_t                             Orphan64bitAddress[8];
    security_params_t                   sec_params; 
} mlme_orphan_ind_t;

/**
 *******************************************************************************
 ** \struct mlme_sync_loss_ind_tag
 ** Structure for the parameters of the MLME sync loss indication callback.
 *******************************************************************************
 **/
typedef struct mlme_sync_loss_ind_tag
{
    uint8_t                             LossReason;	
    uint16_t                            PANId;	
    uint8_t                             LogicalChannel; 
    uint8_t                             ChannelPage;	
    security_params_t                   sec_params;
} mlme_sync_loss_ind_t;
 
/**
 *******************************************************************************
 ** \struct phy_data_params_t
 ** Data structure to store phy data request params
 *******************************************************************************
 **/
typedef struct phy_data_params_struct
{
	uint16_t tx_channel;			/**< Holds the channel on which tx should be done */
	uint8_t PPDUCoding;				/**< Indicates if the PPDU should be FEC encoded or not*/
	uint8_t FCSLength;				/**< The size of the FCS in the passed PPDU. True for 32-bit CRC, false for 16-bit CRC */
	uint8_t ModeSwitch;				/**< Indicates if PPDU should be transmitted in a different mode */
	uint8_t NewModeSUNPage;			/**< The new SUN page if mode switching is required */
	uint8_t ModeSwitchParameterEntry;
}phy_data_params_t;

/**
 *******************************************************************************
 ** \struct generic_frame_build_tag
 ** Structure for the parameters to build the generic frame.
 *******************************************************************************
 **/
typedef struct generic_frame_build_tag
{
    uchar                               type;
    uchar								sub_type;
    mac_address_t                		src; 
    mac_address_t                		dst; 
    uchar                               tx_options;
    uchar                               fc_options;
    uchar                               enc_offset;
    uint8_t 							ie_present;
    phy_data_params_t       			phy_params;
    security_params_t         			sec_params;
    uint16_t                           	payload_length;
    uchar                               *pPayload;
} generic_frame_build_t;

typedef uint8_t*(*allocate_rx_buffer_fp)(uint8_t *recv_buf);

typedef uint8_t*(*Get_phy_rx_struct_location_fp)(uint8_t *buf);

typedef uint8_t*(*Get_Reception_location_fp)(uint8_t *buf, uint8_t *recv_data);

/*@}*/

/*
** ============================================================================
** Public Variable Declarations
** ============================================================================
*/

/* None */

/*
** ============================================================================
** Private Variable Definitions
** ============================================================================
*/

/* None */

/*
** =============================================================================
** Public Variables Definitions
** =============================================================================
**/

/* None */

/*
** ============================================================================
** External Variable Declarations
** ============================================================================
*/

extern allocate_rx_buffer_fp allocate_rx_buffer;

extern Get_phy_rx_struct_location_fp Get_phy_rx_struct_location;

extern Get_Reception_location_fp Get_Reception_location;

/*
** ============================================================================
** Public Function Prototypes
** ============================================================================
*/
uint8_t* allocate_mac_buffer(uint8_t primitiveId, 
							uint8_t secured, 
							uint16_t buff_len,
							void **params);
uint8_t *allocate_buf_request(uint16_t len);
void free_buf_request(void *data);

uint8_t *GetScanConfListLoc(uint8_t *buf, uint16_t payloadlen);
uint8_t *getEBFilterIELoc(	uint8_t *payloadIElist, 
							uint16_t payloadIElistLen, 
							uint8_t *EBFilterIE, 
							uint8_t *EBFilterLen,
							uint8_t **MLME_IE_Loc );
/*@}*/

/*@}*/

#ifdef __cplusplus
}
#endif
#endif /* STACK_SUPPORT_H */


