/*
 * Copyright (C) 2012 Nexell Co., All Rights Reserved
 * Nexell Co. Proprietary & Confidential
 *
 * NEXELL INFORMS THAT THIS CODE AND INFORMATION IS PROVIDED "AS IS" BASE
 * AND WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING
 * BUT NOT LIMITED TO THE IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS
 * FOR A PARTICULAR PURPOSE.
 *
 * Module	: SDXC
 * File		: dw_sd.c
 * Description	:
 * Author	: Hans
 * History	: 2016.05.12 First implementation
 */

#include <debug.h>
#include <platform.h>
#include <nx_s5p6818.h>
#include <nx_clkpwr.h>
#include <nx_clkgen.h>
#include <nx_gpio.h>
#include <nx_rstcon.h>
#include <dw_sdmmc.h>
#include <nx_sdmmc.h>
#include <nx_bootheader.h>

#define NX_CLKSRC_SDMMC     (NX_CLKPWR_PLL_2)

/*----------------------------------------------------------------------------*/
static struct nx_clkgen_registerset * const pgsdclkgenreg[3] = {
	(struct nx_clkgen_registerset *)PHY_BASEADDR_CLKGEN18_MODULE,
	(struct nx_clkgen_registerset *)PHY_BASEADDR_CLKGEN19_MODULE,
	(struct nx_clkgen_registerset *)PHY_BASEADDR_CLKGEN20_MODULE
};
static unsigned int const sdresetnum[3] = {
	RESETINDEX_OF_SDMMC0_MODULE_i_nRST,
	RESETINDEX_OF_SDMMC1_MODULE_i_nRST,
	RESETINDEX_OF_SDMMC2_MODULE_i_nRST
};
static struct nx_sdmmc_registerset * const pgsdxcreg[3] = {
	(struct nx_sdmmc_registerset *)PHY_BASEADDR_SDMMC0_MODULE,
	(struct nx_sdmmc_registerset *)PHY_BASEADDR_SDMMC1_MODULE,
	(struct nx_sdmmc_registerset *)PHY_BASEADDR_SDMMC2_MODULE
};

struct nx_tbbinfo tbi;
struct cardstatus lcardstatus;

/*----------------------------------------------------------------------------*/
struct nx_clkinfo_sdmmc {
	unsigned int npllnum;
	unsigned int nfreqhz;
	unsigned int nclkdiv;
	unsigned int nclkgendiv;
};

bool   nx_sdmmc_getclkparam(struct nx_clkinfo_sdmmc *pclkinfo)
{
	unsigned int srcfreq;
	unsigned int nretry = 1, ntemp = 0;
	bool   fret = false;

	srcfreq = nx_clkpwr_getpllfreq(pclkinfo->npllnum);

	do {
		for (pclkinfo->nclkdiv = 2; ; pclkinfo->nclkdiv += 2) {
			ntemp   = (pclkinfo->nfreqhz * pclkinfo->nclkdiv);
			pclkinfo->nclkgendiv  = srcfreq / ntemp;

			if (srcfreq > (pclkinfo->nfreqhz * pclkinfo->nclkdiv))
				pclkinfo->nclkgendiv += 2;

			if (pclkinfo->nclkgendiv < 255)
				break;
		}

		ntemp = srcfreq / (pclkinfo->nclkgendiv * pclkinfo->nclkdiv);
		if (ntemp <= pclkinfo->nfreqhz) {
			fret = true;
			break;
		}

	} while (nretry--);

	return fret;
}

static bool nx_sdmmc_setclock(
		struct cardstatus *pcardstatus,
		bool enb,
		unsigned int nfreq)
{
	volatile unsigned int timeout;
	volatile struct nx_sdmmc_registerset * const psdxcreg =
				pgsdxcreg[pcardstatus->sdport];
	struct nx_clkgen_registerset * const psdclkgenreg =
				pgsdclkgenreg[pcardstatus->sdport];
	struct nx_clkinfo_sdmmc clkinfo;
	bool ret;

#if 0
	VERBOSE("NX_SDMMC_SetClock : divider = %d\r\n", divider);
#endif


	/*----------------------------------------------------------------------
	* 1. Confirm that no card is engaged in any transaction.
	*	If there's a transaction, wait until it has been finished.
	*	while( NX_SDXC_IsDataTransferBusy() );
	*	while( NX_SDXC_IsCardDataBusy() );
	*/

#if defined(NX_DEBUG)
	if (psdxcreg->status & (NX_SDXC_STATUS_DATABUSY |
				NX_SDXC_STATUS_FSMBUSY)) {
		if (psdxcreg->status & NX_SDXC_STATUS_DATABUSY)
			ERROR("NX_SDMMC_SetClock : ERROR - Data is busy\r\n");

		if (psdxcreg->status & NX_SDXC_STATUS_FSMBUSY)
			ERROR("NX_SDMMC_SetClock : ERROR - Data Transfer is busy\r\n");

		timeout = NX_SDMMC_TIMEOUT;
		while (timeout--) {
			if (!(psdxcreg->status &
					(NX_SDXC_STATUS_DATABUSY |
					 NX_SDXC_STATUS_FSMBUSY)))
				break;
		}
		if (timeout == 0) {
			INFINTE_LOOP();
		}
	}
#endif

	/*----------------------------------------------------------------------
	 * 2. Disable the output clock.
	 */
	psdxcreg->clkena &= ~NX_SDXC_CLKENA_CLKENB;
	psdxcreg->clkena |= NX_SDXC_CLKENA_LOWPWR;

	psdclkgenreg->clkenb = NX_PCLKMODE_ALWAYS<<3 | NX_BCLKMODE_DYNAMIC<<0;
#if 0
	psdclkgenreg->clkgen[0] =
		(psdclkgenreg->clkgen[0] & ~(0x7<<2 | 0xFF<<5)) |
		(SDXC_CLKGENSRC<<2) |			/* set clock source */
		((divider-1)<<5) |			/* set clock divisor */
		(0UL<<1);				/* set clock invert */
#else

	clkinfo.npllnum = NX_CLKSRC_SDMMC;
	clkinfo.nfreqhz = nfreq;
	ret = nx_sdmmc_getclkparam(&clkinfo);
	if (ret == true) {
		psdclkgenreg->clkgen[0] =
			(psdclkgenreg->clkgen[0] & ~(0x7<<2 | 0xFF<<5))
			| (clkinfo.npllnum<<2)		/* set clock source */
			| ((clkinfo.nclkgendiv-1)<<5)	/* set clock divisor */
			| (0UL<<1);			/* set clock invert */
		/*  2*n divider (0 : bypass) */
		psdxcreg->clkdiv = (clkinfo.nclkdiv>>1);
	}
#endif
	psdclkgenreg->clkenb |= 0x1UL<<2;	/* clock generation enable */
	psdxcreg->clkena &= ~NX_SDXC_CLKENA_LOWPWR;	/* normal power mode */
	/*----------------------------------------------------------------------
	 * 3. Program the clock divider as required.
	 */
#if 0
	psdxcreg->clksrc = 0;	/* prescaler 0 */
	psdxcreg->clkdiv = SDXC_CLKDIV>>1;	/* 2*n divider (0 : bypass) */
	psdxcreg->clkdiv = (divider>>1);	/* 2*n divider (0 : bypass) */
#endif

	/*----------------------------------------------------------------------
	 * 4. Start a command with NX_SDXC_CMDFLAG_UPDATECLKONLY flag.
	 */
repeat_4:
	psdxcreg->cmd = 0 | NX_SDXC_CMDFLAG_STARTCMD |
			NX_SDXC_CMDFLAG_UPDATECLKONLY |
			NX_SDXC_CMDFLAG_STOPABORT;

	/*----------------------------------------------------------------------
	 * 5. Wait until a update clock command is taken by the SDXC module.
	 *	If a HLE is occurred, repeat 4.
	 */
	timeout = 0;
	while (psdxcreg->cmd & NX_SDXC_CMDFLAG_STARTCMD) {
		if (++timeout > NX_SDMMC_TIMEOUT) {
			ERROR("NX_SDMMC_SetClock : ERROR - Time-out to update clock.\r\n");
			INFINTE_LOOP();
			return false;
		}
	}

	if (psdxcreg->rintsts & NX_SDXC_RINTSTS_HLE) {
		INFINTE_LOOP();
		psdxcreg->rintsts = NX_SDXC_RINTSTS_HLE;
		goto repeat_4;
	}

	if (false == enb)
		return true;

	/*----------------------------------------------------------------------
	 * 6. Enable the output clock.
	 */
	psdxcreg->clkena |= NX_SDXC_CLKENA_CLKENB;

	/*----------------------------------------------------------------------
	 * 7. Start a command with NX_SDXC_CMDFLAG_UPDATECLKONLY flag.
	 */
repeat_7:
	psdxcreg->cmd = 0 | NX_SDXC_CMDFLAG_STARTCMD |
			NX_SDXC_CMDFLAG_UPDATECLKONLY |
			NX_SDXC_CMDFLAG_STOPABORT;

	/*----------------------------------------------------------------------
	 * 8. Wait until a update clock command is taken by the SDXC module.
	 *	If a HLE is occurred, repeat 7.
	 */
	timeout = 0;
	while (psdxcreg->cmd & NX_SDXC_CMDFLAG_STARTCMD) {
		if (++timeout > NX_SDMMC_TIMEOUT) {
			ERROR("NX_SDMMC_SetClock : ERROR - TIme-out to update clock2.\r\n");
			INFINTE_LOOP();
			return false;
		}
	}

	if (psdxcreg->rintsts & NX_SDXC_RINTSTS_HLE) {
		INFINTE_LOOP();
		psdxcreg->rintsts = NX_SDXC_RINTSTS_HLE;
		goto repeat_7;
	}

	return true;
}

/*----------------------------------------------------------------------------*/
static unsigned int nx_sdmmc_sendcommandinternal(
	       struct cardstatus *pcardstatus,
	       struct nx_sdmmc_command *pcommand)
{
	unsigned int	cmd, flag;
	unsigned int	status = 0;
	volatile unsigned int	timeout;
	volatile struct nx_sdmmc_registerset * const psdxcreg =
					pgsdxcreg[pcardstatus->sdport];

	VERBOSE("NX_SDMMC_SendCommandInternal : Command(0x%08X), Argument(0x%08X)\r\n",
					pcommand->cmdidx, pcommand->arg);

	cmd	= pcommand->cmdidx & 0xff;
	flag = pcommand->flag;

	psdxcreg->rintsts = 0xFFFFFFFF;

	/*----------------------------------------------------------------------
	 * Send command
	 */
	timeout = 0;
	do {
		psdxcreg->rintsts	= NX_SDXC_RINTSTS_HLE;
		psdxcreg->cmdarg	= pcommand->arg;
		psdxcreg->cmd		= cmd | flag |
					NX_SDXC_CMDFLAG_USE_HOLD_REG;
		while (psdxcreg->cmd & NX_SDXC_CMDFLAG_STARTCMD) {
			if (++timeout > NX_SDMMC_TIMEOUT) {
				ERROR("NX_SDMMC_SendCommandInternal : ERROR - Time-Out to send command.\r\n");
				status |= NX_SDMMC_STATUS_CMDBUSY;
				INFINTE_LOOP();
				goto End;
			}
		}
	} while (psdxcreg->rintsts & NX_SDXC_RINTSTS_HLE);

	/*----------------------------------------------------------------------
	* Wait until Command sent to card and got response from card.
	*/
	timeout = 0;
	while (1) {
		if (psdxcreg->rintsts & NX_SDXC_RINTSTS_CD)
			break;

		if (++timeout > NX_SDMMC_TIMEOUT) {
			ERROR("NX_SDMMC_SendCommandInternal : ERROR - Time-Out to wait command done.\r\n");
			status |= NX_SDMMC_STATUS_CMDTOUT;
			INFINTE_LOOP();
			goto End;
		}

		if ((flag & NX_SDXC_CMDFLAG_STOPABORT) &&
				(psdxcreg->rintsts & NX_SDXC_RINTSTS_HTO)) {
			/* You have to clear FIFO when HTO is occurred.
			* After that, SDXC module leaves in stopped State
			* and takes next command.
			*/
			while (0 == (psdxcreg->status &
						NX_SDXC_STATUS_FIFOEMPTY)){
				psdxcreg->data;
			}
		}
	}

	/* Check Response Error */
	if (psdxcreg->rintsts & (NX_SDXC_RINTSTS_RCRC |
				NX_SDXC_RINTSTS_RE |
				NX_SDXC_RINTSTS_RTO)) {
		if (psdxcreg->rintsts & NX_SDXC_RINTSTS_RCRC)
			status |= NX_SDMMC_STATUS_RESCRCFAIL;
		if (psdxcreg->rintsts & NX_SDXC_RINTSTS_RE)
			status |= NX_SDMMC_STATUS_RESERROR;
		if (psdxcreg->rintsts & NX_SDXC_RINTSTS_RTO)
			status |= NX_SDMMC_STATUS_RESTOUT;
	}

	if ((status == NX_SDMMC_STATUS_NOERROR) &&
				(flag & NX_SDXC_CMDFLAG_SHORTRSP)) {
		pcommand->response[0] = psdxcreg->resp0;
		if ((flag & NX_SDXC_CMDFLAG_LONGRSP) ==
						NX_SDXC_CMDFLAG_LONGRSP) {
			pcommand->response[1] = psdxcreg->resp1;
			pcommand->response[2] = psdxcreg->resp2;
			pcommand->response[3] = psdxcreg->resp3;
		}

		if (NX_SDMMC_RSPIDX_R1B == ((pcommand->cmdidx >> 8) & 0xFF)) {
			timeout = 0;
			do {
				if (++timeout > NX_SDMMC_TIMEOUT) {
					ERROR("NX_SDMMC_SendCommandInternal : ERROR - Time-Out to wait card data is ready.\r\n");
					status |= NX_SDMMC_STATUS_DATABUSY;
					INFINTE_LOOP();
					goto End;
				}
			} while (psdxcreg->status & NX_SDXC_STATUS_DATABUSY);
		}
	}

End:

#if defined(NX_DEBUG)
	if (NX_SDMMC_STATUS_NOERROR != status) {
		WARN("NX_SDMMC_SendCommandInternal Failed : command(0x%08X), argument(0x%08X) => status(0x%08X)\r\n",
				pcommand->cmdidx, pcommand->arg, status);
	}
#endif

	pcommand->status = status;

	return status;
}

/*----------------------------------------------------------------------------*/
static unsigned int nx_sdmmc_sendstatus(struct cardstatus *pcardstatus)
{
	unsigned int status;
	struct nx_sdmmc_command cmd;

	cmd.cmdidx	= SEND_STATUS;
	cmd.arg		= pcardstatus->rca;
	cmd.flag	= NX_SDXC_CMDFLAG_STARTCMD |
			NX_SDXC_CMDFLAG_CHKRSPCRC |
			NX_SDXC_CMDFLAG_SHORTRSP;

	status = nx_sdmmc_sendcommandinternal(pcardstatus, &cmd);

#if defined(VERBOSE) && defined(NX_DEBUG) && (1)
	if (NX_SDMMC_STATUS_NOERROR == status) {
		ERROR("\t NX_SDMMC_SendStatus : idx:0x%08X, arg:0x%08X, resp:0x%08X\r\n",
				cmd.cmdidx, cmd.arg, cmd.response[0]);

		if (cmd.response[0] & (1UL<<31))
			ERROR("\t\t ERROR : OUT_OF_RANGE\r\n");
		if (cmd.response[0] & (1UL<<30))
			ERROR("\t\t ERROR : ADDRESS_ERROR\r\n");
		if (cmd.response[0] & (1UL<<29))
			ERROR("\t\t ERROR : BLOCK_LEN_ERROR\r\n");
		if (cmd.response[0] & (1UL<<28))
			ERROR("\t\t ERROR : ERASE_SEQ_ERROR\r\n");
		if (cmd.response[0] & (1UL<<27))
			ERROR("\t\t ERROR : ERASE_PARAM\r\n");
		if (cmd.response[0] & (1UL<<26))
			ERROR("\t\t ERROR : WP_VIOLATION\r\n");
		if (cmd.response[0] & (1UL<<24))
			ERROR("\t\t ERROR : LOCK_UNLOCK_FAILED\r\n");
		if (cmd.response[0] & (1UL<<23))
			ERROR("\t\t ERROR : COM_CRC_ERROR\r\n");
		if (cmd.response[0] & (1UL<<22))
			ERROR("\t\t ERROR : ILLEGAL_COMMAND\r\n");
		if (cmd.response[0] & (1UL<<21))
			ERROR("\t\t ERROR : CARD_ECC_FAILED\r\n");
		if (cmd.response[0] & (1UL<<20))
			ERROR("\t\t ERROR : Internal Card Controller ERROR\r\n");
		if (cmd.response[0] & (1UL<<19))
			ERROR("\t\t ERROR : General Error\r\n");
		if (cmd.response[0] & (1UL<<17))
			ERROR("\t\t ERROR : Deferred Response\r\n");
		if (cmd.response[0] & (1UL<<16))
			ERROR("\t\t ERROR : CID/CSD_OVERWRITE_ERROR\r\n");
		if (cmd.response[0] & (1UL<<15))
			ERROR("\t\t ERROR : WP_ERASE_SKIP\r\n");
		if (cmd.response[0] & (1UL<<3))
			ERROR("\t\t ERROR : AKE_SEQ_ERROR\r\n");

		switch ((cmd.response[0]>>9) & 0xF) {
		case  0:
			ERROR("\t\t CURRENT_STATE : Idle\r\n");
			break;
		case  1:
			ERROR("\t\t CURRENT_STATE : Ready\r\n");
			break;
		case  2:
			ERROR("\t\t CURRENT_STATE : Identification\r\n");
			break;
		case  3:
			ERROR("\t\t CURRENT_STATE : Standby\r\n");
			break;
		case  4:
			ERROR("\t\t CURRENT_STATE : Transfer\r\n");
			break;
		case  5:
			ERROR("\t\t CURRENT_STATE : Data\r\n");
			break;
		case  6:
			ERROR("\t\t CURRENT_STATE : Receive\r\n");
			break;
		case  7:
			ERROR("\t\t CURRENT_STATE : Programming\r\n");
			break;
		case  8:
			ERROR("\t\t CURRENT_STATE : Disconnect\r\n");
			break;
		case  9:
			ERROR("\t\t CURRENT_STATE : Sleep\r\n");
			break;
		default:
			ERROR("\t\t CURRENT_STATE : Reserved\r\n");
			break;
		}
	}
#endif

	return status;
}

/*----------------------------------------------------------------------------*/
static unsigned int nx_sdmmc_sendcommand(
			struct cardstatus *pcardstatus,
			struct nx_sdmmc_command *pcommand)
{
	unsigned int status;

	status = nx_sdmmc_sendcommandinternal(pcardstatus, pcommand);
	if (NX_SDMMC_STATUS_NOERROR != status)
		nx_sdmmc_sendstatus(pcardstatus);

	return status;
}

/*----------------------------------------------------------------------------*/
static unsigned int nx_sdmmc_sendappcommand(struct cardstatus *pcardstatus,
		struct nx_sdmmc_command *pcommand)
{
	unsigned int status;
	struct nx_sdmmc_command cmd;

	cmd.cmdidx	= APP_CMD;
	cmd.arg		= pcardstatus->rca;
	cmd.flag	= NX_SDXC_CMDFLAG_STARTCMD |
			NX_SDXC_CMDFLAG_WAITPRVDAT |
			NX_SDXC_CMDFLAG_CHKRSPCRC |
			NX_SDXC_CMDFLAG_SHORTRSP;

	status = nx_sdmmc_sendcommandinternal(pcardstatus, &cmd);
	if (NX_SDMMC_STATUS_NOERROR == status)
		nx_sdmmc_sendcommand(pcardstatus, pcommand);

	return status;
}


/*----------------------------------------------------------------------------*/
static bool nx_sdmmc_identifycard(struct cardstatus *pcardstatus)
{
	int timeout;
	unsigned int HCS, RCA;
	enum NX_SDMMC_CARDTYPE cardtype = NX_SDMMC_CARDTYPE_UNKNOWN;
	struct nx_sdmmc_command cmd;
	volatile struct nx_sdmmc_registerset * const psdxcreg =
					pgsdxcreg[pcardstatus->sdport];

	if (false == nx_sdmmc_setclock(pcardstatus, true, 400000))
		return false;

	/* Data Bus Width : 0(1-bit), 1(4-bit) */
	psdxcreg->ctype = 0;

	pcardstatus->rca = 0;

	/*----------------------------------------------------------------------
	 *	Identify SD/MMC card
	 *----------------------------------------------------------------------
	 * Go idle State
	 */
	cmd.cmdidx	= GO_IDLE_STATE;
	cmd.arg		= 0;
	cmd.flag	= NX_SDXC_CMDFLAG_STARTCMD |
			NX_SDXC_CMDFLAG_SENDINIT |
			NX_SDXC_CMDFLAG_STOPABORT;

	nx_sdmmc_sendcommand(pcardstatus, &cmd);

	/* Check SD Card Version */
	cmd.cmdidx	= SEND_IF_COND;
	cmd.arg		= (1<<8) | 0xAA;	/* argument = VHS : 2.7~3.6V and Check Pattern(0xAA) */
	cmd.flag	= NX_SDXC_CMDFLAG_STARTCMD |
			NX_SDXC_CMDFLAG_WAITPRVDAT |
			NX_SDXC_CMDFLAG_CHKRSPCRC |
			NX_SDXC_CMDFLAG_SHORTRSP;

	if (NX_SDMMC_STATUS_NOERROR ==
			nx_sdmmc_sendcommandinternal(pcardstatus, &cmd)) {
		/* Ver 2.0 or later SD Memory Card */
		if (cmd.response[0] != ((1<<8) | 0xAA))
			return false;

		HCS = 1<<30;
		VERBOSE("Ver 2.0 or later SD Memory Card\r\n");
	} else {
		/* voltage mismatch or Ver 1.X SD Memory Card
		 * or not SD Memory Card
		 */
		HCS = 0;
		VERBOSE("voltage mismatch or Ver 1.X SD Memory Card or not SD Memory Card\r\n");
	}

	/*----------------------------------------------------------------------
	 * voltage validation
	 */
	timeout = NX_SDMMC_TIMEOUT_IDENTIFY;

	cmd.cmdidx	= APP_CMD;
	cmd.arg		= pcardstatus->rca;
	cmd.flag	= NX_SDXC_CMDFLAG_STARTCMD |
			NX_SDXC_CMDFLAG_WAITPRVDAT |
			NX_SDXC_CMDFLAG_CHKRSPCRC |
			NX_SDXC_CMDFLAG_SHORTRSP;

	if (NX_SDMMC_STATUS_NOERROR ==
			nx_sdmmc_sendcommandinternal(pcardstatus, &cmd)) {
		/*--------------------------------------------------------------
		 * SD memory card
		 */
#define FAST_BOOT	(1<<29)

		cmd.cmdidx	= SD_SEND_OP_COND;
		cmd.arg		= (HCS | FAST_BOOT | 0x00FC0000);/* 3.0 ~ 3.6V */
		cmd.flag	= NX_SDXC_CMDFLAG_STARTCMD |
				NX_SDXC_CMDFLAG_WAITPRVDAT |
				NX_SDXC_CMDFLAG_SHORTRSP;

		if (NX_SDMMC_STATUS_NOERROR !=
			nx_sdmmc_sendcommandinternal(pcardstatus, &cmd))
			return false;

		/* Wait until card has finished the power up routine */
		while (0 == (cmd.response[0] & (1UL<<31))) {

			if (NX_SDMMC_STATUS_NOERROR !=
				nx_sdmmc_sendappcommand(pcardstatus, &cmd))
				return false;

			if (timeout-- <= 0) {
				ERROR("NX_SDMMC_IdentifyCard : ERROR - Time-Out to wait power up for SD.\r\n");
				return false;
			}
		}

		VERBOSE("--> Found SD Memory Card.\r\n");
		VERBOSE("--> SD_SEND_OP_COND Response = 0x%08X\r\n",
							cmd.response[0]);

		cardtype	= NX_SDMMC_CARDTYPE_SDMEM;
		RCA		= 0;
	} else {
		/*--------------------------------------------------------------
		 * MMC memory card
		 */
		cmd.cmdidx	= GO_IDLE_STATE;
		cmd.arg		= 0;
		cmd.flag	= NX_SDXC_CMDFLAG_STARTCMD |
				NX_SDXC_CMDFLAG_SENDINIT |
				NX_SDXC_CMDFLAG_STOPABORT;

		nx_sdmmc_sendcommand(pcardstatus, &cmd);

		do {
			cmd.cmdidx	= SEND_OP_COND;
			cmd.arg		= 0x40FC0000;	/* MMC High Capacity -_-??? */
			cmd.flag	= NX_SDXC_CMDFLAG_STARTCMD |
					NX_SDXC_CMDFLAG_WAITPRVDAT |
					NX_SDXC_CMDFLAG_SHORTRSP;
			if (NX_SDMMC_STATUS_NOERROR !=
				nx_sdmmc_sendcommand(pcardstatus, &cmd))
				return false;

			if (timeout-- <= 0) {
				ERROR("NX_SDMMC_IdentifyCard : ERROR - Time-Out to wait power-up for MMC.\r\n");
				return false;
			}
		/* Wait until card has finished the power up routine */
		} while (0 == (cmd.response[0] & (1UL<<31)));

		VERBOSE("--> Found MMC Memory Card.\r\n");
		VERBOSE("--> SEND_OP_COND Response = 0x%08X\r\n", cmd.response[0]);

		cardtype	= NX_SDMMC_CARDTYPE_MMC;
		RCA		= 2<<16;
	}


	pcardstatus->bhighcapacity =
			(cmd.response[0] & (1<<30)) ? true : false;

	if (pcardstatus->bhighcapacity)
		VERBOSE("--> High Capacity Memory Card.\r\n");

	/*----------------------------------------------------------------------
	 * Get CID
	 */
	cmd.cmdidx	= ALL_SEND_CID;
	cmd.arg		= 0;
	cmd.flag	= NX_SDXC_CMDFLAG_STARTCMD |
			NX_SDXC_CMDFLAG_WAITPRVDAT |
			NX_SDXC_CMDFLAG_CHKRSPCRC |
			NX_SDXC_CMDFLAG_LONGRSP;
	if (NX_SDMMC_STATUS_NOERROR !=
			nx_sdmmc_sendcommand(pcardstatus, &cmd))
		return false;

	/*----------------------------------------------------------------------
	 * Get RCA and change to Stand-by State in data transfer mode
	 */
	cmd.cmdidx	= (cardtype == NX_SDMMC_CARDTYPE_SDMEM) ?
					SEND_RELATIVE_ADDR : SET_RELATIVE_ADDR;
	cmd.arg		= RCA;
	cmd.flag	= NX_SDXC_CMDFLAG_STARTCMD |
			NX_SDXC_CMDFLAG_WAITPRVDAT |
			NX_SDXC_CMDFLAG_CHKRSPCRC |
			NX_SDXC_CMDFLAG_SHORTRSP;
	if (NX_SDMMC_STATUS_NOERROR !=
			nx_sdmmc_sendcommand(pcardstatus, &cmd))
		return false;

	if (cardtype == NX_SDMMC_CARDTYPE_SDMEM)
		pcardstatus->rca = cmd.response[0] & 0xFFFF0000;
	else
		pcardstatus->rca = RCA;

	VERBOSE("RCA = 0x%08X\r\n", pcardstatus->rca);

	pcardstatus->cardtype = cardtype;

	return true;
}

/*----------------------------------------------------------------------------*/
static bool nx_sdmmc_selectcard(struct cardstatus *pcardstatus)
{
	unsigned int status;
	struct nx_sdmmc_command cmd;

	cmd.cmdidx	= SELECT_CARD;
	cmd.arg		= pcardstatus->rca;
	cmd.flag	= NX_SDXC_CMDFLAG_STARTCMD |
			NX_SDXC_CMDFLAG_WAITPRVDAT |
			NX_SDXC_CMDFLAG_CHKRSPCRC |
			NX_SDXC_CMDFLAG_SHORTRSP;

	status = nx_sdmmc_sendcommand(pcardstatus, &cmd);

	return (NX_SDMMC_STATUS_NOERROR == status) ? true : false;
}

/*----------------------------------------------------------------------------*/
static bool nx_sdmmc_setcarddetectpullup(
		struct cardstatus *pcardstatus,
		bool benb)
{
	unsigned int status;
	struct nx_sdmmc_command cmd;

	cmd.cmdidx	= SET_CLR_CARD_DETECT;
	cmd.arg		= (benb) ? 1 : 0;
	cmd.flag	= NX_SDXC_CMDFLAG_STARTCMD |
			NX_SDXC_CMDFLAG_WAITPRVDAT |
			NX_SDXC_CMDFLAG_CHKRSPCRC |
			NX_SDXC_CMDFLAG_SHORTRSP;

	status = nx_sdmmc_sendappcommand(pcardstatus, &cmd);

	return (NX_SDMMC_STATUS_NOERROR == status) ? true : false;
}

/*----------------------------------------------------------------------------*/
static bool nx_sdmmc_setbuswidth(
		struct cardstatus *pcardstatus,
		unsigned int buswidth)
{
	unsigned int status;
	struct nx_sdmmc_command cmd;
	volatile struct nx_sdmmc_registerset * const psdxcreg =
					pgsdxcreg[pcardstatus->sdport];

	if (pcardstatus->cardtype == NX_SDMMC_CARDTYPE_SDMEM) {
		cmd.cmdidx	= SET_BUS_WIDTH;
		cmd.arg		= (buswidth>>1);
		cmd.flag	= NX_SDXC_CMDFLAG_STARTCMD |
				NX_SDXC_CMDFLAG_WAITPRVDAT |
				NX_SDXC_CMDFLAG_CHKRSPCRC |
				NX_SDXC_CMDFLAG_SHORTRSP;
		status = nx_sdmmc_sendappcommand(pcardstatus, &cmd);
	} else {
		/* ExtCSD[183] : BUS_WIDTH <= 0 : 1-bit, 1 : 4-bit, 2 : 8-bit */
		cmd.cmdidx	= SWITCH_FUNC;
		cmd.arg		=	(3<<24) |
					(183<<16) |
					((buswidth>>2)<<8) |
					(0<<0);
		cmd.flag	= NX_SDXC_CMDFLAG_STARTCMD |
				NX_SDXC_CMDFLAG_WAITPRVDAT |
				NX_SDXC_CMDFLAG_CHKRSPCRC |
				NX_SDXC_CMDFLAG_SHORTRSP;
		status = nx_sdmmc_sendcommand(pcardstatus, &cmd);
	}

	if (NX_SDMMC_STATUS_NOERROR != status)
		return false;

	/* 0 : 1-bit mode, 1 : 4-bit mode */
	psdxcreg->ctype = buswidth >> 2;

	return true;
}

/*----------------------------------------------------------------------------*/
static bool nx_sdmmc_setblocklength(
		struct cardstatus *pcardstatus,
		unsigned int blocklength)
{
	unsigned int status;
	struct nx_sdmmc_command cmd;

	volatile struct nx_sdmmc_registerset * const psdxcreg =
				pgsdxcreg[pcardstatus->sdport];

	cmd.cmdidx	= SET_BLOCKLEN;
	cmd.arg		= blocklength;
	cmd.flag	= NX_SDXC_CMDFLAG_STARTCMD |
			NX_SDXC_CMDFLAG_WAITPRVDAT |
			NX_SDXC_CMDFLAG_CHKRSPCRC |
			NX_SDXC_CMDFLAG_SHORTRSP;
	status = nx_sdmmc_sendcommand(pcardstatus, &cmd);

	if (NX_SDMMC_STATUS_NOERROR != status)
		return false;

	psdxcreg->blksiz = blocklength;

	return true;
}

/*----------------------------------------------------------------------------*/
bool nx_sdmmc_init(struct cardstatus *pcardstatus)
{
	volatile struct nx_sdmmc_registerset * const psdxcreg =
					pgsdxcreg[pcardstatus->sdport];
	struct nx_clkgen_registerset * const psdclkgenreg =
					pgsdclkgenreg[pcardstatus->sdport];
#if 1
	struct nx_clkinfo_sdmmc clkinfo;
	bool ret;

	clkinfo.npllnum = NX_CLKSRC_SDMMC;
#ifdef QUICKBOOT
	clkinfo.nfreqhz = 100000000;
#else
	clkinfo.nfreqhz = 25000000;
#endif

	ret = nx_sdmmc_getclkparam(&clkinfo);
	if (ret == false)
		ERROR("get clock param fail.\r\n");
#endif

	/* CLKGEN */
	psdclkgenreg->clkenb = NX_PCLKMODE_ALWAYS<<3 | NX_BCLKMODE_DYNAMIC<<0;

	psdclkgenreg->clkgen[0] =
		(psdclkgenreg->clkgen[0] & ~(0x7<<2 | 0xff<<5))
		| (clkinfo.npllnum<<2)			/* set clock source */
		| ((clkinfo.nclkgendiv-1)<<5)		/* set clock divisor */
		| (0ul<<1);				/* set clock invert */

	psdclkgenreg->clkenb |= 0x1ul<<2;	/* clock generation enable */

	nx_setresetcon(sdresetnum[pcardstatus->sdport], true);	/* reset on */
	nx_setresetcon(sdresetnum[pcardstatus->sdport], false);	/* reset negate */

	psdxcreg->pwren = 0<<0;	/* set power disable */

	/*	pSDXCReg->UHS_REG |= 1<<0;*/		/* for DDR mode */

	psdxcreg->clkena = NX_SDXC_CLKENA_LOWPWR;	/* low power mode & clock disable */
	psdxcreg->clkctrl =
		0<<24 |	/* sample clock phase shift 0:0 1:90 2:180 3:270 */
		2<<16 |	/* drive clock phase shift 0:0 1:90 2:180 3:270 */
		0<<8 |	/* sample clock delay */
		0<<0;	/* drive clock delay */

	psdxcreg->clksrc = 0;	/* prescaler 0 */

	psdxcreg->clkdiv = (clkinfo.nclkgendiv>>1);

	/* fifo mode, not read wait(only use sdio mode) */
	psdxcreg->ctrl &= ~(NX_SDXC_CTRL_DMAMODE_EN | NX_SDXC_CTRL_READWAIT);

	/* Reset the controller & DMA interface & FIFO */
	psdxcreg->ctrl = NX_SDXC_CTRL_DMARST |
			NX_SDXC_CTRL_FIFORST |
			NX_SDXC_CTRL_CTRLRST;
	while (psdxcreg->ctrl &
			(NX_SDXC_CTRL_DMARST |
			NX_SDXC_CTRL_FIFORST |
			NX_SDXC_CTRL_CTRLRST))
		;

	psdxcreg->pwren = 0x1<<0;	/* Set Power Enable */


	/* Data Timeout = 0xFFFFFF, Response Timeout = 0x64 */
	psdxcreg->tmout = (0xFFFFFFU << 8) | (0x64 << 0);

	/* Data Bus Width : 0(1-bit), 1(4-bit) */
	psdxcreg->ctype = 0;

	/* Block size */
	psdxcreg->blksiz = BLOCK_LENGTH;

	/* Issue when RX FIFO Count >= 8 x 4 bytes & TX FIFO Count <= 8 x 4 bytes */
	psdxcreg->fifoth =	((8-1)<<16) |		/* Rx threshold */
				(8<<0)<<0;		/* Tx threshold */

	/* Mask & Clear All interrupts */
	psdxcreg->intmask = 0;
	psdxcreg->rintsts = 0xffffffff;

	return true;
}

/*----------------------------------------------------------------------------*/
bool	nx_sdmmc_terminate(struct cardstatus *pcardstatus)
{
#ifndef QUICKBOOT
	volatile struct nx_sdmmc_registerset * const psdxcreg =
				pgsdxcreg[pcardstatus->sdport];

	psdxcreg->rintsts = 0xffffffff;

	/* Reset the controller & DMA interface & FIFO */
	psdxcreg->ctrl = NX_SDXC_CTRL_DMARST |
			NX_SDXC_CTRL_FIFORST |
			NX_SDXC_CTRL_CTRLRST;
	while (psdxcreg->ctrl &
			(NX_SDXC_CTRL_DMARST |
			NX_SDXC_CTRL_FIFORST |
			NX_SDXC_CTRL_CTRLRST))
		;

	pgsdclkgenreg[pcardstatus->sdport]->clkenb = 0;

	nx_setresetcon(sdresetnum[pcardstatus->sdport], true);	/* reset on */
#endif

	return true;
}

/*----------------------------------------------------------------------------*/
bool nx_sdmmc_open(struct cardstatus *pcardstatus)
{
	/*----------------------------------------------------------------------
	 * card identification mode : Identify & Initialize
	 */
	if (false == nx_sdmmc_identifycard(pcardstatus)) {
		NOTICE("Card Identify Failure\r\n");
		return false;
	}

	/*----------------------------------------------------------------------
	 * data transfer mode : Stand-by state
	 */
#ifdef QUICKBOOT
	if (false == nx_sdmmc_setclock(pcardstatus, true, 100000000))
#else
	if (false == nx_sdmmc_setclock(pcardstatus, true, 25000000))
#endif
		return false;

	if (false == nx_sdmmc_selectcard(pcardstatus))
		return false;

	/*----------------------------------------------------------------------
	 * data transfer mode : Transfer state
	 */
	if (pcardstatus->cardtype == NX_SDMMC_CARDTYPE_SDMEM)
		nx_sdmmc_setcarddetectpullup(pcardstatus, false);

	if (false == nx_sdmmc_setblocklength(pcardstatus, BLOCK_LENGTH))
		return false;

	nx_sdmmc_setbuswidth(pcardstatus, 4);

	return true;
}

/*----------------------------------------------------------------------------*/
bool nx_sdmmc_close(struct cardstatus *pcardstatus)
{
	nx_sdmmc_setclock(pcardstatus, false, 400000);
	return true;
}

/*----------------------------------------------------------------------------*/
static bool nx_sdmmc_readsectordata(
		struct cardstatus *pcardstatus,
		unsigned int numberofsector,
		unsigned int *pdwbuffer)
{
	unsigned int	count;
	volatile struct nx_sdmmc_registerset * const psdxcreg =
		pgsdxcreg[pcardstatus->sdport];

	count = numberofsector * BLOCK_LENGTH;

	while (0 < count) {
		if ((psdxcreg->rintsts & NX_SDXC_RINTSTS_RXDR)
				|| (psdxcreg->rintsts & NX_SDXC_RINTSTS_DTO)) {
			unsigned int fsize, timeout = NX_SDMMC_TIMEOUT;

			while ((psdxcreg->status & NX_SDXC_STATUS_FIFOEMPTY) &&
					timeout--)
				;
			if (0 == timeout)
				break;
			fsize = (psdxcreg->status & NX_SDXC_STATUS_FIFOCOUNT)
						>> 17;
			count -= (fsize * 4);
			while (fsize) {
				*pdwbuffer++ = psdxcreg->data;
				fsize--;
			}

			psdxcreg->rintsts = NX_SDXC_RINTSTS_RXDR;
		}

		/* Check Errors */
		if (psdxcreg->rintsts & (NX_SDXC_RINTSTS_DRTO |
					NX_SDXC_RINTSTS_EBE |
					NX_SDXC_RINTSTS_SBE |
					NX_SDXC_RINTSTS_DCRC)) {
			ERROR("Read left = %d\r\n", count);

			if (psdxcreg->rintsts & NX_SDXC_RINTSTS_DRTO)
				ERROR("ERROR : NX_SDMMC_ReadSectors - NX_SDXC_RINTSTS_DRTO\r\n");
			if (psdxcreg->rintsts & NX_SDXC_RINTSTS_EBE)
				ERROR("ERROR : NX_SDMMC_ReadSectors - NX_SDXC_RINTSTS_EBE\r\n");
			if (psdxcreg->rintsts & NX_SDXC_RINTSTS_SBE)
				ERROR("ERROR : NX_SDMMC_ReadSectors - NX_SDXC_RINTSTS_SBE\r\n");
			if (psdxcreg->rintsts & NX_SDXC_RINTSTS_DCRC)
				ERROR("ERROR : NX_SDMMC_ReadSectors - NX_SDXC_RINTSTS_DCRC\r\n");

			return false;
		}

		if (psdxcreg->rintsts & NX_SDXC_RINTSTS_DTO) {
			if (count == 0) {
				psdxcreg->rintsts = NX_SDXC_RINTSTS_DTO;
				break;
			}
		}

		if (psdxcreg->rintsts & NX_SDXC_RINTSTS_HTO) {
			ERROR("ERROR : NX_SDMMC_ReadSectors - NX_SDXC_RINTSTS_HTO\r\n");
			psdxcreg->rintsts = NX_SDXC_RINTSTS_HTO;
		}
	}

	psdxcreg->rintsts = NX_SDXC_RINTSTS_DTO;

	return true;
}

/*----------------------------------------------------------------------------*/
bool nx_sdmmc_readsectors(
		struct cardstatus *pcardstatus,
		unsigned int sectornum,
		unsigned int numberofsector,
		unsigned int *pdwbuffer)
{
	bool	result = false;
	unsigned int	status;
	unsigned int	response;
	struct nx_sdmmc_command cmd;
	volatile struct nx_sdmmc_registerset * const psdxcreg =
		pgsdxcreg[pcardstatus->sdport];

	while (psdxcreg->status &
			(NX_SDXC_STATUS_DATABUSY | NX_SDXC_STATUS_FSMBUSY))
		;

	/*---------------------------------------------------------------------
	 * wait until 'Ready for data' is set and card is in transfer state.
	 */
	do {
		cmd.cmdidx	= SEND_STATUS;
		cmd.arg		= pcardstatus->rca;
		cmd.flag	= NX_SDXC_CMDFLAG_STARTCMD |
				NX_SDXC_CMDFLAG_CHKRSPCRC |
				NX_SDXC_CMDFLAG_SHORTRSP;
		status = nx_sdmmc_sendcommand(pcardstatus, &cmd);
		if (NX_SDMMC_STATUS_NOERROR != status)
			goto End;
	} while (!((cmd.response[0] & (1<<8)) &&
				(((cmd.response[0]>>9) & 0xF) == 4)));

	/* Set byte count */
	psdxcreg->bytcnt = numberofsector * BLOCK_LENGTH;

	/*----------------------------------------------------------------------
	 * Send command
	 */
	if (numberofsector > 1) {
		cmd.cmdidx	= READ_MULTIPLE_BLOCK;
		cmd.flag	= NX_SDXC_CMDFLAG_STARTCMD |
				NX_SDXC_CMDFLAG_WAITPRVDAT |
				NX_SDXC_CMDFLAG_CHKRSPCRC |
				NX_SDXC_CMDFLAG_SHORTRSP |
				NX_SDXC_CMDFLAG_BLOCK |
				NX_SDXC_CMDFLAG_RXDATA |
				NX_SDXC_CMDFLAG_SENDAUTOSTOP;
	} else {
		cmd.cmdidx	= READ_SINGLE_BLOCK;
		cmd.flag	= NX_SDXC_CMDFLAG_STARTCMD |
				NX_SDXC_CMDFLAG_WAITPRVDAT |
				NX_SDXC_CMDFLAG_CHKRSPCRC |
				NX_SDXC_CMDFLAG_SHORTRSP |
				NX_SDXC_CMDFLAG_BLOCK |
				NX_SDXC_CMDFLAG_RXDATA;
	}
	cmd.arg		= (pcardstatus->bhighcapacity) ?
					sectornum : sectornum * BLOCK_LENGTH;

	status = nx_sdmmc_sendcommand(pcardstatus, &cmd);
	if (NX_SDMMC_STATUS_NOERROR != status)
		goto End;

	/*----------------------------------------------------------------------
	 * Read Data
	 */
	if (true == nx_sdmmc_readsectordata(pcardstatus,
					numberofsector, pdwbuffer)) {
		if (numberofsector > 1) {
			/* Wait until the Auto-stop command has been finished.*/
			while (0 == (psdxcreg->rintsts & NX_SDXC_RINTSTS_ACD))
				;

			/* Get Auto-stop response and then check it. */
			response = psdxcreg->resp1;
			if (response & 0xFDF98008) {
				ERROR("ERROR : NX_SDMMC_ReadSectors - "\
					"Auto Stop Response Failed = 0x%08X\r\n",
						response);
				/* goto End; */
			}
		}

		result = true;
	}

End:
	if (false == result) {
		cmd.cmdidx	= STOP_TRANSMISSION;
		cmd.arg		= 0;
		cmd.flag	=	NX_SDXC_CMDFLAG_STARTCMD |
					NX_SDXC_CMDFLAG_CHKRSPCRC |
					NX_SDXC_CMDFLAG_SHORTRSP |
					NX_SDXC_CMDFLAG_STOPABORT;
		nx_sdmmc_sendcommandinternal(pcardstatus, &cmd);

		if (0 == (psdxcreg->status & NX_SDXC_STATUS_FIFOEMPTY)) {
			psdxcreg->ctrl = NX_SDXC_CTRL_FIFORST;
			while (psdxcreg->ctrl & NX_SDXC_CTRL_FIFORST)
				;
		}
	}

	return result;
}


/*
 * sdmmc 0		sdmmc 1			sdmmc 2
 * clk  a 29 1 gpio:0	clk  d 22 1 gpio:0	clk  c 18 2 gpio:1
 * cmd  a 31 1 gpio:0	cmd  d 23 1 gpio:0	cmd  c 19 2 gpio:1
 * dat0 b  1 1 gpio:0	dat0 d 24 1 gpio:0	dat0 c 20 2 gpio:1
 * dat1 b  3 1 gpio:0	dat1 d 25 1 gpio:0	dat1 c 21 2 gpio:1
 * dat2 b  5 1 gpio:0	dat2 d 26 1 gpio:0	dat2 c 22 2 gpio:1
 * dat3 b  7 1 gpio:0	dat3 d 27 1 gpio:0	dat3 c 23 2 gpio:1
 */
void nx_sdpadsetalt(unsigned int portnum)
{
	struct nx_gpio_registerset (*volatile const pgpio)[1] =
		(struct nx_gpio_registerset (*)[])PHY_BASEADDR_GPIOA_MODULE;

	if (portnum == 0) {
		volatile struct nx_gpio_registerset *pgpioa =
			(struct nx_gpio_registerset *)pgpio[NX_GPIO_GROUP_A];
		volatile struct nx_gpio_registerset *pgpiob =
			(struct nx_gpio_registerset *)pgpio[NX_GPIO_GROUP_B];
		volatile unsigned int *pgpioarega1 =
			(unsigned int *)&pgpio[NX_GPIO_GROUP_A]->gpioxaltfn[1];
		volatile unsigned int *pgpiobrega0 =
			(unsigned int *)&pgpio[NX_GPIO_GROUP_B]->gpioxaltfn[0];

		/* a 29, a 31 all alt is 1 */
		*pgpioarega1 = (*pgpioarega1 & ~0xcc000000) | 0x44000000;
		/* b 1, 3, 5, 7 all alt is 1 */
		*pgpiobrega0 = (*pgpiobrega0 & ~0x0000cccc) | 0x00004444;

		pgpioa->gpiox_slew                    &= ~(5<<29);
		pgpioa->gpiox_slew_disable_default    |=  (5<<29);
		pgpioa->gpiox_drv0                    |=  (5<<29);
		pgpioa->gpiox_drv0_disable_default    |=  (5<<29);
		pgpioa->gpiox_drv1                    |=  (5<<29);
		pgpioa->gpiox_drv1_disable_default    |=  (5<<29);
		pgpioa->gpiox_pullsel                 |=  (5<<29);
		pgpioa->gpiox_pullsel_disable_default |=  (5<<29);
	/* 	pgpioa->gpiox_pullenb                 |=  (5<<29); */
		pgpioa->gpiox_pullenb                 |=  (4<<29);         /* clk is not pull-up. */
		pgpioa->gpiox_pullenb_disable_default |=  (5<<29);

		pgpiob->gpiox_slew                    &= ~(0x55<<1);
		pgpiob->gpiox_slew_disable_default    |=  (0x55<<1);
		pgpiob->gpiox_drv0                    |=  (0x55<<1);
		pgpiob->gpiox_drv0_disable_default    |=  (0x55<<1);
		pgpiob->gpiox_drv1                    |=  (0x55<<1);
		pgpiob->gpiox_drv1_disable_default    |=  (0x55<<1);
		pgpiob->gpiox_pullsel                 |=  (0x55<<1);
		pgpiob->gpiox_pullsel_disable_default |=  (0x55<<1);
		pgpiob->gpiox_pullenb                 |=  (0x55<<1);
		pgpiob->gpiox_pullenb_disable_default |=  (0x55<<1);
	} else if (portnum == 1) {
		volatile struct nx_gpio_registerset *pgpiod =
			(struct nx_gpio_registerset *)pgpio[NX_GPIO_GROUP_D];
		register unsigned int *pgpiodrega1 =
			(unsigned int *)&pgpio[NX_GPIO_GROUP_D]->gpioxaltfn[1]; /* d 22, 23, 24, 25, 26, 27 */
		*pgpiodrega1 = (*pgpiodrega1 & ~0x00fff000) | 0x00555000;	/* all alt is 1 */
		pgpiod->gpiox_slew                    &= ~(0x3f<<22);
		pgpiod->gpiox_slew_disable_default    |=  (0x3f<<22);
		pgpiod->gpiox_drv0                    |=  (0x3f<<22);
		pgpiod->gpiox_drv0_disable_default    |=  (0x3f<<22);
		pgpiod->gpiox_drv1                    |=  (0x3f<<22);
		pgpiod->gpiox_drv1_disable_default    |=  (0x3f<<22);
		pgpiod->gpiox_pullsel                 |=  (0x3f<<22);
		pgpiod->gpiox_pullsel_disable_default |=  (0x3f<<22);
	/*	pgpiod->gpiox_pullenb                 |=  (0x3f<<22); */
		pgpiod->gpiox_pullenb                 |=  (0x3e<<22);      /* clk is not pull-up. */
		pgpiod->gpiox_pullenb_disable_default |=  (0x3f<<22);
	} else {
		volatile struct nx_gpio_registerset *pgpioc =
			(struct nx_gpio_registerset *)pgpio[NX_GPIO_GROUP_C];
		register unsigned int *pgpiocrega1 =
			(unsigned int *)&pgpio[NX_GPIO_GROUP_C]->gpioxaltfn[1]; /* c 18, 19, 20, 21, 22, 23 */
		*pgpiocrega1 = (*pgpiocrega1 & ~0x0000fff0) | 0x0000aaa0;	/* all alt is 2 */
		pgpioc->gpiox_slew                    &= ~(0x3f<<18);
		pgpioc->gpiox_slew_disable_default    |=  (0x3f<<18);
		pgpioc->gpiox_drv0                    |=  (0x3f<<18);
		pgpioc->gpiox_drv0_disable_default    |=  (0x3f<<18);
		pgpioc->gpiox_drv1                    |=  (0x3f<<18);
		pgpioc->gpiox_drv1_disable_default    |=  (0x3f<<18);
		pgpioc->gpiox_pullsel                 |=  (0x3f<<18);
		pgpioc->gpiox_pullsel_disable_default |=  (0x3f<<18);
	/*	pgpioc->gpiox_pullenb                 |=  (0x3f<<18); */
		pgpioc->gpiox_pullenb                 |=  (0x3e<<18);      /* clk is not pull-up. */
		pgpioc->gpiox_pullenb_disable_default |=  (0x3f<<18);
	}
}

#if 1
void nx_sdpadsetgpio(unsigned int portnum)
{
	struct nx_gpio_registerset (*volatile const pgpio)[1] =
		(struct nx_gpio_registerset (*)[])PHY_BASEADDR_GPIOA_MODULE;

	if (portnum == 0) {
		volatile struct nx_gpio_registerset *pgpioa =
			(struct nx_gpio_registerset *)pgpio[NX_GPIO_GROUP_A];
		volatile struct nx_gpio_registerset *pgpiob =
			(struct nx_gpio_registerset *)pgpio[NX_GPIO_GROUP_B];
		volatile unsigned int *pgpioarega1 =
			(unsigned int *)&pgpio[NX_GPIO_GROUP_A]->gpioxaltfn[1];
		volatile unsigned int *pgpiobrega0 =
			(unsigned int *)&pgpio[NX_GPIO_GROUP_B]->gpioxaltfn[0];
		*pgpioarega1 &= ~0xcc000000;	/* all gpio is 0 */
		*pgpiobrega0 &= ~0x0000cccc;
		pgpioa->gpiox_slew                    |=  (5<<29);
		pgpioa->gpiox_slew_disable_default    |=  (5<<29);
		pgpioa->gpiox_drv0                    &= ~(5<<29);
		pgpioa->gpiox_drv0_disable_default    |=  (5<<29);
		pgpioa->gpiox_drv1                    &= ~(5<<29);
		pgpioa->gpiox_drv1_disable_default    |=  (5<<29);
		pgpioa->gpiox_pullsel                 &= ~(5<<29);
		pgpioa->gpiox_pullsel_disable_default &= ~(5<<29);
		pgpioa->gpiox_pullenb                 &= ~(5<<29);
		pgpioa->gpiox_pullenb_disable_default &= ~(5<<29);

		pgpiob->gpiox_slew                    |=  (0x55<<1);
		pgpiob->gpiox_slew_disable_default    |=  (0x55<<1);
		pgpiob->gpiox_drv0                    &= ~(0x55<<1);
		pgpiob->gpiox_drv0_disable_default    |=  (0x55<<1);
		pgpiob->gpiox_drv1                    &= ~(0x55<<1);
		pgpiob->gpiox_drv1_disable_default    |=  (0x55<<1);
		pgpiob->gpiox_pullsel                 &= ~(0x55<<1);
		pgpiob->gpiox_pullsel_disable_default &= ~(0x55<<1);
		pgpiob->gpiox_pullenb                 &= ~(0x55<<1);
		pgpiob->gpiox_pullenb_disable_default &= ~(0x55<<1);
	} else if (portnum == 1) {
		volatile struct nx_gpio_registerset *pgpiod =
			(struct nx_gpio_registerset *)pgpio[NX_GPIO_GROUP_D];
		volatile unsigned int *pgpiodrega1 =
			(unsigned int *)&pgpio[NX_GPIO_GROUP_D]->gpioxaltfn[1]; /* d 22, 23, 24, 25, 26, 27 */
		*pgpiodrega1 = (*pgpiodrega1 & ~0x00fff000);	/* all gpio is 0 */
		pgpiod->gpiox_slew                    |=  (0x3f<<22);
		pgpiod->gpiox_slew_disable_default    |=  (0x3f<<22);
		pgpiod->gpiox_drv0                    &= ~(0x3f<<22);
		pgpiod->gpiox_drv0_disable_default    |=  (0x3f<<22);
		pgpiod->gpiox_drv1                    &= ~(0x3f<<22);
		pgpiod->gpiox_drv1_disable_default    |=  (0x3f<<22);
		pgpiod->gpiox_pullsel                 &= ~(0x3f<<22);
		pgpiod->gpiox_pullsel_disable_default &= ~(0x3f<<22);
		pgpiod->gpiox_pullenb                 &= ~(0x3f<<22);
		pgpiod->gpiox_pullenb_disable_default &= ~(0x3f<<22);
	} else {
		volatile struct nx_gpio_registerset *pgpioc =
			(struct nx_gpio_registerset *)pgpio[NX_GPIO_GROUP_C];
		volatile unsigned int *pgpiocrega1 =
			(unsigned int *)&pgpio[NX_GPIO_GROUP_C]->gpioxaltfn[1];
		*pgpiocrega1 = (*pgpiocrega1 & ~0x0000fff0) | 0x00005550;	/* all gpio is 1 */
		pgpioc->gpiox_slew                    |=  (0x3f<<18);
		pgpioc->gpiox_slew_disable_default    |=  (0x3f<<18);
		pgpioc->gpiox_drv0                    &= ~(0x3f<<18);
		pgpioc->gpiox_drv0_disable_default    |=  (0x3f<<18);
		pgpioc->gpiox_drv1                    &= ~(0x3f<<18);
		pgpioc->gpiox_drv1_disable_default    |=  (0x3f<<18);
		pgpioc->gpiox_pullsel                 &= ~(0x3f<<18);
		pgpioc->gpiox_pullsel_disable_default &= ~(0x3f<<18);
		pgpioc->gpiox_pullenb                 &= ~(0x3f<<18);
		pgpioc->gpiox_pullenb_disable_default &= ~(0x3f<<18);
	}
}
#endif
/*----------------------------------------------------------------------------*/
bool init_mmc(unsigned int portnum)
{
	struct cardstatus *pcardstatus;

	pcardstatus = &lcardstatus;

	pcardstatus->sdport = portnum;

	nx_sdpadsetalt(portnum);

	nx_sdmmc_init(pcardstatus);

	return true;
}

void deinit_mmc(unsigned int portnum)
{
	struct cardstatus *pcardstatus;

	pcardstatus = &lcardstatus;

	nx_sdmmc_close(pcardstatus);
	nx_sdmmc_terminate(pcardstatus);

	nx_sdpadsetgpio(pcardstatus->sdport);
}

/*----------------------------------------------------------------------------*/
bool load_mmc(unsigned int portnum,
		unsigned int startsector,
		unsigned int sectorcount,
		void *pmem)
{
	struct cardstatus *pcardstatus = &lcardstatus;
	bool	result = false;
	volatile struct nx_sdmmc_registerset * const psdxcreg =
		pgsdxcreg[pcardstatus->sdport];

	if (true != nx_sdmmc_open(pcardstatus)) {
		ERROR("cannot detect sdmmc\r\n");
		return false;
	}

	if (0 == (psdxcreg->status & NX_SDXC_STATUS_FIFOEMPTY)) {
		VERBOSE("fifo reset!!!\r\n");
		psdxcreg->ctrl = NX_SDXC_CTRL_FIFORST;
		while (psdxcreg->ctrl & NX_SDXC_CTRL_FIFORST)
			;
	}

	INFO("sd%d load image at %x sector, cnt:%x, target:%lx\r\n",
			portnum, startsector, sectorcount, (unsigned long)pmem);

	result = nx_sdmmc_readsectors(pcardstatus,
					startsector,
					sectorcount,
					(unsigned int *)pmem);
	if (result == false) {
		ERROR("image read failure\r\n");
	}

	return result;
}
