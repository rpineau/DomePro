//
// X2Dome implemntation
//

#include "x2dome.h"

X2Dome::X2Dome(const char* pszSelection, 
							 const int& nISIndex,
					SerXInterface*						pSerX,
					TheSkyXFacadeForDriversInterface*	pTheSkyXFacadeForDriversInterface,
					SleeperInterface*					pSleeper,
					BasicIniUtilInterface*			pIniUtil,
					LoggerInterface*					pLogger,
					MutexInterface*						pIOMutex,
					TickCountInterface*					pTickCount)
{

    m_nPrivateISIndex				= nISIndex;
	m_pSerX							= pSerX;
	m_pTheSkyXFacadeForDriversInterface = pTheSkyXFacadeForDriversInterface;
	m_pSleeper						= pSleeper;
	m_pIniUtil						= pIniUtil;
	m_pLogger						= pLogger;	
	m_pIOMutex						= pIOMutex;
	m_pTickCount					= pTickCount;

	m_bLinked = false;
    m_nLearningDomeCPR = NONE;
    m_bBattRequest = 0;

    m_DomePro.SetSerxPointer(pSerX);
    m_DomePro.setLogger(pLogger);

    if (m_pIniUtil)
    {   
    }
}


X2Dome::~X2Dome()
{
	if (m_pSerX)
		delete m_pSerX;
	if (m_pTheSkyXFacadeForDriversInterface)
		delete m_pTheSkyXFacadeForDriversInterface;
	if (m_pSleeper)
		delete m_pSleeper;
	if (m_pIniUtil)
		delete m_pIniUtil;
	if (m_pLogger)
		delete m_pLogger;
	if (m_pIOMutex)
		delete m_pIOMutex;
	if (m_pTickCount)
		delete m_pTickCount;

}


int X2Dome::establishLink(void)					
{
    int nErr;
    char szPort[DRIVER_MAX_STRING];

    X2MutexLocker ml(GetMutex());
    // get serial port device name
    portNameOnToCharPtr(szPort,DRIVER_MAX_STRING);
    nErr = m_DomePro.Connect(szPort);
    if(nErr)
        m_bLinked = false;
    else
        m_bLinked = true;

    m_bHasShutterControl = m_DomePro.hasShutterUnit();
	return nErr;
}

int X2Dome::terminateLink(void)					
{
    X2MutexLocker ml(GetMutex());
    m_DomePro.Disconnect();
	m_bLinked = false;
	return SB_OK;
}

 bool X2Dome::isLinked(void) const				
{
	return m_bLinked;
}


int X2Dome::queryAbstraction(const char* pszName, void** ppVal)
{
    *ppVal = NULL;

    if (!strcmp(pszName, LoggerInterface_Name))
        *ppVal = GetLogger();
    else if (!strcmp(pszName, ModalSettingsDialogInterface_Name))
        *ppVal = dynamic_cast<ModalSettingsDialogInterface*>(this);
    else if (!strcmp(pszName, X2GUIEventInterface_Name))
        *ppVal = dynamic_cast<X2GUIEventInterface*>(this);
    else if (!strcmp(pszName, SerialPortParams2Interface_Name))
        *ppVal = dynamic_cast<SerialPortParams2Interface*>(this);
    
    return SB_OK;
}

#pragma mark - UI binding

int X2Dome::execModalSettingsDialog()
{
    int nErr = SB_OK;
    X2ModalUIUtil uiutil(this, GetTheSkyXFacadeForDrivers());
    X2GUIInterface*					ui = uiutil.X2UI();
    X2GUIExchangeInterface*			dx = NULL;//Comes after ui is loaded
    bool bPressedOK = false;
    char szTmpBuf[SERIAL_BUFFER_SIZE];
    int nTmp = 0;
    double dTmp = 0;
    bool bTmp = false;

    if (NULL == ui)
        return ERR_POINTER;

    if ((nErr = ui->loadUserInterface("domepro.ui", deviceType(), m_nPrivateISIndex)))
        return nErr;

    if (NULL == (dx = uiutil.X2DX()))
        return ERR_POINTER;


    memset(szTmpBuf,0,SERIAL_BUFFER_SIZE);

    X2MutexLocker ml(GetMutex());

    // set controls state depending on the connection state
    if(m_bLinked) {
        // Az Motors
        dx->setEnabled(MOTOR_POLARITY, true);
        nErr = m_DomePro.getDomeAzMotorPolarity(nTmp);
        dx->setChecked(MOTOR_POLARITY,nTmp==POSITIVE?true:false);

        dx->setEnabled(OVER_CURRENT_PROTECTION, true);
        m_DomePro.getDomeAzimuthOCP_Limit(dTmp);
        dx->setPropertyDouble(OVER_CURRENT_PROTECTION, "value", dTmp);

        // Az Encoders
        dx->setEnabled(TICK_PER_REV, true);
        nErr = m_DomePro.getDomeAzCPR(nTmp);
        dx->setPropertyInt(TICK_PER_REV, "value", nTmp);

        dx->setEnabled(ROTATION_COAST, true);
        nErr = m_DomePro.getDomeAzCoast(dTmp);
        dx->setPropertyDouble(ROTATION_COAST, "value", dTmp);

        dx->setEnabled(LEARN_AZIMUTH_CPR_RIGHT, true);
        dx->setEnabled(LEARN_AZIMUTH_CPR_LEFT, true);
        dx->setPropertyString(L_CPR_VALUE, "text", ": not learned");
        dx->setPropertyString(R_CPR_VALUE, "text", ": not learned");

        dx->setEnabled(ENCODDER_POLARITY, true);
        nErr = m_DomePro.getDomeAzEncoderPolarity(nTmp);
        dx->setPropertyDouble(ENCODDER_POLARITY, "value", dTmp);

        dx->setEnabled(SET_AZIMUTH_CPR, true);

        // Homing
        dx->setEnabled(HOMING_DIR, true); // no corresponding function, need to ping Chris
        nErr = m_DomePro.getDomeHomeDirection(nTmp);
        printf("Homing direction = %d\n", nTmp);
        dx->setCurrentIndex(HOMING_DIR, nTmp-1);

        dx->setEnabled(HOME_POS, true);
        nErr = m_DomePro.getDomeHomeAz(dTmp);
        dx->setPropertyDouble(HOME_POS, "value", dTmp);

        dx->setEnabled(PARK_POS, true);
        nErr = m_DomePro.getDomeParkAz(dTmp);
        dx->setPropertyDouble(PARK_POS, "value", dTmp);

        dx->setEnabled(SHUTTER_BUTTON, true);
        dx->setEnabled(TIMEOUTS_BUTTON, true);
        dx->setEnabled(DIAG_BUTTON, true);

    }
    else { // not connected, disable all controls
        // Az motor
        dx->setEnabled(MOTOR_POLARITY, false);
        dx->setEnabled(OVER_CURRENT_PROTECTION, false);

        // Az Encoders
        dx->setEnabled(TICK_PER_REV, false);
        dx->setEnabled(ROTATION_COAST, false);

        dx->setEnabled(LEARN_AZIMUTH_CPR_RIGHT, false);
        dx->setEnabled(LEARN_AZIMUTH_CPR_LEFT, false);
        dx->setPropertyString(L_CPR_VALUE, "text", ": --");
        dx->setPropertyString(R_CPR_VALUE, "text", ": --");

        dx->setEnabled(ENCODDER_POLARITY, false);
        dx->setEnabled(SET_AZIMUTH_CPR, false);

        // Homing
        dx->setEnabled(HOMING_DIR, false);
        dx->setEnabled(HOME_POS, false);
        dx->setEnabled(PARK_POS, false);

        dx->setEnabled(SHUTTER_BUTTON, false);
        dx->setEnabled(TIMEOUTS_BUTTON, false);
        dx->setEnabled(DIAG_BUTTON, false);

    }

    m_nLearningDomeCPR = NONE;
    

    //Display the user interface
    m_nCurrentDialog = MAIN;
    if ((nErr = ui->exec(bPressedOK)))
        return nErr;

    //Retreive values from the user interface
    if (bPressedOK)
    {
        if(m_bLinked)
        {
            // read all controls and set new values
            // Az Motor
            bTmp = dx->isChecked(MOTOR_POLARITY);
            m_DomePro.setDomeAzMotorPolarity(bTmp?POSITIVE:NEGATIVE);

            dx->propertyDouble(OVER_CURRENT_PROTECTION, "value", dTmp);
            m_DomePro.setDomeAzimuthOCP_Limit(dTmp);

            // Az Encoders
            dx->propertyInt(TICK_PER_REV, "value", nTmp);
            m_DomePro.setDomeAzCPR(nTmp);

            dx->propertyDouble(ROTATION_COAST, "value", dTmp);
            m_DomePro.setDomeAzCoast(dTmp);

            bTmp = dx->isChecked(ENCODDER_POLARITY);
            m_DomePro.setDomeAzEncoderPolarity(bTmp?POSITIVE:NEGATIVE);

            // Homing
            nTmp = dx->currentIndex(HOMING_DIR);
            m_DomePro.setDomeHomeDirection(nTmp+1);

            dx->propertyDouble(HOME_POS, "value", dTmp);
            m_DomePro.setHomeAz(dTmp);

            dx->propertyDouble(PARK_POS, "value", dTmp);
            m_DomePro.setParkAz(dTmp);
        }
    }
    return nErr;

}

void X2Dome::uiEvent(X2GUIExchangeInterface* uiex, const char* pszEvent)
{

    switch(m_nCurrentDialog) {
        case MAIN:
            doMainDialogEvents(uiex, pszEvent);
            break;
        case SHUTTER:
            doShutterDialogEvents(uiex, pszEvent);
            break;
        case TIMEOUTS:
            doTimeoutsDialogEvents(uiex, pszEvent);
            break;
        case DIAG:
            doDiagDialogEvents(uiex, pszEvent);
            break;

    }


}
//
// Main setting ui
//

int X2Dome::doMainDialogEvents(X2GUIExchangeInterface* uiex, const char* pszEvent)
{
    bool bComplete = false;
    int nErr = SB_OK;
    char szErrorMessage[LOG_BUFFER_SIZE];
    char szTmpBuf[SERIAL_BUFFER_SIZE];

    int nTmp, nTmp2;
    bool bPressedOK = false;

    printf("[doMainDialogEvents] pszEvent : %s\n", pszEvent);

    if (!strcmp(pszEvent, "on_pushButtonCancel_clicked") && m_nLearningDomeCPR != NONE)
        m_DomePro.abortCurrentCommand();

    if (!strcmp(pszEvent, "on_timer"))
    {
        if(m_bLinked) {
            if(m_nLearningDomeCPR != NONE) {
                // are we still learning CPR ?
                bComplete = false;
                nErr = m_DomePro.isLearningCPRComplete(bComplete);
                if(nErr) {
                    uiex->setEnabled(LEARN_AZIMUTH_CPR_RIGHT, true);
                    uiex->setEnabled(LEARN_AZIMUTH_CPR_LEFT, true);
                    uiex->setEnabled(BUTTON_OK, true);
                    snprintf(szErrorMessage, LOG_BUFFER_SIZE, "Error learning dome CPR : Error %d", nErr);
                    uiex->messageBox("DomePro Learn CPR", szErrorMessage);
                    m_nLearningDomeCPR = NONE;
                    return nErr;
                }

                if(!bComplete) {
                    return nErr;
                }

                // enable "ok" and "Lean Azimuth CPR"
                uiex->setEnabled(LEARN_AZIMUTH_CPR_RIGHT, true);
                uiex->setEnabled(LEARN_AZIMUTH_CPR_LEFT, true);
                uiex->setEnabled(BUTTON_OK, true);
                // read guaged step per rev from dome
                switch (m_nLearningDomeCPR) {
                    case LEFT:
                        nTmp = m_DomePro.getLeftCPR();
                        snprintf(szTmpBuf, SERIAL_BUFFER_SIZE, "%d", nTmp);
                        uiex->setPropertyString(L_CPR_VALUE, "text", szTmpBuf);
                        break;

                    case RIGHT:
                        nTmp = m_DomePro.getRightCPR();
                        snprintf(szTmpBuf, SERIAL_BUFFER_SIZE, "%d", nTmp);
                        uiex->setPropertyString(R_CPR_VALUE, "text", szTmpBuf);
                        break;
                    default:
                        break;
                }
                m_nLearningDomeCPR = NONE;

            }
        }
    }

    if (!strcmp(pszEvent, LEARN_AZIMUTH_CPR_RIGHT_CLICKED) )
    {
        if(m_bLinked) {
            // disable "ok" and "Lean Azimuth CPR"
            uiex->setEnabled(LEARN_AZIMUTH_CPR_RIGHT, false);
            uiex->setEnabled(LEARN_AZIMUTH_CPR_LEFT, false);
            uiex->setEnabled(BUTTON_OK, false);
            m_DomePro.learnAzimuthCprRight();
            m_nLearningDomeCPR = RIGHT;
        }
    }

    if (!strcmp(pszEvent, LEARN_AZIMUTH_CPR_LEFT_CLICKED) )
    {
        if(m_bLinked) {
            // disable "ok" and "Lean Azimuth CPR"
            uiex->setEnabled(LEARN_AZIMUTH_CPR_RIGHT, false);
            uiex->setEnabled(LEARN_AZIMUTH_CPR_LEFT, false);
            uiex->setEnabled(BUTTON_OK, false);
            m_DomePro.learnAzimuthCprLeft();
            m_nLearningDomeCPR = LEFT;
        }
    }

    if (!strcmp(pszEvent, SET_CPR_FROM_GAUGED) )
    {
        if(m_bLinked) {
            nTmp = m_DomePro.getRightCPR();
            if(!nTmp) {
                snprintf(szErrorMessage, LOG_BUFFER_SIZE, "Error setting dome CPR , right value can't be 0");
                uiex->messageBox("DomePro Set CPR", szErrorMessage);
            }

            nTmp2 = m_DomePro.getLeftCPR();
            if(!nTmp) {
                snprintf(szErrorMessage, LOG_BUFFER_SIZE, "Error setting dome CPR , left value can't be 0");
                uiex->messageBox("DomePro Set CPR", szErrorMessage);
            }
            nTmp =  (int)floor( 0.5 +(nTmp + nTmp2)/2);
            nErr = m_DomePro.setDomeAzCPR(nTmp);
            if(nErr) {
                snprintf(szErrorMessage, LOG_BUFFER_SIZE, "Error setting dome CPR : Error %d", nErr);
                uiex->messageBox("DomePro Set CPR", szErrorMessage);
            }
            uiex->setPropertyInt(TICK_PER_REV, "value", nTmp);
        }
    }

    if (!strcmp(pszEvent, SHUTTER_CKICKED))
    {
        doDomeProShutter(bPressedOK);
    }

    if (!strcmp(pszEvent, TIMEOUTS_CKICKED))
    {
        doDomeProTimeouts(bPressedOK);
    }

    if (!strcmp(pszEvent, DIAG_CKICKED))
    {
        doDomeProDiag(bPressedOK);
    }

    return nErr;
}

//
// Shutter settings UI
//
int X2Dome::doDomeProShutter(bool& bPressedOK)
{
    int nErr = SB_OK;
    char szTmpBuf[SERIAL_BUFFER_SIZE];
    bool bTmp;
    int nTmp;
    double dTmp;

    X2ModalUIUtil uiutil(this, GetTheSkyXFacadeForDrivers());
    X2GUIInterface*                    ui = uiutil.X2UI();
    X2GUIExchangeInterface*            dx = NULL;

    bPressedOK = false;
    if (NULL == ui)
        return ERR_POINTER;
    nErr = ui->loadUserInterface("domeshutter.ui", deviceType(), m_nPrivateISIndex);
    if (nErr)
        return nErr;

    dx = uiutil.X2DX();
    if (NULL == dx)
        return ERR_POINTER;

    m_nCurrentDialog = SHUTTER;
    // sequencing
    if(m_bLinked) {
        dx->setEnabled(SINGLE_SHUTTER, true);
        dx->setEnabled(OPEN_FIRST, true);
        dx->setEnabled(CLOSE_FIRST, true);
        dx->setEnabled(INHIBIT_SIMULT, true);
        m_DomePro.getModel(szTmpBuf, SERIAL_BUFFER_SIZE);
        dx->setPropertyString(DOMEPRO_MODEL, "text", szTmpBuf);
        // sequencing
        if(m_DomePro.hasShutterUnit()) {
            dx->setEnabled(SINGLE_SHUTTER, true);
            m_DomePro.getDomeSingleShutterMode(bTmp);
            if(bTmp) {
                dx->setEnabled(OPEN_FIRST, false);
                dx->setEnabled(CLOSE_FIRST, false);
                dx->setEnabled(INHIBIT_SIMULT, false);

            }
            else { // 2 shutters
                dx->setEnabled(OPEN_FIRST, true);
                m_DomePro.getDomeShutterOpenFirst(nTmp);
                dx->setCurrentIndex(OPEN_FIRST, nTmp-1);
                dx->setEnabled(CLOSE_FIRST, true);
                m_DomePro.getDomeShutterCloseFirst(nTmp);
                dx->setCurrentIndex(OPEN_FIRST, nTmp-1);

                dx->setEnabled(INHIBIT_SIMULT, false); // no corresponding function, need to ping Chris

                dx->setEnabled(SHUTTER_OPERATE_AT_HOME, true);
                m_DomePro.getDomeShutOpOnHome(bTmp);
                dx->setChecked(SHUTTER_OPERATE_AT_HOME, bTmp);

                dx->setEnabled(HOME_ON_SHUTTER_CLOSE, true);
                m_DomePro.getHomeWithShutterClose(bTmp);
                dx->setChecked(HOME_ON_SHUTTER_CLOSE, bTmp);

                m_DomePro.getShutter1_LimitFaultCheckEnabled(bTmp);
                dx->setChecked(UPPER_SHUTTER_LIMIT_CHECK, bTmp);
                m_DomePro.getShutter2_LimitFaultCheckEnabled(bTmp);
                dx->setChecked(LOWER_SHUTTER_LIMIT_CHECK, bTmp);

                m_DomePro.getDomeShutter1_OCP_Limit(dTmp);
                dx->setPropertyDouble(SHUTTER1_OCP, "value", dTmp);
                m_DomePro.getDomeShutter2_OCP_Limit(dTmp);
                dx->setPropertyDouble(SHUTTER2_OCP, "value", dTmp);
            }
        }
        else { // no shutter unit
            dx->setChecked(SINGLE_SHUTTER,false);
            dx->setEnabled(OPEN_FIRST, false);
            dx->setEnabled(CLOSE_FIRST, false);
            dx->setEnabled(INHIBIT_SIMULT, false);
            dx->setPropertyString(DOMEPRO_MODEL, "text", "");
        }

    } else {
        dx->setEnabled(SINGLE_SHUTTER, false);
        dx->setEnabled(OPEN_FIRST, false);
        dx->setEnabled(CLOSE_FIRST, false);
        dx->setEnabled(INHIBIT_SIMULT, false);
    }

    nErr = ui->exec(bPressedOK);
    if (nErr )
        return nErr;

    //Retreive values from the user interface
    if (bPressedOK)
    {
        if(m_bLinked)
        {
            // read all controls and set new values
            bTmp = dx->isChecked(SINGLE_SHUTTER);
            m_DomePro.setDomeSingleShutterMode(bTmp);

            if(!bTmp) {
                nTmp = dx->currentIndex(OPEN_FIRST);
                m_DomePro.setDomeShutterOpenFirst(nTmp+1);

                nTmp = dx->currentIndex(CLOSE_FIRST);
                m_DomePro.setDomeShutterCloseFirst(nTmp+1);
            }

            // no command for Inhibit simultaneous shutter motion

            bTmp = dx->isChecked(SHUTTER_OPERATE_AT_HOME);
            m_DomePro.setDomeShutOpOnHome(bTmp);

            bTmp = dx->isChecked(HOME_ON_SHUTTER_CLOSE);
            m_DomePro.setHomeWithShutterClose(bTmp);

            bTmp = dx->isChecked(UPPER_SHUTTER_LIMIT_CHECK);
            m_DomePro.setShutter1_LimitFaultCheckEnabled(bTmp);

            bTmp = dx->isChecked(LOWER_SHUTTER_LIMIT_CHECK);
            m_DomePro.setShutter2_LimitFaultCheckEnabled(bTmp);


            bTmp = dx->isChecked(HOME_ON_SHUTTER_CLOSE);

            dx->propertyDouble(SHUTTER1_OCP, "value", dTmp);
            m_DomePro.setDomeShutter1_OCP_Limit(dTmp);

            dx->propertyDouble(SHUTTER2_OCP, "value", dTmp);
            m_DomePro.setDomeShutter2_OCP_Limit(dTmp);
        }
    }



    m_nCurrentDialog = MAIN;
    return nErr;

}

//
// Shutter settings ui events
//
int X2Dome::doShutterDialogEvents(X2GUIExchangeInterface* uiex, const char* pszEvent)
{
    int nErr = SB_OK;
    printf("[doShutterDialogEvents] pszEvent : %s\n", pszEvent);

    if (!strcmp(pszEvent, CLEAR_LIMIT_FAULT_CLICKED))
        m_DomePro.clearDomeLimitFault();
    return nErr;
}

//
// Dome timout and automatic closure UI
//
int X2Dome::doDomeProTimeouts(bool& bPressedOK)
{
    int nErr = SB_OK;
    char szTmpBuf[SERIAL_BUFFER_SIZE];
    int nTmp;
    bool bTmp;

    X2ModalUIUtil uiutil(this, GetTheSkyXFacadeForDrivers());
    X2GUIInterface*                    ui = uiutil.X2UI();
    X2GUIExchangeInterface*            dx = NULL;

    bPressedOK = false;
    if (NULL == ui)
        return ERR_POINTER;
    nErr = ui->loadUserInterface("dometimeouts.ui", deviceType(), m_nPrivateISIndex);
    if (nErr)
        return nErr;

    dx = uiutil.X2DX();
    if (NULL == dx)
        return ERR_POINTER;

    m_nCurrentDialog = TIMEOUTS;
    if(m_bLinked) {
        dx->setEnabled(AZ_TIMEOUT_EN, true);
        dx->setEnabled(AZ_TIMEOUT_VAL, true);
        // Az timout
        dx->setEnabled(AZ_TIMEOUT_EN, true);
        m_DomePro.getDomeAzimuthTimeOutEnabled(bTmp);
        if(bTmp) {
            dx->setChecked(AZ_TIMEOUT_EN,true);
            dx->setEnabled(AZ_TIMEOUT_VAL, true);
            m_DomePro.getDomeAzimuthTimeOut(nTmp);
            dx->setPropertyInt(AZ_TIMEOUT_VAL, "value", nTmp);
        }
        else{
            dx->setChecked(AZ_TIMEOUT_EN,false);
            dx->setEnabled(AZ_TIMEOUT_VAL, false);
        }

    } else {
        dx->setEnabled(AZ_TIMEOUT_EN, false);
        dx->setEnabled(AZ_TIMEOUT_VAL, false);
    }

    nErr = ui->exec(bPressedOK);
    if (nErr )
        return nErr;

    m_nCurrentDialog = MAIN;
    return nErr;

}
//
// Timeouts ui events
//
int X2Dome::doTimeoutsDialogEvents(X2GUIExchangeInterface* uiex, const char* pszEvent)
{
    int nErr = SB_OK;
    printf("[doTimeoutsDialogEvents] pszEvent : %s\n", pszEvent);

    return nErr;
}

int X2Dome::doDomeProDiag(bool& bPressedOK)
{
    int nErr = SB_OK;
    X2ModalUIUtil uiutil(this, GetTheSkyXFacadeForDrivers());
    X2GUIInterface*                    ui = uiutil.X2UI();
    X2GUIExchangeInterface*            dx = NULL;

    bPressedOK = false;
    if (NULL == ui)
        return ERR_POINTER;
    nErr = ui->loadUserInterface("domeprodiag.ui", deviceType(), m_nPrivateISIndex);
    if (nErr)
        return nErr;

    dx = uiutil.X2DX();
    if (NULL == dx)
        return ERR_POINTER;

    m_nCurrentDialog = DIAG;

    nErr = ui->exec(bPressedOK);
    if (nErr )
        return nErr;

    m_nCurrentDialog = MAIN;
    return nErr;
}

//
// diag ui events
//
int X2Dome::doDiagDialogEvents(X2GUIExchangeInterface* uiex, const char* pszEvent)
{
    int nErr = SB_OK;

    printf("[doDiagDialogEvents] pszEvent : %s\n", pszEvent);

    if (!strcmp(pszEvent, CLEAR_DIAG_COUNT_CLICKED) ) {
    }

    if (!strcmp(pszEvent, CLEAR_DIAG_DEG_CLICKED)) {
    }

    if (!strcmp(pszEvent, CLEAR_RFLINK_ERRORS_CLICKED)) {
    }

    return nErr;
}




//
//HardwareInfoInterface
//
#pragma mark - HardwareInfoInterface

void X2Dome::deviceInfoNameShort(BasicStringInterface& str) const					
{
	str = "DomePro";
}

void X2Dome::deviceInfoNameLong(BasicStringInterface& str) const					
{
    str = "Astrometric Instruments DomePro";
}

void X2Dome::deviceInfoDetailedDescription(BasicStringInterface& str) const		
{
    str = "Astrometric Instruments DomePro";
}

 void X2Dome::deviceInfoFirmwareVersion(BasicStringInterface& str)					
{
    X2MutexLocker ml(GetMutex());
    if(m_bLinked) {
        char cFirmware[SERIAL_BUFFER_SIZE];
        m_DomePro.getFirmwareVersion(cFirmware, SERIAL_BUFFER_SIZE);
        str = cFirmware;

    }
    else
        str = "N/A";
}

void X2Dome::deviceInfoModel(BasicStringInterface& str)
{
    X2MutexLocker ml(GetMutex());
    if(m_bLinked) {
        char cModel[SERIAL_BUFFER_SIZE];
        m_DomePro.getModel(cModel, SERIAL_BUFFER_SIZE);
        str = cModel;
    }
    else
        str = "N/A";
}

//
//DriverInfoInterface
//
#pragma mark - DriverInfoInterface

 void	X2Dome::driverInfoDetailedInfo(BasicStringInterface& str) const	
{
    str = "Astrometric Instruments DomePro X2 plugin by Rodolphe Pineau";
}

double	X2Dome::driverInfoVersion(void) const
{
	return DRIVER_VERSION;
}

//
//DomeDriverInterface
//
#pragma mark - DomeDriverInterface

int X2Dome::dapiGetAzEl(double* pdAz, double* pdEl)
{
    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;

    *pdAz = m_DomePro.getCurrentAz();
    *pdEl = m_DomePro.getCurrentEl();
    return SB_OK;
}

int X2Dome::dapiGotoAzEl(double dAz, double dEl)
{
    int nErr;

    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;

    nErr = m_DomePro.gotoAzimuth(dAz);
    if(nErr)
        return ERR_CMDFAILED;

    else
        return SB_OK;
}

int X2Dome::dapiAbort(void)
{

    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;

    m_DomePro.abortCurrentCommand();

    return SB_OK;
}

int X2Dome::dapiOpen(void)
{
    int nErr;
    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;

    if(!m_bHasShutterControl)
        return SB_OK;

    nErr = m_DomePro.openDomeShutters();
    if(nErr)
        return ERR_CMDFAILED;

	return SB_OK;
}

int X2Dome::dapiClose(void)
{
    int nErr;
    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;

    if(!m_bHasShutterControl)
        return SB_OK;

    nErr = m_DomePro.CloseDomeShutters();
    if(nErr)
        return ERR_CMDFAILED;

	return SB_OK;
}

int X2Dome::dapiPark(void)
{
    int nErr;
    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;

    if(m_bHasShutterControl)
    {
        nErr = m_DomePro.CloseDomeShutters();
        if(nErr)
            return ERR_CMDFAILED;
    }

    nErr = m_DomePro.gotoDomePark();
    if(nErr)
        return ERR_CMDFAILED;

	return SB_OK;
}

int X2Dome::dapiUnpark(void)
{
    int nErr;
    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;
    // may be we need a flag to decide if we want to open shutters on unpark.
    if(m_bHasShutterControl)
    {
        nErr = m_DomePro.openDomeShutters();
        if(nErr)
            return ERR_CMDFAILED;
    }

    nErr = m_DomePro.unparkDome();
    if(nErr)
        return ERR_CMDFAILED;

	return SB_OK;
}

int X2Dome::dapiFindHome(void)
{
    int nErr;
    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;

    nErr = m_DomePro.goHome();
    if(nErr)
        return ERR_CMDFAILED;

    return SB_OK;
}

int X2Dome::dapiIsGotoComplete(bool* pbComplete)
{
    int nErr;
    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;

    nErr = m_DomePro.isGoToComplete(*pbComplete);
    if(nErr)
        return ERR_CMDFAILED;
    return SB_OK;
}

int X2Dome::dapiIsOpenComplete(bool* pbComplete)
{
    int nErr;
    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;
    
    if(!m_bHasShutterControl)
    {
        *pbComplete = true;
        return SB_OK;
    }

    nErr = m_DomePro.isOpenComplete(*pbComplete);
    if(nErr)
        return ERR_CMDFAILED;

    return SB_OK;
}

int	X2Dome::dapiIsCloseComplete(bool* pbComplete)
{
    int nErr;
    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;

    if(!m_bHasShutterControl)
    {
        *pbComplete = true;
        return SB_OK;
    }

    nErr = m_DomePro.isCloseComplete(*pbComplete);
    if(nErr)
        return ERR_CMDFAILED;

    return SB_OK;
}

int X2Dome::dapiIsParkComplete(bool* pbComplete)
{
    int nErr;
    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;

    nErr = m_DomePro.isParkComplete(*pbComplete);
    if(nErr)
        return ERR_CMDFAILED;

    return SB_OK;
}

int X2Dome::dapiIsUnparkComplete(bool* pbComplete)
{
    int nErr;
    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;

    nErr = m_DomePro.isUnparkComplete(*pbComplete);
    if(nErr)
        return ERR_CMDFAILED;

    return SB_OK;
}

int X2Dome::dapiIsFindHomeComplete(bool* pbComplete)
{
    int nErr;
    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;

    nErr = m_DomePro.isFindHomeComplete(*pbComplete);
    if(nErr)
        return ERR_CMDFAILED;

    return SB_OK;
}

int X2Dome::dapiSync(double dAz, double dEl)
{
    int nErr;

    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;

    nErr = m_DomePro.syncDome(dAz, dEl);
    if(nErr)
        return ERR_CMDFAILED;
	return SB_OK;
}

//
// SerialPortParams2Interface
//
#pragma mark - SerialPortParams2Interface

void X2Dome::portName(BasicStringInterface& str) const
{
    char szPortName[DRIVER_MAX_STRING];

    portNameOnToCharPtr(szPortName, DRIVER_MAX_STRING);

    str = szPortName;

}

void X2Dome::setPortName(const char* pszPort)
{
    if (m_pIniUtil)
        m_pIniUtil->writeString(PARENT_KEY, CHILD_KEY_PORTNAME, pszPort);
    
}


void X2Dome::portNameOnToCharPtr(char* pszPort, const int& nMaxSize) const
{
    if (NULL == pszPort)
        return;

    snprintf(pszPort, nMaxSize,DEF_PORT_NAME);

    if (m_pIniUtil)
        m_pIniUtil->readString(PARENT_KEY, CHILD_KEY_PORTNAME, pszPort, pszPort, nMaxSize);
    
}



