//
// X2Dome implemntation
//

#include "x2dome.h"

X2Dome::X2Dome(const char* pszSelection, 
							 const int& nISIndex,
					SerXInterface*						pSerX,
					TheSkyXFacadeForDriversInterface*	pTheSkyXForMounts,
					SleeperInterface*					pSleeper,
					BasicIniUtilInterface*			pIniUtil,
					LoggerInterface*					pLogger,
					MutexInterface*						pIOMutex,
					TickCountInterface*					pTickCount)
{

    m_nPrivateISIndex				= nISIndex;
	m_pSerX							= pSerX;
	m_pTheSkyXForMounts				= pTheSkyXForMounts;
	m_pSleeper						= pSleeper;
	m_pIniUtil						= pIniUtil;
	m_pLogger						= pLogger;	
	m_pIOMutex						= pIOMutex;
	m_pTickCount					= pTickCount;

	m_bLinked = false;
    m_bCalibratingDome = false;
    m_bBattRequest = 0;
    m_bDomeProDiagUI_enable = false;

    m_DomePro.SetSerxPointer(pSerX);
    m_DomePro.setLogger(pLogger);

    if (m_pIniUtil)
    {   
        m_DomePro.setHomeAz( m_pIniUtil->readDouble(PARENT_KEY, CHILD_KEY_HOME_AZ, 180) );
        m_DomePro.setParkAz( m_pIniUtil->readDouble(PARENT_KEY, CHILD_KEY_PARK_AZ, 180) );
    }
}


X2Dome::~X2Dome()
{
	if (m_pSerX)
		delete m_pSerX;
	if (m_pTheSkyXForMounts)
		delete m_pTheSkyXForMounts;
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
    double dHomeAz;
    double dParkAz;
    int nTmp;
    double dTmp;
    bool bTmp;

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
        //
        // Dome Az
        //

        // Az motor
        dx->setEnabled(CALIBRATE, true);
        nErr = m_DomePro.getDomeAzCPR(nTmp);
        dx->setPropertyInt(TICK_PER_REV, "value", nTmp);
        dx->setEnabled(ROTATION_COAST, true);
        nErr = m_DomePro.getDomeAzCoast(dTmp);
        dx->setPropertyDouble(ROTATION_COAST, "value", dTmp);
        dx->setEnabled(ENCODDER_POLARITY, true);
        nErr = m_DomePro.getDomeAzEncoderPolarity(nTmp);
        dx->setPropertyDouble(ENCODDER_POLARITY, "value", dTmp);
        // Homing
        dx->setEnabled(HOMING_DIR, true); // no corresponding function, need to ping Chris
        dx->setEnabled(HOME_POS, true);
        nErr = m_DomePro.getDomeHomeAz(dTmp);
        dx->setPropertyDouble(HOME_POS, "value", dTmp);
        dx->setEnabled(PARK_POS, true);
        nErr = m_DomePro.getDomeParkAz(dTmp);
        dx->setPropertyDouble(PARK_POS, "value", dTmp);
        //
        // Dome Shutter / Roof
        //
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

                dx->setEnabled(INHIBIT_SIMULT, true); // no corresponding function, need to ping Chris
            }
        }
        else { // no shutter unit
            dx->setChecked(SINGLE_SHUTTER,false);
            dx->setEnabled(OPEN_FIRST, false);
            dx->setEnabled(CLOSE_FIRST, false);
            dx->setEnabled(INHIBIT_SIMULT, false);
        }
        //
        // Dome timoute and automatic closure
        //
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

        dx->setEnabled(TICK_PER_REV, true);

        dx->setEnabled(DIAG_BUTTON, true);
    }
    else { // not connected, disable all controls
        // Az motor
        dx->setEnabled(CALIBRATE, false);
        dx->setEnabled(TICK_PER_REV, false);
        dx->setEnabled(ROTATION_COAST, false);
        dx->setEnabled(ENCODDER_POLARITY, false);
        // Homing
        dx->setEnabled(HOMING_DIR, false);
        dx->setEnabled(HOME_POS, false);
        dx->setEnabled(PARK_POS, false);
        // Dome Shutter / Roof
        dx->setPropertyString(DOMEPRO_MODEL, "text", "");
        // sequencing
        dx->setEnabled(SINGLE_SHUTTER, false);
        dx->setEnabled(OPEN_FIRST, false);
        dx->setEnabled(CLOSE_FIRST, false);
        dx->setEnabled(INHIBIT_SIMULT, false);
        
        dx->setEnabled(AZ_TIMEOUT_EN, false);
        dx->setEnabled(AZ_TIMEOUT_VAL, false);

        dx->setEnabled(TICK_PER_REV, false);
        dx->setEnabled(TICK_PER_REV, false);
        dx->setEnabled(TICK_PER_REV, false);

        dx->setEnabled(DIAG_BUTTON, false);

    }

    m_bCalibratingDome = false;
    

    //Display the user interface
    if ((nErr = ui->exec(bPressedOK)))
        return nErr;

    //Retreive values from the user interface
    if (bPressedOK)
    {
        dx->propertyDouble(HOME_POS, "value", dHomeAz);
        dx->propertyDouble(PARK_POS, "value", dParkAz);

        if(m_bLinked)
        {
            m_DomePro.setHomeAz(dHomeAz);
            m_DomePro.setParkAz(dParkAz);
        }

        // save the values to persistent storage
        nErr |= m_pIniUtil->writeDouble(PARENT_KEY, CHILD_KEY_HOME_AZ, dHomeAz);
        nErr |= m_pIniUtil->writeDouble(PARENT_KEY, CHILD_KEY_PARK_AZ, dParkAz);
    }
    return nErr;

}

void X2Dome::uiEvent(X2GUIExchangeInterface* uiex, const char* pszEvent)
{
    bool bComplete = false;
    int nErr;
    char szTmpBuf[SERIAL_BUFFER_SIZE];
    char szErrorMessage[LOG_BUFFER_SIZE];
    bool bPressedOK = false;
    int nTmp;
    double dTmp;

    printf("pszEvent : %s\n", pszEvent);
    printf("m_bDomeProDiagUI_enable : %s\n", m_bDomeProDiagUI_enable ? "true" : "false");

    if (!strcmp(pszEvent, "on_pushButtonCancel_clicked"))
        m_DomePro.abortCurrentCommand();

    if (!strcmp(pszEvent, "on_timer"))
    {
        if(m_bLinked) {
            if(m_bCalibratingDome) {
                // are we still calibrating ?
                bComplete = false;
                nErr = m_DomePro.isCalibratingComplete(bComplete);
                if(nErr) {
                    uiex->setEnabled(CALIBRATE, true);
                    uiex->setEnabled(BUTTON_OK, true);
                    snprintf(szErrorMessage, LOG_BUFFER_SIZE, "Error calibrating dome : Error %d", nErr);
                    uiex->messageBox("DomePro Calibrate", szErrorMessage);
                    m_bCalibratingDome = false;
                    return;;
                }
                
                if(!bComplete) {
                    return;
                }
                
                // enable "ok" and "calibrate"
                uiex->setEnabled(CALIBRATE, true);
                uiex->setEnabled(BUTTON_OK, true);
                // read step per rev from dome
                nErr = m_DomePro.getDomeAzCPR(nTmp);
                uiex->setPropertyInt(TICK_PER_REV, "value", nTmp);
                m_bCalibratingDome = false;
                
            }
        }
    }

    //
    // main dialog
    //
    if (!strcmp(pszEvent, CALIBRATE_CLICKED) && !m_bDomeProDiagUI_enable)
    {
        if(m_bLinked) {
            // disable "ok" and "calibrate"
            uiex->setEnabled(CALIBRATE, false);
            uiex->setEnabled(BUTTON_OK, false);
            m_DomePro.calibrate();
            m_bCalibratingDome = true;
        }
    }

    if (!strcmp(pszEvent, DIAG_CKICKED))
    {
        doAddDomeProDiag(bPressedOK);
    }

    //
    // diag ui
    //
    if (!strcmp(pszEvent, DIAG_OK_CLICKED) && m_bDomeProDiagUI_enable) {
    }

    if (!strcmp(pszEvent, CLEAR_DIAG_COUNT_CLICKED) && m_bDomeProDiagUI_enable)
    {
    }

    if (!strcmp(pszEvent, CLEAR_DIAG_DEG_CLICKED) && m_bDomeProDiagUI_enable)
    {
    }

    if (!strcmp(pszEvent, CLEAR_RFLINK_ERRORS_CLICKED) && m_bDomeProDiagUI_enable)
    {
    }

}

int X2Dome::doAddDomeProDiag(bool& bPressedOK)
{
    int nErr = SB_OK;
    X2ModalUIUtil uiutil(this, GetTheSkyXFacadeForDrivers());
    X2GUIInterface*					ui = uiutil.X2UI();
    X2GUIExchangeInterface*			dx = NULL;

    bPressedOK = false;
    if (NULL == ui)
        return ERR_POINTER;
    nErr = ui->loadUserInterface("domeprodiag.ui", deviceType(), m_nPrivateISIndex);
    if (nErr)
        return nErr;

    dx = uiutil.X2DX();
    if (NULL == dx)
        return ERR_POINTER;

    m_bDomeProDiagUI_enable = true;

    nErr = ui->exec(bPressedOK);
    if (nErr )
        return nErr;

    m_bDomeProDiagUI_enable = false;
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



