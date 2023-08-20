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
    m_bShutterGotoEnabled = false;
    m_DomePro.SetSerxPointer(pSerX);
    m_DomePro.setLogger(pLogger);

    if (m_pIniUtil)
    {
        // read home Az
        m_DomePro.setHomeAz( m_pIniUtil->readDouble(PARENT_KEY, CHILD_KEY_HOME_AZ, 0));

        // shutter angle calibration
        m_Shutter1OpenAngle = m_pIniUtil->readInt(PARENT_KEY, CHILD_KEY_SHUTTER1_OPEN_ANGLE, 90);
        m_Shutter1OpenAngle_ADC = m_pIniUtil->readInt(PARENT_KEY, CHILD_KEY_SHUTTER1_OPEN_ANGLE_ADC, 3000);
        m_Shutter1CloseAngle = m_pIniUtil->readInt(PARENT_KEY, CHILD_KEY_SHUTTER1_CLOSE_ANGLE, 0);
        m_Shutter1CloseAngle_ADC = m_pIniUtil->readInt(PARENT_KEY, CHILD_KEY_SHUTTER1_CLOSE_ANGLE_ADC, 500);
        m_ADC_Ratio1 = (m_Shutter1OpenAngle_ADC - m_Shutter1CloseAngle_ADC) / (m_Shutter1OpenAngle - m_Shutter1CloseAngle);

        m_Shutter2OpenAngle = m_pIniUtil->readInt(PARENT_KEY, CHILD_KEY_SHUTTER2_OPEN_ANGLE, 90);
        m_Shutter2OpenAngle_ADC = m_pIniUtil->readInt(PARENT_KEY, CHILD_KEY_SHUTTER2_OPEN_ANGLE_ADC, 3000);
        m_Shutter2CloseAngle = m_pIniUtil->readInt(PARENT_KEY, CHILD_KEY_SHUTTER2_CLOSE_ANGLE, 0);
        m_Shutter2CloseAngle_ADC = m_pIniUtil->readInt(PARENT_KEY, CHILD_KEY_SHUTTER2_CLOSE_ANGLE_ADC, 500);
        m_ADC_Ratio2 = (m_Shutter2OpenAngle_ADC - m_Shutter2CloseAngle_ADC) / (m_Shutter2OpenAngle - m_Shutter2CloseAngle);

        m_bShutterGotoEnabled = m_pIniUtil->readInt(PARENT_KEY, CHILD_KEY_SHUTTER_GOTO, false);

        m_DomePro.setShutterAngleCalibration(m_Shutter1OpenAngle, m_Shutter1OpenAngle_ADC,
                                             m_Shutter1CloseAngle, m_Shutter1CloseAngle_ADC,
                                             m_Shutter2OpenAngle, m_Shutter2OpenAngle_ADC,
                                             m_Shutter2CloseAngle, m_Shutter2CloseAngle_ADC,
                                             m_bShutterGotoEnabled);
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
    int nTmp = 0;
    double dTmp = 0;
    bool bTmp = false;
    std::stringstream sTmpBuf;

    if (NULL == ui)
        return ERR_POINTER;

    if ((nErr = ui->loadUserInterface("domepro.ui", deviceType(), m_nPrivateISIndex)))
        return nErr;

    if (NULL == (dx = uiutil.X2DX()))
        return ERR_POINTER;


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

        m_DomePro.getDomePowerGoodInputState(bTmp);
        sTmpBuf << (bTmp ? "<html><head/><body><p><span style=\" color:#00FF00;\">Power Good input : OK</span></p></body></html>" : "<html><head/><body><p><span style=\" color:#FF0000;\">Power Good input : NOT OK</span></p></body></html>");
        dx->setPropertyString(POWER_GOOD,"text", sTmpBuf.str().c_str());

        // Homing
        dx->setEnabled(HOMING_DIR, true);
        nErr = m_DomePro.getDomeHomeDirection(nTmp);
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

        dx->setPropertyString(POWER_GOOD,"text", "Power Good input : NA");

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
            nErr = m_pIniUtil->writeDouble(PARENT_KEY, CHILD_KEY_HOME_AZ, dTmp);

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
    std::stringstream sTmpBuf;
    bool bTmp;

    int nTmp, nTmp2;
    bool bPressedOK = false;
    bool bIsAtHome;

    if (!strcmp(pszEvent, "on_pushButtonCancel_clicked") && m_nLearningDomeCPR != NONE) {
        m_DomePro.abortCurrentCommand();
        m_nLearningDomeCPR = NONE;
    }

    if (!strcmp(pszEvent, "on_timer"))
    {
        if(m_bLinked) {
            m_DomePro.getDomePowerGoodInputState(bTmp);
            sTmpBuf << (bTmp ? "<html><head/><body><p><span style=\" color:#00FF00;\">Power Good input: OK</span></p></body></html>" : "<html><head/><body><p><span style=\" color:#FF0000;\">Power Good input: NOT OK</span></p></body></html>");
            uiex->setPropertyString(POWER_GOOD,"text", sTmpBuf.str().c_str());

            switch(m_nLearningDomeCPR) {
                case RIGHT:
                case LEFT:
                    // are we still learning CPR ?
                    bComplete = false;
                    nErr = m_DomePro.isLearningCPRComplete(bComplete);
                    if(nErr) {
                        uiex->setEnabled(LEARN_AZIMUTH_CPR_RIGHT, true);
                        uiex->setEnabled(LEARN_AZIMUTH_CPR_LEFT, true);
                        uiex->setEnabled(BUTTON_OK, true);
                        sTmpBuf << "Error learning dome CPR : Error " << nErr;
                        uiex->messageBox("DomePro Learn CPR", sTmpBuf.str().c_str());
                        m_nLearningDomeCPR = NONE;
                        return nErr;
                    }

                    if(!bComplete) {
                        return nErr;
                    }

                    // enable "ok" and "Learn Azimuth CPR"
                    uiex->setEnabled(LEARN_AZIMUTH_CPR_RIGHT, true);
                    uiex->setEnabled(LEARN_AZIMUTH_CPR_LEFT, true);
                    uiex->setEnabled(BUTTON_OK, true);
                    // read gauged step per rev from dome
                    switch (m_nLearningDomeCPR) {
                        case LEFT:
                            nTmp = m_DomePro.getLeftCPR();
                            uiex->setPropertyString(L_CPR_VALUE, "text", std::to_string(nTmp).c_str());
                            break;

                        case RIGHT:
                            nTmp = m_DomePro.getRightCPR();
                            uiex->setPropertyString(R_CPR_VALUE, "text", std::to_string(nTmp).c_str());
                            break;
                        default:
                            break;
                    }
                    m_nLearningDomeCPR = NONE;
                    break;

                case CLEARING_LEFT :
                    m_DomePro.isDomeAtHome(bIsAtHome);
                    if(!bIsAtHome) {
                        m_DomePro.abortCurrentCommand();
                        m_DomePro.learnAzimuthCprLeft();
                        m_nLearningDomeCPR = LEFT;
                    }
                    break;

                case CLEARING_RIGHT :
                    m_DomePro.isDomeAtHome(bIsAtHome);
                    if(!bIsAtHome) {
                        m_DomePro.abortCurrentCommand();
                        m_DomePro.learnAzimuthCprRight();
                        m_nLearningDomeCPR = RIGHT;
                    }
                    break;

                default:
                    break;
            }
        }
    }

    if (!strcmp(pszEvent, LEARN_AZIMUTH_CPR_RIGHT_CLICKED) )
    {
        if(m_bLinked) {
            // disable "ok" and "Learn Azimuth CPR"
            uiex->setEnabled(LEARN_AZIMUTH_CPR_RIGHT, false);
            uiex->setEnabled(LEARN_AZIMUTH_CPR_LEFT, false);
            uiex->setEnabled(BUTTON_OK, false);
            m_DomePro.isDomeAtHome(bIsAtHome);
            if(bIsAtHome) {
                m_DomePro.learnAzimuthCprRight();
                m_nLearningDomeCPR = RIGHT;
            }
            else {
                m_DomePro.setDomeLeftOn();
                m_nLearningDomeCPR = CLEARING_RIGHT;
            }
        }
    }

    if (!strcmp(pszEvent, LEARN_AZIMUTH_CPR_LEFT_CLICKED) )
    {
        if(m_bLinked) {
            // disable "ok" and "Learn Azimuth CPR"
            uiex->setEnabled(LEARN_AZIMUTH_CPR_RIGHT, false);
            uiex->setEnabled(LEARN_AZIMUTH_CPR_LEFT, false);
            uiex->setEnabled(BUTTON_OK, false);
            m_DomePro.isDomeAtHome(bIsAtHome);
            if(bIsAtHome) {
                m_DomePro.learnAzimuthCprLeft();
                m_nLearningDomeCPR = LEFT;
            }
            else {
                m_DomePro.setDomeRightOn();
                m_nLearningDomeCPR = CLEARING_LEFT;
            }
        }
    }

    if (!strcmp(pszEvent, SET_CPR_FROM_GAUGED) )
    {
        if(m_bLinked) {
            nTmp = m_DomePro.getRightCPR();
            if(!nTmp) {
                uiex->messageBox("DomePro Set CPR", "Error setting dome CPR , right value can't be 0");
                return nErr;
            }

            nTmp2 = m_DomePro.getLeftCPR();
            if(!nTmp) {
                uiex->messageBox("DomePro Set CPR", "Error setting dome CPR , left value can't be 0");
                return nErr;
            }
            nTmp =  (int)floor( 0.5 +(nTmp + nTmp2)/2);
            nErr = m_DomePro.setDomeAzCPR(nTmp);
            if(nErr) {
                std::stringstream().swap(sTmpBuf);
                sTmpBuf << "Error setting dome CPR : Error " << nErr;
                uiex->messageBox("DomePro Set CPR", sTmpBuf.str().c_str());
                return nErr;
            }
            uiex->setPropertyInt(TICK_PER_REV, "value", nTmp);
        }
    }

    if (!strcmp(pszEvent, SHUTTER_CKICKED))
    {
        setMainDialogControlState(uiex, false);
        doDomeProShutter(bPressedOK);
        setMainDialogControlState(uiex, true);
    }

    if (!strcmp(pszEvent, TIMEOUTS_CKICKED))
    {
        setMainDialogControlState(uiex, false);
        doDomeProTimeouts(bPressedOK);
        setMainDialogControlState(uiex, true);
    }

    if (!strcmp(pszEvent, DIAG_CKICKED))
    {
        setMainDialogControlState(uiex, false);
        doDomeProDiag(bPressedOK);
        setMainDialogControlState(uiex, true);
    }

    return nErr;
}

void X2Dome::setMainDialogControlState(X2GUIExchangeInterface* uiex, bool enabled)
{
    uiex->setEnabled(LEARN_AZIMUTH_CPR_RIGHT, enabled);
    uiex->setEnabled(LEARN_AZIMUTH_CPR_LEFT, enabled);
    uiex->setEnabled(SET_AZIMUTH_CPR, enabled);
    uiex->setEnabled(SHUTTER_BUTTON, enabled);
    uiex->setEnabled(TIMEOUTS_BUTTON, enabled);
    uiex->setEnabled(DIAG_BUTTON, enabled);
    uiex->setEnabled(BUTTON_OK, enabled);
    uiex->setEnabled(BUTTON_CANCEL, enabled);

}

//
// Shutter settings UI
//
int X2Dome::doDomeProShutter(bool& bPressedOK)
{
    int nErr = SB_OK;
    std::string sTmp;
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
        m_DomePro.getModel(sTmp);
        dx->setPropertyString(DOMEPRO_MODEL, "text", sTmp.c_str());
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
                dx->setCurrentIndex(CLOSE_FIRST, nTmp-1);

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

                // shutter angle calibration
                if(m_DomePro.getModelType() == CLAMSHELL) {
                    dx->setPropertyInt(SHUT1_OPEN_ANGLE, "value", m_Shutter1OpenAngle);
                    dx->setPropertyInt(SHUT1_OPEN_ANGLE_ADC, "value", m_Shutter1OpenAngle_ADC);

                    dx->setPropertyInt(SHUT1_CLOSE_ANGLE, "value", m_Shutter1CloseAngle);
                    dx->setPropertyInt(SHUT1_CLOSE_ANGLE_ADC, "value", m_Shutter1CloseAngle_ADC);

                    dx->setPropertyInt(SHUT2_OPEN_ANGLE, "value", m_Shutter2OpenAngle);
                    dx->setPropertyInt(SHUT2_OPEN_ANGLE_ADC, "value", m_Shutter2OpenAngle_ADC);

                    dx->setPropertyInt(SHUT2_CLOSE_ANGLE, "value", m_Shutter2CloseAngle);
                    dx->setPropertyInt(SHUT2_CLOSE_ANGLE_ADC, "value", m_Shutter2CloseAngle_ADC);
                }
                else {
                    dx->setEnabled(SHUT1_OPEN_ANGLE, false);
                    dx->setEnabled(SHUT1_OPEN_ANGLE_ADC, false);
                    dx->setEnabled(SHUT1_CLOSE_ANGLE, false);
                    dx->setEnabled(SHUT1_CLOSE_ANGLE_ADC, false);
                    dx->setEnabled(SHUT2_OPEN_ANGLE, false);
                    dx->setEnabled(SHUT2_OPEN_ANGLE_ADC, false);
                    dx->setEnabled(SHUT2_CLOSE_ANGLE, false);
                    dx->setEnabled(SHUT2_CLOSE_ANGLE_ADC, false);
                }

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
            dx->setEnabled(SHUTTER_OPERATE_AT_HOME, false);
            dx->setEnabled(HOME_ON_SHUTTER_CLOSE, false);
            dx->setEnabled(UPPER_SHUTTER_LIMIT_CHECK, false);
            dx->setEnabled(LOWER_SHUTTER_LIMIT_CHECK, false);
            dx->setEnabled(SHUT1_OPEN_ANGLE, false);
            dx->setEnabled(SHUT1_OPEN_ANGLE_ADC, false);
            dx->setEnabled(SHUT1_CLOSE_ANGLE, false);
            dx->setEnabled(SHUT1_CLOSE_ANGLE_ADC, false);
            dx->setEnabled(SHUT2_OPEN_ANGLE, false);
            dx->setEnabled(SHUT2_OPEN_ANGLE_ADC, false);
            dx->setEnabled(SHUT2_CLOSE_ANGLE, false);
            dx->setEnabled(SHUT2_CLOSE_ANGLE_ADC, false);
            dx->setEnabled(SHUTTER1_OCP, false);
            dx->setEnabled(SHUTTER2_OCP, false);
        }

    } else {
        dx->setChecked(SINGLE_SHUTTER,false);
        dx->setEnabled(OPEN_FIRST, false);
        dx->setEnabled(CLOSE_FIRST, false);
        dx->setEnabled(INHIBIT_SIMULT, false);
        dx->setEnabled(SHUTTER_OPERATE_AT_HOME, false);
        dx->setEnabled(HOME_ON_SHUTTER_CLOSE, false);
        dx->setEnabled(UPPER_SHUTTER_LIMIT_CHECK, false);
        dx->setEnabled(LOWER_SHUTTER_LIMIT_CHECK, false);
        dx->setEnabled(SHUT1_OPEN_ANGLE, false);
        dx->setEnabled(SHUT1_OPEN_ANGLE_ADC, false);
        dx->setEnabled(SHUT1_CLOSE_ANGLE, false);
        dx->setEnabled(SHUT1_CLOSE_ANGLE_ADC, false);
        dx->setEnabled(SHUT2_OPEN_ANGLE, false);
        dx->setEnabled(SHUT2_OPEN_ANGLE_ADC, false);
        dx->setEnabled(SHUT2_CLOSE_ANGLE, false);
        dx->setEnabled(SHUT2_CLOSE_ANGLE_ADC, false);
        dx->setEnabled(SHUTTER1_OCP, false);
        dx->setEnabled(SHUTTER2_OCP, false);
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

            dx->propertyInt(SHUT1_OPEN_ANGLE, "value", m_Shutter1OpenAngle);
            dx->propertyInt(SHUT1_OPEN_ANGLE_ADC, "value", m_Shutter1OpenAngle_ADC);
            dx->propertyInt(SHUT1_CLOSE_ANGLE, "value", m_Shutter1CloseAngle);
            dx->propertyInt(SHUT1_CLOSE_ANGLE_ADC, "value", m_Shutter1CloseAngle_ADC);
            m_ADC_Ratio1 = (m_Shutter1OpenAngle_ADC - m_Shutter1CloseAngle_ADC) / (m_Shutter1OpenAngle - m_Shutter1CloseAngle);

            dx->propertyInt(SHUT2_OPEN_ANGLE, "value", m_Shutter2OpenAngle);
            dx->propertyInt(SHUT2_OPEN_ANGLE_ADC, "value", m_Shutter2OpenAngle_ADC);
            dx->propertyInt(SHUT2_CLOSE_ANGLE, "value", m_Shutter2CloseAngle);
            dx->propertyInt(SHUT2_CLOSE_ANGLE_ADC, "value", m_Shutter2CloseAngle_ADC);
            m_ADC_Ratio2 = (m_Shutter2OpenAngle_ADC - m_Shutter2CloseAngle_ADC) / (m_Shutter2OpenAngle - m_Shutter2CloseAngle);

            m_bShutterGotoEnabled = dx->isChecked(SHUT_ANGLE_GOTO);
            
            nErr = SB_OK;
            nErr |= m_pIniUtil->writeInt(PARENT_KEY, SHUT1_OPEN_ANGLE, m_Shutter1OpenAngle);
            nErr |= m_pIniUtil->writeInt(PARENT_KEY, SHUT1_OPEN_ANGLE_ADC, m_Shutter1OpenAngle_ADC);
            nErr |= m_pIniUtil->writeInt(PARENT_KEY, SHUT1_CLOSE_ANGLE, m_Shutter1CloseAngle);
            nErr |= m_pIniUtil->writeInt(PARENT_KEY, SHUT1_CLOSE_ANGLE_ADC, m_Shutter1CloseAngle_ADC);
            nErr |= m_pIniUtil->writeInt(PARENT_KEY, SHUT2_OPEN_ANGLE, m_Shutter2OpenAngle);
            nErr |= m_pIniUtil->writeInt(PARENT_KEY, SHUT2_OPEN_ANGLE_ADC, m_Shutter2OpenAngle_ADC);
            nErr |= m_pIniUtil->writeInt(PARENT_KEY, SHUT2_CLOSE_ANGLE, m_Shutter2CloseAngle);
            nErr |= m_pIniUtil->writeInt(PARENT_KEY, SHUT2_CLOSE_ANGLE_ADC, m_Shutter2CloseAngle_ADC);
            nErr |= m_pIniUtil->writeInt(PARENT_KEY, CHILD_KEY_SHUTTER_GOTO, m_bShutterGotoEnabled);


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

    if (!strcmp(pszEvent, CLEAR_LIMIT_FAULT_CLICKED))
        m_DomePro.clearDomeLimitFault();
    return nErr;
}

//
// Dome timeout and automatic closure UI
//
int X2Dome::doDomeProTimeouts(bool& bPressedOK)
{
    int nErr = SB_OK;
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
        // Az timout
        dx->setEnabled(AZ_TIMEOUT_EN, true);
        m_DomePro.getDomeAzimuthTimeOutEnabled(bTmp);
        dx->setChecked(AZ_TIMEOUT_EN, bTmp);

        dx->setEnabled(AZ_TIMEOUT_VAL, true);
        m_DomePro.getDomeAzimuthTimeOut(nTmp);
        dx->setPropertyInt(AZ_TIMEOUT_VAL, "value", nTmp);

        dx->setEnabled(FIST_SHUTTER_TIMEOUT_VAL, true);
        m_DomePro.getDomeShutter1_OpTimeOut(nTmp);
        dx->setPropertyInt(FIST_SHUTTER_TIMEOUT_VAL, "value", nTmp);

        dx->setEnabled(SECOND_SHUTTER_TIMEOUT_VAL, true);
        m_DomePro.getDomeShutter2_OpTimeOut(nTmp);
        dx->setPropertyInt(SECOND_SHUTTER_TIMEOUT_VAL, "value", nTmp);

        dx->setEnabled(OPPOSITE_DIR_TIMEOUT, true);
        m_DomePro.getDomeShutODirTimeOut(nTmp);
        dx->setPropertyInt(OPPOSITE_DIR_TIMEOUT, "value", nTmp);

        dx->setEnabled(CLOSE_NO_COMM, true);
        m_DomePro.getDomeShutCloseOnClientTimeOut(bTmp);
        dx->setChecked(CLOSE_NO_COMM, bTmp);

        dx->setEnabled(CLOSE_NO_COMM_VAL, true);
        m_DomePro.getDomeShutCloseClientTimeOut(nTmp);
        dx->setPropertyInt(CLOSE_NO_COMM_VAL, "value", nTmp);

        dx->setEnabled(CLOSE_ON_RADIO_TIMEOUT, true);
        m_DomePro.getDomeShutCloseOnLinkTimeOut(bTmp);
        dx->setChecked(CLOSE_ON_RADIO_TIMEOUT, bTmp);

        dx->setEnabled(CLOSE_ON_POWER_FAIL, true);
        m_DomePro.getShutterAutoCloseEnabled(bTmp);
        dx->setChecked(CLOSE_ON_POWER_FAIL, bTmp);

    } else {
        dx->setEnabled(AZ_TIMEOUT_EN, false);
        dx->setEnabled(AZ_TIMEOUT_VAL, false);
        dx->setEnabled(FIST_SHUTTER_TIMEOUT_VAL, false);
        dx->setEnabled(SECOND_SHUTTER_TIMEOUT_VAL, false);
        dx->setEnabled(OPPOSITE_DIR_TIMEOUT, false);
        dx->setEnabled(CLOSE_NO_COMM, false);
        dx->setEnabled(CLOSE_NO_COMM_VAL, false);
        dx->setEnabled(CLOSE_ON_RADIO_TIMEOUT, false);
        dx->setEnabled(CLOSE_ON_POWER_FAIL, false);
    }

    nErr = ui->exec(bPressedOK);
    if (nErr )
        return nErr;

    //Retreive values from the user interface
    if (bPressedOK)
    {
        if(m_bLinked)
        {
            bTmp = dx->isChecked(AZ_TIMEOUT_EN);
            m_DomePro.setDomeAzimuthTimeOutEnabled(bTmp);
            dx->propertyInt(AZ_TIMEOUT_VAL, "value", nTmp);
            m_DomePro.setDomeAzimuthTimeOut(nTmp);

            dx->propertyInt(FIST_SHUTTER_TIMEOUT_VAL, "value", nTmp);
            m_DomePro.setDomeShutter1_OpTimeOut(nTmp);

            dx->propertyInt(SECOND_SHUTTER_TIMEOUT_VAL, "value", nTmp);
            m_DomePro.setDomeShutter2_OpTimeOut(nTmp);

            dx->propertyInt(OPPOSITE_DIR_TIMEOUT, "value", nTmp);
            m_DomePro.setDomeShutODirTimeOut(nTmp);

            bTmp = dx->isChecked(CLOSE_NO_COMM);
            m_DomePro.setDomeShutCloseOnClientTimeOut(bTmp);

            dx->propertyInt(CLOSE_NO_COMM_VAL, "value", nTmp);
            m_DomePro.setDomeShutCloseClientTimeOut(nTmp);

            bTmp = dx->isChecked(CLOSE_ON_RADIO_TIMEOUT);
            m_DomePro.setDomeShutCloseOnLinkTimeOut(bTmp);

            bTmp = dx->isChecked(CLOSE_ON_POWER_FAIL);
            m_DomePro.setShutterAutoCloseEnabled(bTmp);

        }
    }

    m_nCurrentDialog = MAIN;
    return nErr;

}
//
// Timeouts ui events
//
int X2Dome::doTimeoutsDialogEvents(X2GUIExchangeInterface* uiex, const char* pszEvent)
{
    int nErr = SB_OK;
    return nErr;
}

int X2Dome::doDomeProDiag(bool& bPressedOK)
{
    int nErr = SB_OK;
    X2ModalUIUtil uiutil(this, GetTheSkyXFacadeForDrivers());
    X2GUIInterface*                    ui = uiutil.X2UI();
    X2GUIExchangeInterface*            dx = NULL;
    double dTmp;
    int nTmp;
    int nCPR;
    std::stringstream sTmpBuf;

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

    if(m_bLinked) {
        m_DomePro.getDomeSupplyVoltageAzimuthL(dTmp);
        sTmpBuf << std::fixed << std::setprecision(2) << dTmp << " V";
        dx->setText(AZ_SUPPLY_VOLTAGE, sTmpBuf.str().c_str());
        std::stringstream().swap(sTmpBuf);

        m_DomePro.getDomeAzimuthMotorADC(dTmp);
        sTmpBuf << std::fixed << std::setprecision(2) << dTmp << " A";
        dx->setText(AZ_MOTOR_CURRENT, sTmpBuf.str().c_str());
        std::stringstream().swap(sTmpBuf);

        m_DomePro.getDomeAzimuthTempADC(dTmp);
        sTmpBuf << std::fixed << std::setprecision(2) << dTmp << " ºC";
        dx->setText(AZ_TEMP, sTmpBuf.str().c_str());
        std::stringstream().swap(sTmpBuf);

        m_DomePro.getDomeAzDiagPosition(nTmp);
        dx->setText(AZ_DIAG_COUNT, std::to_string(dTmp).c_str());
        std::stringstream().swap(sTmpBuf);

        m_DomePro.getDomeAzCPR(nCPR);
        dTmp = (nTmp * 360.0 / nCPR);
        sTmpBuf << std::fixed << std::setprecision(2) << dTmp << " º";
        dx->setText(AZ_DIAG_DEG, sTmpBuf.str().c_str());
        std::stringstream().swap(sTmpBuf);

        m_DomePro.getDomeSupplyVoltageShutterL(dTmp);
        sTmpBuf << std::fixed << std::setprecision(2) << dTmp << " V";
        dx->setText(SHUT_SUPPLY_VOLTAGE, sTmpBuf.str().c_str());
        std::stringstream().swap(sTmpBuf);

        m_DomePro.getDomeShutterMotorADC(dTmp);
        sTmpBuf << std::fixed << std::setprecision(2) << dTmp << " A";
        dx->setText(SHUT_SUPPLY_CURRENT, sTmpBuf.str().c_str());
        std::stringstream().swap(sTmpBuf);

        m_DomePro.getDomeShutterTempADC(dTmp);
        sTmpBuf << std::fixed << std::setprecision(2) << dTmp << " ºC";
        dx->setText(SHUT_TEMPERATURE, sTmpBuf.str().c_str());
        std::stringstream().swap(sTmpBuf);

        m_DomePro.getDomeLinkErrCnt(nTmp);
        dx->setText(NB_REF_LINK_ERROR, std::to_string(dTmp).c_str());
    }
    else {

    }

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
    std::stringstream sTmpBuf;
    double dTmp;
    int nTmp;
    int nCPR;


    if (!strcmp(pszEvent, CLEAR_DIAG_COUNT_CLICKED) || !strcmp(pszEvent, CLEAR_DIAG_DEG_CLICKED)) {
        nErr = m_DomePro.clearDomeAzDiagPosition();
        nErr |= m_DomePro.getDomeAzDiagPosition(nTmp);
        uiex->setText(AZ_DIAG_COUNT, std::to_string(nTmp).c_str());

        nErr |= m_DomePro.getDomeAzCPR(nCPR);
        dTmp = (nTmp * 360.0 / nCPR);
        sTmpBuf << std::fixed << std::setprecision(2) << dTmp << " º";
        uiex->setText(AZ_DIAG_DEG, sTmpBuf.str().c_str());

    }

    if (!strcmp(pszEvent, CLEAR_RFLINK_ERRORS_CLICKED)) {
        nErr = m_DomePro.clearDomeLinkErrCnt();
        nErr |= m_DomePro.getDomeLinkErrCnt(nTmp);
        uiex->setText(NB_REF_LINK_ERROR, std::to_string(nTmp).c_str());
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
        std::string sFirmware;
        m_DomePro.getFirmwareVersion(sFirmware);
        str = sFirmware.c_str();

    }
    else
        str = "N/A";
}

void X2Dome::deviceInfoModel(BasicStringInterface& str)
{
    X2MutexLocker ml(GetMutex());
    if(m_bLinked) {
        std::string sModel;
        m_DomePro.getModel(sModel);
        str = sModel.c_str();
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
	return PLUGIN_VERSION;
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
    int nErr = SB_OK;

    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;

    nErr = m_DomePro.gotoAzimuth(dAz);
    if(nErr)
        return ERR_CMDFAILED;

    if(m_bShutterGotoEnabled)
        nErr = m_DomePro.gotoElevation(dEl);

    return nErr;
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
    bool bAzGotoDone, bElGotoDone = false;
    X2MutexLocker ml(GetMutex());

    if(!m_bLinked)
        return ERR_NOLINK;

    nErr = m_DomePro.isGoToComplete(bAzGotoDone);
    if(nErr)
        return ERR_CMDFAILED;

    *pbComplete = bAzGotoDone;

    if(m_bShutterGotoEnabled) {
        nErr = m_DomePro.isGoToElComplete(bElGotoDone);
        if(nErr)
            return ERR_CMDFAILED;
        *pbComplete = bAzGotoDone & bElGotoDone;
    }

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

    snprintf(pszPort, nMaxSize, DEF_PORT_NAME);

    if (m_pIniUtil)
        m_pIniUtil->readString(PARENT_KEY, CHILD_KEY_PORTNAME, pszPort, pszPort, nMaxSize);
    
}



