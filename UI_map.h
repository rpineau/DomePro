// map UI objectName to #define.
// *********************************************
// ----------
// DomePro.ui
// ----------
// Dome Az
// Az motor
#define MOTOR_POLARITY	"checkBox"
#define OVER_CURRENT_PROTECTION "overCurrentProtection"
// encoder
#define LEARN_AZIMUTH_CPR   "pushButton"
#define TICK_PER_REV	"ticksPerRev"
#define ROTATION_COAST	"doubleSpinBox"
#define ENCODDER_POLARITY   "checkBox_2"
// Homing
#define HOMING_DIR		"comboBox_3"
#define HOME_POS		"homePosition"
#define PARK_POS		"parkPosition"
// extra dialogs
#define SHUTTER_BUTTON  "pushButton_3"
#define TIMEOUTS_BUTTON "pushButton_4"
#define DIAG_BUTTON     "pushButton_2"

// Cancel/Ok
#define BUTTON_CANCEL    "pushButtonCancel"
#define BUTTON_OK       "pushButtonOK"
//
// Main settings dialog Events
//
#define LEARN_AZIMUTH_CPR_CLICKED   "on_pushButton_clicked"
#define DIAG_CKICKED        "on_pushButton_2_clicked"
#define SHUTTER_CKICKED     "on_pushButton_3_clicked"
#define TIMEOUTS_CKICKED    "on_pushButton_4_clicked"

// *********************************************
// -------------------
// Dome Shutter / Roof UI
// -------------------
#define DOMEPRO_MODEL	"label_6"
// sequencing
#define SINGLE_SHUTTER	"checkBox_3"
#define OPEN_FIRST		"comboBox"
#define CLOSE_FIRST		"comboBox_2"
#define INHIBIT_SIMULT	"checkBox_4"
#define SHUTTER_OPERATE_AT_HOME "checkBox_9"
#define HOME_ON_SHUTTER_CLOSE   "checkBox_8"

// *********************************************
// -------------------------------------
// Dome timoute and automatic closure UI
// -------------------------------------
// Az timout
#define AZ_TIMEOUT_EN	"checkBox_5"
#define AZ_TIMEOUT_VAL	"spinBox_2"

// Shutter timeout
#define FIST_SHUTTER_TIMEOUT		"label_9"
#define FIST_SHUTTER_TIMEOUT_VAL	"spinBox_3"
#define SECOND_SHUTTER_TIMEOUT		"label_10"
#define SECOND_SHUTTER_TIMEOUT_VAL	"spinBox_4"
#define OPPOSITE_DIR_TIMEOUT		"spinBox_5"
#define CLOSE_NO_COMM				"checkBox_6"
#define CLOSE_NO_COMM_VAL			"spinBox_6"
#define CLOSE_ON_RADIO_TIMEOUT		"checkBox_7"
#define CLOSE_ON_POWER_FAIL         "checkBox_10"



// *********************************************
// --------------
// DomeProDiag.ui
// --------------

// Azimuth
#define AZ_SUPPLY_VOLTAGE	"label_2"
#define AZ_MOTOR_CURRENT	"label_4"
#define AZ_TEMP				"label_8"
#define AZ_HOME_SWITCH		"label_10"
#define AZ_DIAG_COUNT		"label_12"
#define AZ_DIAG_COUNT_CLEAR	"pushButton"
#define AZ_DIAG_DEG			"label_14"
#define AX_DIAG_DEG_CLEAN	"pushButton_2"
// Shutter
#define SHUT_SUPPLY_VOLTAGE	"label_5"
#define SHUT_SUPPLY_CURRECT	"label_18"
#define SHUT_TEMPERATURE	"label_16"
#define NB_REF_LINK_ERROR	"label_20"
#define RF_LINK_ERROR_CLEAR	"pushButton_3"
// Cancel/Ok
#define DIAG_BUTTON_OK		"pushButtonOK"
//
// Events
//
#define DIAG_OK_CLICKED             "on_pushButtonOK_clicked"
#define CLEAR_DIAG_COUNT_CLICKED    "on_pushButton_clicked"
#define CLEAR_DIAG_DEG_CLICKED      "on_pushButton_2_clicked"
#define CLEAR_RFLINK_ERRORS_CLICKED "on_pushButton_3_clicked"

