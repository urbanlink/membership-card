/**************************************************************************
    
  @author   Elmü
  DIY electronic RFID Door Lock with Battery Backup (2016)

  Check for a new version of this code on 
  http://www.codeproject.com/Articles/1096861/DIY-electronic-RFID-Door-Lock-with-Battery-Backup

**************************************************************************/


#define USE_DESFIRE   false
#define USE_AES   false
#define COMPILE_SELFTEST 1

#define RESET_PIN        2
#define SPI_CLK_PIN      14 // The software SPI SCK  pin (Clock)
#define SPI_MISO_PIN     12 // The software SPI MISO pin (Master In, Slave Out)
#define SPI_MOSI_PIN     13 // The software SPI MOSI pin (Master Out, Slave In)
#define SPI_CS_PIN       5  // The software SPI SSEL pin (Chip Select)
// This is the interval that the RF field is switched off to save battery.
// The shorter this interval, the more power is consumed by the PN532.
// The longer  this interval, the longer the user has to wait until the door opens.
// The recommended interval is 1000 ms.
// Please note that the slowness of reading a Desfire card is not caused by this interval.
// The SPI bus speed is throttled to 10 kHz, which allows to transmit the data over a long cable, but this obviously makes reading the card slower.
#define RF_OFF_INTERVAL  1000

// ######################################################################################

#if USE_DESFIRE
    #if USE_AES
        #define DESFIRE_KEY_TYPE   AES
        #define DEFAULT_APP_KEY    gi_PN532.AES_DEFAULT_KEY
    #else
        #define DESFIRE_KEY_TYPE   DES
        #define DEFAULT_APP_KEY    gi_PN532.DES3_DEFAULT_KEY
    #endif
    
    #include "Desfire.h"
    #include "Secrets.h"
    #include "Buffer.h"
    Desfire          gi_PN532; // The class instance that communicates with Mifare Desfire cards   
    DESFIRE_KEY_TYPE gi_PiccMasterKey;
#else
    #include "Classic.h"
    Classic          gi_PN532; // The class instance that communicates with Mifare Classic cards
#endif

#include <EEPROM.h>

struct kCard {
    byte     u8_UidLength;   // UID = 4 or 7 bytes
    byte     u8_KeyVersion;  // for Desfire random ID cards
    bool      b_PN532_Error; // true -> the error comes from the PN532, false -> crypto error
    eCardType e_CardType;    
};

// global variables
char       gs8_CommandBuffer[500];  // Stores commands typed by the user via Terminal and the password
uint32_t   gu32_CommandPos = 0;     // Index in gs8_CommandBuffer
uint64_t   gu64_LastPasswd = 0;     // Timestamp when the user has enetered the password successfully
uint64_t   gu64_LastID     = 0;     // The last card UID that has been read by the RFID reader  
bool       gb_InitSuccess  = false; // true if the PN532 has been initialized successfully



void setup() {
    gs8_CommandBuffer[0] = 0;

//    gi_PN532.SetDebugLevel(2);
    
    gi_PN532.InitI2C(RESET_PIN);
    //gi_PN532.InitSoftwareSPI(SPI_CLK_PIN, SPI_MISO_PIN, SPI_MOSI_PIN, SPI_CS_PIN, RESET_PIN);
    SerialClass::Begin(115200);

    InitReader(false);

    #if USE_DESFIRE
        gi_PiccMasterKey.SetKeyData(SECRET_PICC_MASTER_KEY, sizeof(SECRET_PICC_MASTER_KEY), CARD_KEY_VERSION);
    #endif
}


void loop() {   
    uint64_t u64_StartTick = Utils::GetMillis64();

    static uint64_t u64_LastRead = 0;
    
    if (gb_InitSuccess) {
        
        // Turn on the RF field for 100 ms then turn it off for one second (RF_OFF_INTERVAL) to safe battery
        if ((int)(u64_StartTick - u64_LastRead) < RF_OFF_INTERVAL)
            return;
    }
    // pseudo loop (just used for aborting with break;)
    do {
      if (!gb_InitSuccess) {
        InitReader(true); // flash red LED for 2.4 seconds
        break;
      }

      // gi_PN532.DumpCardMemory("A","0x00,0x00,0x00,0x00,0x00,0x00", true); 
      
      // Create new card var
      kCard k_Card;
        
      if (!ReadCard("123456778", &k_Card)) {
        if (IsDesfireTimeout()) {
            // Nothing to do here because IsDesfireTimeout() prints additional error message and blinks the red LED
        } else if (k_Card.b_PN532_Error) {
            // Another error from PN532 -> reset the chip
            InitReader(true); // flash red LED for 2.4 seconds
        } else {
            // e.g. Error while authenticating with master key
            //FlashLED(LED_RED, 1000);
        }
        
        Utils::Print("> ");
        break;
      }

      // No card present in the RF field
      if (k_Card.u8_UidLength == 0) {
          gu64_LastID = 0;
          break;
      }

        // Still the same card present
//        if (gu64_LastID == k_User.ID.u64) 
//            break;
        
        // A different card was found in the RF field
        // OpenDoor() needs the RF field to be ON (for CheckDesfireSecret())
//      	OpenDoor(k_User.ID.u64, &k_Card, u64_StartTick);
      Utils::Print(">> ");
    }
    while (false);

    // Turn off the RF field to save battery
    // When the RF field is on,  the PN532 board consumes approx 110 mA.
    // When the RF field is off, the PN532 board consumes approx 18 mA.
    gi_PN532.SwitchOffRfField();

    u64_LastRead = Utils::GetMillis64();
}

// Reset the PN532 chip and initialize, set gb_InitSuccess = true on success
// If b_ShowError == true -> flash the red LED very slwoly
void InitReader(bool b_ShowError) {
  if (b_ShowError) {
    Utils::Print("Communication Error -> Reset PN532\r\n");
  }
  // pseudo loop (just used for aborting with break;)
  do {
    gb_InitSuccess = false;
      
        // Reset the PN532
        gi_PN532.begin(); // delay > 400 ms
    
        byte IC, VersionHi, VersionLo, Flags;
        if (!gi_PN532.GetFirmwareVersion(&IC, &VersionHi, &VersionLo, &Flags))
            break;
    
        char Buf[80];
        sprintf(Buf, "Chip: PN5%02X, Firmware version: %d.%d\r\n", IC, VersionHi, VersionLo);
        Utils::Print(Buf);
        sprintf(Buf, "Supports ISO 14443A:%s, ISO 14443B:%s, ISO 18092:%s\r\n", (Flags & 1) ? "Yes" : "No",
                                                                                (Flags & 2) ? "Yes" : "No",
                                                                                (Flags & 4) ? "Yes" : "No");
        Utils::Print(Buf);
         
        // Set the max number of retry attempts to read from a card.
        // This prevents us from waiting forever for a card, which is the default behaviour of the PN532.
        if (!gi_PN532.SetPassiveActivationRetries())
            break;
        
        // configure the PN532 to read RFID tags
        if (!gi_PN532.SamConfig())
            break;
    
        gb_InitSuccess = true;
    }
    while (false);

    if (b_ShowError)
    {
        delay(2000); // a long interval to make the LED flash very slowly        
        delay(100);
    }  
}

// Reads the card in the RF field.
// In case of a Random ID card reads the real UID of the card (requires PICC authentication)
// ATTENTION: If no card is present, this function returns true. This is not an error. (check that pk_Card->u8_UidLength > 0)
// pk_Card->u8_KeyVersion is > 0 if a random ID card did a valid authentication with SECRET_PICC_MASTER_KEY
// pk_Card->b_PN532_Error is set true if the error comes from the PN532.
bool ReadCard(byte u8_UID[8], kCard* pk_Card)
{
    memset(pk_Card, 0, sizeof(kCard));
  
    if (!gi_PN532.ReadPassiveTargetID(u8_UID, &pk_Card->u8_UidLength, &pk_Card->e_CardType))
    {
        pk_Card->b_PN532_Error = true;
        return false;
    }

    if (pk_Card->e_CardType == CARD_DesRandom) // The card is a Desfire card in random ID mode
    {
        #if USE_DESFIRE
            if (!AuthenticatePICC(&pk_Card->u8_KeyVersion))
                return false;
        
            // replace the random ID with the real UID
            if (!gi_PN532.GetRealCardID(u8_UID))
                return false;

            pk_Card->u8_UidLength = 7; // random ID is only 4 bytes
        #else
            Utils::Print("Cards with random ID are not supported in Classic mode.\r\n");
            return false;    
        #endif
    }
    return true;
}

// returns true if the cause of the last error was a Timeout.
// This may happen for Desfire cards when the card is too far away from the reader.
bool IsDesfireTimeout()
{
    #if USE_DESFIRE
        // For more details about this error see comment of GetLastPN532Error()
        if (gi_PN532.GetLastPN532Error() == 0x01) // Timeout
        {
            Utils::Print("A Timeout mostly means that the card is too far away from the reader.\r\n");
            
            // In this special case we make a short pause only because someone tries to open the door -> don't let him wait unnecessarily.
            FlashLED(LED_RED, 200);
            return true;
        }
    #endif
    return false;
}


// =================================== DESFIRE ONLY =========================================

#if USE_DESFIRE

// If the card is personalized -> authenticate with SECRET_PICC_MASTER_KEY,
// otherwise authenticate with the factory default DES key.
bool AuthenticatePICC(byte* pu8_KeyVersion)
{
    if (!gi_PN532.SelectApplication(0x000000)) // PICC level
        return false;

    if (!gi_PN532.GetKeyVersion(0, pu8_KeyVersion)) // Get version of PICC master key
        return false;

    // The factory default key has version 0, while a personalized card has key version CARD_KEY_VERSION
    if (*pu8_KeyVersion == CARD_KEY_VERSION)
    {
        if (!gi_PN532.Authenticate(0, &gi_PiccMasterKey))
            return false;
    }
    else // The card is still in factory default state
    {
        if (!gi_PN532.Authenticate(0, &gi_PN532.DES2_DEFAULT_KEY))
            return false;
    }
    return true;
}

// Generate two dynamic secrets: the Application master key (AES 16 byte or DES 24 byte) and the 16 byte StoreValue.
// Both are derived from the 7 byte card UID and the the user name + random data stored in EEPROM using two 24 byte 3K3DES keys.
// This function takes only 6 milliseconds to do the cryptographic calculations.
bool GenerateDesfireSecrets(kUser* pk_User, DESFireKey* pi_AppMasterKey, byte u8_StoreValue[16])
{
    // The buffer is initialized to zero here
    byte u8_Data[24] = {0}; 

    // Copy the 7 byte card UID into the buffer
    memcpy(u8_Data, pk_User->ID.u8, 7);

    // XOR the user name and the random data that are stored in EEPROM over the buffer.
    // s8_Name[NAME_BUF_SIZE] contains for example { 'P', 'e', 't', 'e', 'r', 0, 0xDE, 0x45, 0x70, 0x5A, 0xF9, 0x11, 0xAB }
    int B=0;
    for (int N=0; N<NAME_BUF_SIZE; N++)
    {
        u8_Data[B++] ^= pk_User->s8_Name[N];
        if (B > 15) B = 0; // Fill the first 16 bytes of u8_Data, the rest remains zero.
    }

    byte u8_AppMasterKey[24];

    DES i_3KDes;
    if (!i_3KDes.SetKeyData(SECRET_APPLICATION_KEY, sizeof(SECRET_APPLICATION_KEY), 0) || // set a 24 byte key (168 bit)
        !i_3KDes.CryptDataCBC(CBC_SEND, KEY_ENCIPHER, u8_AppMasterKey, u8_Data, 24))
        return false;
    
    if (!i_3KDes.SetKeyData(SECRET_STORE_VALUE_KEY, sizeof(SECRET_STORE_VALUE_KEY), 0) || // set a 24 byte key (168 bit)
        !i_3KDes.CryptDataCBC(CBC_SEND, KEY_ENCIPHER, u8_StoreValue, u8_Data, 16))
        return false;

    // If the key is an AES key only the first 16 bytes will be used
    if (!pi_AppMasterKey->SetKeyData(u8_AppMasterKey, sizeof(u8_AppMasterKey), CARD_KEY_VERSION))
        return false;

    return true;
}

// Check that the data stored on the card is the same as the secret generated by GenerateDesfireSecrets()
bool CheckDesfireSecret(kUser* pk_User)
{
    DESFIRE_KEY_TYPE i_AppMasterKey;
    byte u8_StoreValue[16];
    if (!GenerateDesfireSecrets(pk_User, &i_AppMasterKey, u8_StoreValue))
        return false;

    if (!gi_PN532.SelectApplication(0x000000)) // PICC level
        return false;

    byte u8_Version; 
    if (!gi_PN532.GetKeyVersion(0, &u8_Version))
        return false;

    // The factory default key has version 0, while a personalized card has key version CARD_KEY_VERSION
    if (u8_Version != CARD_KEY_VERSION)
        return false;

    if (!gi_PN532.SelectApplication(CARD_APPLICATION_ID))
        return false;

    if (!gi_PN532.Authenticate(0, &i_AppMasterKey))
        return false;

    // Read the 16 byte secret from the card
    byte u8_FileData[16];
    if (!gi_PN532.ReadFileData(CARD_FILE_ID, 0, 16, u8_FileData))
        return false;

    if (memcmp(u8_FileData, u8_StoreValue, 16) != 0)
        return false;

    return true;
}

// Store the SECRET_PICC_MASTER_KEY on the card
bool ChangePiccMasterKey()
{
    byte u8_KeyVersion;
    if (!AuthenticatePICC(&u8_KeyVersion))
        return false;

    if (u8_KeyVersion != CARD_KEY_VERSION) // empty card
    {
        // Store the secret PICC master key on the card.
        if (!gi_PN532.ChangeKey(0, &gi_PiccMasterKey, NULL))
            return false;

        // A key change always requires a new authentication
        if (!gi_PN532.Authenticate(0, &gi_PiccMasterKey))
            return false;
    }
    return true;
}

// Create the application SECRET_APPLICATION_ID,
// store the dynamic Application master key in the application,
// create a StandardDataFile SECRET_FILE_ID and store the dynamic 16 byte value into that file.
// This function requires previous authentication with PICC master key.
bool StoreDesfireSecret(kUser* pk_User)
{
    if (CARD_APPLICATION_ID == 0x000000 || CARD_KEY_VERSION == 0)
        return false; // severe errors in Secrets.h -> abort
  
    DESFIRE_KEY_TYPE i_AppMasterKey;
    byte u8_StoreValue[16];
    if (!GenerateDesfireSecrets(pk_User, &i_AppMasterKey, u8_StoreValue))
        return false;

    // First delete the application (The current application master key may have changed after changing the user name for that card)
    if (!gi_PN532.DeleteApplicationIfExists(CARD_APPLICATION_ID))
        return false;

    // Create the new application with default settings (we must still have permission to change the application master key later)
    if (!gi_PN532.CreateApplication(CARD_APPLICATION_ID, KS_FACTORY_DEFAULT, 1, i_AppMasterKey.GetKeyType()))
        return false;

    // After this command all the following commands will apply to the application (rather than the PICC)
    if (!gi_PN532.SelectApplication(CARD_APPLICATION_ID))
        return false;

    // Authentication with the application's master key is required
    if (!gi_PN532.Authenticate(0, &DEFAULT_APP_KEY))
        return false;

    // Change the master key of the application
    if (!gi_PN532.ChangeKey(0, &i_AppMasterKey, NULL))
        return false;

    // A key change always requires a new authentication with the new key
    if (!gi_PN532.Authenticate(0, &i_AppMasterKey))
        return false;

    // After this command the application's master key and it's settings will be frozen. They cannot be changed anymore.
    // To read or enumerate any content (files) in the application the application master key will be required.
    // Even if someone knows the PICC master key, he will neither be able to read the data in this application nor to change the app master key.
    if (!gi_PN532.ChangeKeySettings(KS_CHANGE_KEY_FROZEN))
        return false;

    // --------------------------------------------

    // Create Standard Data File with 16 bytes length
    DESFireFilePermissions k_Permis;
    k_Permis.e_ReadAccess         = AR_KEY0;
    k_Permis.e_WriteAccess        = AR_KEY0;
    k_Permis.e_ReadAndWriteAccess = AR_KEY0;
    k_Permis.e_ChangeAccess       = AR_KEY0;
    if (!gi_PN532.CreateStdDataFile(CARD_FILE_ID, &k_Permis, 16))
        return false;

    // Write the StoreValue into that file
    if (!gi_PN532.WriteFileData(CARD_FILE_ID, 0, 16, u8_StoreValue))
        return false;       
  
    return true;
}

// If you have already written the master key to a card and want to use the card for another purpose 
// you can restore the master key with this function. Additionally the application SECRET_APPLICATION_ID is deleted.
// If a user has been stored in the EEPROM for this card he will also be deleted.
bool RestoreDesfireCard()
{
    kUser k_User;
    kCard k_Card;  
    if (!WaitForCard(&k_User, &k_Card))
        return false;

    UserManager::DeleteUser(k_User.ID.u64, NULL);    

    if ((k_Card.e_CardType & CARD_Desfire) == 0)
    {
        Utils::Print("The card is not a Desfire card.\r\n");
        return false;
    }

    byte u8_KeyVersion;
    if (!AuthenticatePICC(&u8_KeyVersion))
        return false;

    // If the key version is zero AuthenticatePICC() has already successfully authenticated with the factory default DES key
    if (u8_KeyVersion == 0)
        return true;

    // An error in DeleteApplication must not abort. 
    // The key change below is more important and must always be executed.
    bool b_Success = gi_PN532.DeleteApplicationIfExists(CARD_APPLICATION_ID);
    if (!b_Success)
    {
        // After any error the card demands a new authentication
        if (!gi_PN532.Authenticate(0, &gi_PiccMasterKey))
            return false;
    }
    
    if (!gi_PN532.ChangeKey(0, &gi_PN532.DES2_DEFAULT_KEY, NULL))
        return false;

    // Check if the key change was successfull
    if (!gi_PN532.Authenticate(0, &gi_PN532.DES2_DEFAULT_KEY))
        return false;

    return b_Success;
}

bool MakeRandomCard()
{
    Utils::Print("\r\nATTENTION: Configuring the card to send a random ID cannot be reversed.\r\nThe card will be a random ID card FOREVER!\r\nIf you are really sure what you are doing hit 'Y' otherwise hit 'N'.\r\n\r\n");
    if (!WaitForKeyYesNo())
        return false;
    
    kUser k_User;
    kCard k_Card;  
    if (!WaitForCard(&k_User, &k_Card))
        return false;

    if ((k_Card.e_CardType & CARD_Desfire) == 0)
    {
        Utils::Print("The card is not a Desfire card.\r\n");
        return false;
    }

    byte u8_KeyVersion;
    if (!AuthenticatePICC(&u8_KeyVersion))
        return false;

    return gi_PN532.EnableRandomIDForever();
}

#endif // USE_DESFIRE


