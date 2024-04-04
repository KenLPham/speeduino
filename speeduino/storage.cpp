/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the projects root directory
*/
/** @file
 * Lower level ConfigPage*, Table2D, Table3D and EEPROM storage operations.
 */

#include "globals.h"
#include EEPROM_LIB_H //This is defined in the board .h files
#include "storage.h"
#include "pages.h"
#include "table3d_axis_io.h"

#ifdef USE_LITTLEFS
#include "src/LittleFS/src/LittleFS.h"
#endif

#define EEPROM_DATA_VERSION   0


// Calibration data is stored at the end of the EEPROM (This is in case any further calibration tables are needed as they are large blocks)
#define STORAGE_END 0xFFF       // Should be E2END?
#define EEPROM_CALIBRATION_CLT_VALUES (STORAGE_END-sizeof(cltCalibration_values))
#define EEPROM_CALIBRATION_CLT_BINS   (EEPROM_CALIBRATION_CLT_VALUES-sizeof(cltCalibration_bins))
#define EEPROM_CALIBRATION_IAT_VALUES (EEPROM_CALIBRATION_CLT_BINS-sizeof(iatCalibration_values))
#define EEPROM_CALIBRATION_IAT_BINS   (EEPROM_CALIBRATION_IAT_VALUES-sizeof(iatCalibration_bins))
#define EEPROM_CALIBRATION_O2_VALUES  (EEPROM_CALIBRATION_IAT_BINS-sizeof(o2Calibration_values))
#define EEPROM_CALIBRATION_O2_BINS    (EEPROM_CALIBRATION_O2_VALUES-sizeof(o2Calibration_bins))
#define EEPROM_LAST_BARO              (EEPROM_CALIBRATION_O2_BINS-1)


uint32_t deferEEPROMWritesUntil = 0;

bool isEepromWritePending(void)
{
  return BIT_CHECK(currentStatus.status4, BIT_STATUS4_BURNPENDING);
}

#ifdef USE_LITTLEFS
// WARNING: write is blocking
// TODO: use external NAND chip
#define CONFIG_FLASH_SIZE (1024 * 1024) // 1MB
LittleFS_Program fs;
#endif

bool initialiseStorage()
{
    #ifdef USE_LITTLEFS
    return fs.begin(CONFIG_FLASH_SIZE);
    #else
    return true;
    #endif
}

/** Write all config pages to EEPROM.
 */
void writeAllConfig(void)
{
  uint8_t pageCount = getPageCount();
  uint8_t page = 1U;
  writeConfig(page);
  page = page + 1;
  while (page<pageCount && !isEepromWritePending())
  {
    writeConfig(page);
    page = page + 1;
  }
}


//  ================================= Internal write support ===============================
struct write_location {
  #ifdef USE_LITTLEFS
  // TODO: global file object for both read and write?
  File *file;
  #endif
  eeprom_address_t address; // EEPROM address to write next
  uint16_t counter; // Number of bytes written
  uint8_t write_block_size; // Maximum number of bytes to write

  /** Update byte to EEPROM by first comparing content and the need to write it.
  We only ever write to the EEPROM where the new value is different from the currently stored byte
  This is due to the limited write life of the EEPROM (Approximately 100,000 writes)
  */
  void update(uint8_t value)
  {
    #ifdef USE_LITTLEFS
    file->seek(address);
    file->write(&value, sizeof(value));
    ++counter;
    #else
    if (EEPROM.read(address)!=value)
    {
      EEPROM.write(address, value);
      ++counter;
    }
    #endif 
  }

  /** Create a copy with a different write address.
   * Allows chaining of instances.
   */
  write_location changeWriteAddress(eeprom_address_t newAddress) const {
    return {
            #ifdef USE_LITTLEFS
        file,
            #endif
            newAddress, counter, write_block_size };
  }

  write_location& operator++()
  {
    ++address;
    return *this;
  }

  bool can_write() const
  {
    bool canWrite = false;
    if(currentStatus.RPM > 0) { canWrite = (counter <= write_block_size); }
    else { canWrite = (counter <= (write_block_size * 8)); } //Write to EEPROM more aggressively if the engine is not running

    return canWrite;
  }
};

static inline write_location write_range(const byte *pStart, const byte *pEnd, write_location location)
{
  while ( location.can_write() && pStart!=pEnd)
  {
    location.update(*pStart);
    ++pStart; 
    ++location;
  }
  return location;
}

static inline write_location write(const table_row_iterator &row, write_location location)
{
  return write_range(&*row, row.end(), location);
}

static inline write_location write(table_value_iterator it, write_location location)
{
  while (location.can_write() && !it.at_end())
  {
    location = write(*it, location);
    ++it;
  }
  return location;
}

static inline write_location write(table_axis_iterator it, write_location location)
{
  const table3d_axis_io_converter converter = get_table3d_axis_converter(it.get_domain());
  while (location.can_write() && !it.at_end())
  {
    location.update(converter.to_byte(*it));
    ++location;
    ++it;
  }
  return location;
}


static inline write_location writeTable(void *pTable, table_type_t key, write_location location)
{
  return write(y_rbegin(pTable, key), 
                write(x_begin(pTable, key), 
                  write(rows_begin(pTable, key), location)));
}

//Simply an alias for EEPROM.update()
void EEPROMWriteRaw(uint16_t address, uint8_t data) {
    // TODO: this is fucked
    #ifdef USE_LITTLEFS
    File file = fs.open("ecu.cfg", FILE_WRITE);
    file.seek(address);
    file.write(&data, sizeof(data));
    file.close();
    #else
    EEPROM.update(address, data);
    #endif
}
uint8_t EEPROMReadRaw(uint16_t address) {
    #ifdef USE_LITTLEFS
    uint8_t data;
    File file = fs.open("ecu.cfg", FILE_READ);
    file.seek(address);
    file.read(&data, sizeof(data));
    file.close();
    return data;
    #else
    return EEPROM.read(address);
    #endif
}

//  ================================= End write support ===============================

/** Write a table or map to EEPROM storage.
Takes the current configuration (config pages and maps)
and writes them to EEPROM as per the layout defined in storage.h.
*/
void writeConfig(uint8_t pageNum)
{
//The maximum number of write operations that will be performed in one go.
//If we try to write to the EEPROM too fast (Eg Each write takes ~3ms on the AVR) then 
//the rest of the system can hang)
#if defined(USE_SPI_EEPROM)
  //For use with common Winbond SPI EEPROMs Eg W25Q16JV
  uint8_t EEPROM_MAX_WRITE_BLOCK = 20; //This needs tuning
#elif defined(CORE_STM32) || defined(CORE_TEENSY)
  uint8_t EEPROM_MAX_WRITE_BLOCK = 64;
#else
  uint8_t EEPROM_MAX_WRITE_BLOCK = 18;
  if(BIT_CHECK(currentStatus.status4, BIT_STATUS4_COMMS_COMPAT)) { EEPROM_MAX_WRITE_BLOCK = 8; } //If comms compatibility mode is on, slow the burn rate down even further

  #ifdef CORE_AVR
    //In order to prevent missed pulses during EEPROM writes on AVR, scale the
    //maximum write block size based on the RPM.
    //This calculation is based on EEPROM writes taking approximately 4ms per byte
    //(Actual value is 3.8ms, so 4ms has some safety margin) 
    if(currentStatus.RPM > 65) //Min RPM of 65 prevents overflow of uint8_t
    { 
      EEPROM_MAX_WRITE_BLOCK = (uint8_t)(15000U / currentStatus.RPM);
      EEPROM_MAX_WRITE_BLOCK = max(EEPROM_MAX_WRITE_BLOCK, 1);
      EEPROM_MAX_WRITE_BLOCK = min(EEPROM_MAX_WRITE_BLOCK, 15); //Any higher than this will cause comms timeouts on AVR
    }
  #endif

#endif

  #ifdef USE_LITTLEFS
  File file = fs.open("ecu.cfg", FILE_WRITE);
  #endif

  write_location result = {
        #ifdef USE_LITTLEFS
        &file,
        #endif
        0, 0, EEPROM_MAX_WRITE_BLOCK };

  switch(pageNum)
  {
    case veMapPage:
      /*---------------------------------------------------
      | Fuel table (See storage.h for data layout) - Page 1
      | 16x16 table itself + the 16 values along each of the axis
      -----------------------------------------------------*/
      result = writeTable(&fuelTable, decltype(fuelTable)::type_key, result.changeWriteAddress(EEPROM_CONFIG1_MAP));
      break;

    case veSetPage:
      /*---------------------------------------------------
      | Config page 2 (See storage.h for data layout)
      | 64 byte long config table
      -----------------------------------------------------*/
      result = write_range((byte *)&configPage2, (byte *)&configPage2+sizeof(configPage2), result.changeWriteAddress(EEPROM_CONFIG2_START));
      break;

    case ignMapPage:
      /*---------------------------------------------------
      | Ignition table (See storage.h for data layout) - Page 1
      | 16x16 table itself + the 16 values along each of the axis
      -----------------------------------------------------*/
      result = writeTable(&ignitionTable, decltype(ignitionTable)::type_key, result.changeWriteAddress(EEPROM_CONFIG3_MAP));
      break;

    case ignSetPage:
      /*---------------------------------------------------
      | Config page 2 (See storage.h for data layout)
      | 64 byte long config table
      -----------------------------------------------------*/
      result = write_range((byte *)&configPage4, (byte *)&configPage4+sizeof(configPage4), result.changeWriteAddress(EEPROM_CONFIG4_START));
      break;

    case afrMapPage:
      /*---------------------------------------------------
      | AFR table (See storage.h for data layout) - Page 5
      | 16x16 table itself + the 16 values along each of the axis
      -----------------------------------------------------*/
      result = writeTable(&afrTable, decltype(afrTable)::type_key, result.changeWriteAddress(EEPROM_CONFIG5_MAP));
      break;

    case afrSetPage:
      /*---------------------------------------------------
      | Config page 3 (See storage.h for data layout)
      | 64 byte long config table
      -----------------------------------------------------*/
      result = write_range((byte *)&configPage6, (byte *)&configPage6+sizeof(configPage6), result.changeWriteAddress(EEPROM_CONFIG6_START));
      break;

    case boostvvtPage:
      /*---------------------------------------------------
      | Boost and vvt tables (See storage.h for data layout) - Page 8
      | 8x8 table itself + the 8 values along each of the axis
      -----------------------------------------------------*/
      result = writeTable(&boostTable, decltype(boostTable)::type_key, result.changeWriteAddress(EEPROM_CONFIG7_MAP1));
      result = writeTable(&vvtTable, decltype(vvtTable)::type_key, result.changeWriteAddress(EEPROM_CONFIG7_MAP2));
      result = writeTable(&stagingTable, decltype(stagingTable)::type_key, result.changeWriteAddress(EEPROM_CONFIG7_MAP3));
      break;

    case seqFuelPage:
      /*---------------------------------------------------
      | Fuel trim tables (See storage.h for data layout) - Page 9
      | 6x6 tables itself + the 6 values along each of the axis
      -----------------------------------------------------*/
      result = writeTable(&trim1Table, decltype(trim1Table)::type_key, result.changeWriteAddress(EEPROM_CONFIG8_MAP1));
      result = writeTable(&trim2Table, decltype(trim2Table)::type_key, result.changeWriteAddress(EEPROM_CONFIG8_MAP2));
      result = writeTable(&trim3Table, decltype(trim3Table)::type_key, result.changeWriteAddress(EEPROM_CONFIG8_MAP3));
      result = writeTable(&trim4Table, decltype(trim4Table)::type_key, result.changeWriteAddress(EEPROM_CONFIG8_MAP4));
      result = writeTable(&trim5Table, decltype(trim5Table)::type_key, result.changeWriteAddress(EEPROM_CONFIG8_MAP5));
      result = writeTable(&trim6Table, decltype(trim6Table)::type_key, result.changeWriteAddress(EEPROM_CONFIG8_MAP6));
      result = writeTable(&trim7Table, decltype(trim7Table)::type_key, result.changeWriteAddress(EEPROM_CONFIG8_MAP7));
      result = writeTable(&trim8Table, decltype(trim8Table)::type_key, result.changeWriteAddress(EEPROM_CONFIG8_MAP8));
      break;

    case canbusPage:
      /*---------------------------------------------------
      | Config page 10 (See storage.h for data layout)
      | 192 byte long config table
      -----------------------------------------------------*/
      result = write_range((byte *)&configPage9, (byte *)&configPage9+sizeof(configPage9), result.changeWriteAddress(EEPROM_CONFIG9_START));
      break;

    case warmupPage:
      /*---------------------------------------------------
      | Config page 11 (See storage.h for data layout)
      | 192 byte long config table
      -----------------------------------------------------*/
      result = write_range((byte *)&configPage10, (byte *)&configPage10+sizeof(configPage10), result.changeWriteAddress(EEPROM_CONFIG10_START));
      break;

    case fuelMap2Page:
      /*---------------------------------------------------
      | Fuel table 2 (See storage.h for data layout)
      | 16x16 table itself + the 16 values along each of the axis
      -----------------------------------------------------*/
      result = writeTable(&fuelTable2, decltype(fuelTable2)::type_key, result.changeWriteAddress(EEPROM_CONFIG11_MAP));
      break;

    case wmiMapPage:
      /*---------------------------------------------------
      | WMI and Dwell tables (See storage.h for data layout) - Page 12
      | 8x8 WMI table itself + the 8 values along each of the axis
      | 8x8 VVT2 table + the 8 values along each of the axis
      | 4x4 Dwell table itself + the 4 values along each of the axis
      -----------------------------------------------------*/
      result = writeTable(&wmiTable, decltype(wmiTable)::type_key, result.changeWriteAddress(EEPROM_CONFIG12_MAP));
      result = writeTable(&vvt2Table, decltype(vvt2Table)::type_key, result.changeWriteAddress(EEPROM_CONFIG12_MAP2));
      result = writeTable(&dwellTable, decltype(dwellTable)::type_key, result.changeWriteAddress(EEPROM_CONFIG12_MAP3));
      break;
      
    case progOutsPage:
      /*---------------------------------------------------
      | Config page 13 (See storage.h for data layout)
      -----------------------------------------------------*/
      result = write_range((byte *)&configPage13, (byte *)&configPage13+sizeof(configPage13), result.changeWriteAddress(EEPROM_CONFIG13_START));
      break;
    
    case ignMap2Page:
      /*---------------------------------------------------
      | Ignition table (See storage.h for data layout) - Page 1
      | 16x16 table itself + the 16 values along each of the axis
      -----------------------------------------------------*/
      result = writeTable(&ignitionTable2, decltype(ignitionTable2)::type_key, result.changeWriteAddress(EEPROM_CONFIG14_MAP));
      break;

    case boostvvtPage2:
      /*---------------------------------------------------
      | Boost duty cycle lookuptable (See storage.h for data layout) - Page 15
      | 8x8 table itself + the 8 values along each of the axis
      -----------------------------------------------------*/
      result = writeTable(&boostTableLookupDuty, decltype(boostTableLookupDuty)::type_key, result.changeWriteAddress(EEPROM_CONFIG15_MAP));

      /*---------------------------------------------------
      | Config page 15 (See storage.h for data layout)
      -----------------------------------------------------*/
      result = write_range((byte *)&configPage15, (byte *)&configPage15+sizeof(configPage15), result.changeWriteAddress(EEPROM_CONFIG15_START));
      break;
    
    case dbwPage:
      result = write_range((byte *)&configPage16, (byte *)&configPage16 + sizeof(configPage16), result.changeWriteAddress(EEPROM_CONFIG16_START));
      break;

    default:
      break;
  }

  #ifdef USE_LITTLEFS
  file.close();
  #endif
  BIT_WRITE(currentStatus.status4, BIT_STATUS4_BURNPENDING, !result.can_write());
}

/** Reset all configPage* structs (2,4,6,9,10,13) and write them full of null-bytes.
 */
void resetConfigPages(void)
{
  for (uint8_t page=1; page<getPageCount(); ++page)
  {
    page_iterator_t entity = page_begin(page);
    while (entity.type!=End)
    {
      if (entity.type==Raw)
      {
        memset(entity.pData, 0, entity.size);
      }
      entity = advance(entity);
    }
  }
}

//  ================================= Internal read support ===============================

/** Load range of bytes form EEPROM offset to memory.
 * @param address - start offset in EEPROM
 * @param pFirst - Start memory address
 * @param pLast - End memory address
 */
static inline eeprom_address_t load_range(eeprom_address_t address, byte *pFirst, const byte *pLast)
{
#if defined(CORE_AVR)
  // The generic code in the #else branch works but this provides a 45% speed up on AVR
  size_t size = pLast-pFirst;
  eeprom_read_block(pFirst, (const void*)(size_t)address, size);
  return address+size;
#elif defined(USE_LITTLEFS)
  size_t size = pLast-pFirst;
  File file = fs.open("ecu.cfg", FILE_READ);
  file.seek(address);
  file.read(pFirst, size);
  file.close();
  return address+size;
#else
  for (; pFirst != pLast; ++address, (void)++pFirst)
  {
    *pFirst = EEPROM.read(address);
  }
  return address;
#endif
}

static inline eeprom_address_t load(table_row_iterator row, eeprom_address_t address)
{
  return load_range(address, &*row, row.end());
}

static inline eeprom_address_t load(table_value_iterator it, eeprom_address_t address)
{
  while (!it.at_end())
  {
    address = load(*it, address);
    ++it;
  }
  return address; 
}

static inline eeprom_address_t load(table_axis_iterator it, eeprom_address_t address)
{
    const table3d_axis_io_converter converter = get_table3d_axis_converter(it.get_domain());
  #ifdef USE_LITTLEFS
  byte data;
  File file = fs.open("ecu.cfg", FILE_READ);
  #endif
  while (!it.at_end())
  {
    #ifdef USE_LITTLEFS
    file.seek(address);
    file.read(&data, sizeof(data));
    *it = converter.from_byte(data);
    #else
    *it = converter.from_byte(EEPROM.read(address));
    #endif
    ++address;
    ++it;
  }
  return address;    
}


static inline eeprom_address_t loadTable(void *pTable, table_type_t key, eeprom_address_t address)
{
  return load(y_rbegin(pTable, key),
                load(x_begin(pTable, key), 
                  load(rows_begin(pTable, key), address)));
}

//  ================================= End internal read support ===============================


/** Load all config tables from storage.
 */
void loadConfig(void)
{
  loadTable(&fuelTable, decltype(fuelTable)::type_key, EEPROM_CONFIG1_MAP);
  load_range(EEPROM_CONFIG2_START, (byte *)&configPage2, (byte *)&configPage2+sizeof(configPage2));
  
  //*********************************************************************************************************************************************************************************
  //IGNITION CONFIG PAGE (2)

  loadTable(&ignitionTable, decltype(ignitionTable)::type_key, EEPROM_CONFIG3_MAP);
  load_range(EEPROM_CONFIG4_START, (byte *)&configPage4, (byte *)&configPage4+sizeof(configPage4));

  //*********************************************************************************************************************************************************************************
  //AFR TARGET CONFIG PAGE (3)

  loadTable(&afrTable, decltype(afrTable)::type_key, EEPROM_CONFIG5_MAP);
  load_range(EEPROM_CONFIG6_START, (byte *)&configPage6, (byte *)&configPage6+sizeof(configPage6));

  //*********************************************************************************************************************************************************************************
  // Boost and vvt tables load
  loadTable(&boostTable, decltype(boostTable)::type_key, EEPROM_CONFIG7_MAP1);
  loadTable(&vvtTable, decltype(vvtTable)::type_key,  EEPROM_CONFIG7_MAP2);
  loadTable(&stagingTable, decltype(stagingTable)::type_key, EEPROM_CONFIG7_MAP3);

  //*********************************************************************************************************************************************************************************
  // Fuel trim tables load
  loadTable(&trim1Table, decltype(trim1Table)::type_key, EEPROM_CONFIG8_MAP1);
  loadTable(&trim2Table, decltype(trim2Table)::type_key, EEPROM_CONFIG8_MAP2);
  loadTable(&trim3Table, decltype(trim3Table)::type_key, EEPROM_CONFIG8_MAP3);
  loadTable(&trim4Table, decltype(trim4Table)::type_key, EEPROM_CONFIG8_MAP4);
  loadTable(&trim5Table, decltype(trim5Table)::type_key, EEPROM_CONFIG8_MAP5);
  loadTable(&trim6Table, decltype(trim6Table)::type_key, EEPROM_CONFIG8_MAP6);
  loadTable(&trim7Table, decltype(trim7Table)::type_key, EEPROM_CONFIG8_MAP7);
  loadTable(&trim8Table, decltype(trim8Table)::type_key, EEPROM_CONFIG8_MAP8);

  //*********************************************************************************************************************************************************************************
  //canbus control page load
  load_range(EEPROM_CONFIG9_START, (byte *)&configPage9, (byte *)&configPage9+sizeof(configPage9));

  //*********************************************************************************************************************************************************************************

  //CONFIG PAGE (10)
  load_range(EEPROM_CONFIG10_START, (byte *)&configPage10, (byte *)&configPage10+sizeof(configPage10));

  //*********************************************************************************************************************************************************************************
  //Fuel table 2 (See storage.h for data layout)
  loadTable(&fuelTable2, decltype(fuelTable2)::type_key, EEPROM_CONFIG11_MAP);

  //*********************************************************************************************************************************************************************************
  // WMI, VVT2 and Dwell table load
  loadTable(&wmiTable, decltype(wmiTable)::type_key, EEPROM_CONFIG12_MAP);
  loadTable(&vvt2Table, decltype(vvt2Table)::type_key, EEPROM_CONFIG12_MAP2);
  loadTable(&dwellTable, decltype(dwellTable)::type_key, EEPROM_CONFIG12_MAP3);

  //*********************************************************************************************************************************************************************************
  //CONFIG PAGE (13)
  load_range(EEPROM_CONFIG13_START, (byte *)&configPage13, (byte *)&configPage13+sizeof(configPage13));

  //*********************************************************************************************************************************************************************************
  //SECOND IGNITION CONFIG PAGE (14)

  loadTable(&ignitionTable2, decltype(ignitionTable2)::type_key, EEPROM_CONFIG14_MAP);

  //*********************************************************************************************************************************************************************************
  //CONFIG PAGE (15) + boost duty lookup table (LUT)
  loadTable(&boostTableLookupDuty, decltype(boostTableLookupDuty)::type_key, EEPROM_CONFIG15_MAP);
  load_range(EEPROM_CONFIG15_START, (byte *)&configPage15, (byte *)&configPage15+sizeof(configPage15));  

  //*********************************************************************************************************************************************************************************
  //CONFIG PAGE (16) DBW settings
  load_range(EEPROM_CONFIG16_START, (byte *)&configPage16, (byte *)&configPage16+sizeof(configPage16));
}

/** Read the calibration information from EEPROM.
This is separate from the config load as the calibrations do not exist as pages within the ini file for Tuner Studio.
*/
void loadCalibration(void)
{
  // If you modify this function be sure to also modify writeCalibration();
  // it should be a mirror image of this function.

  #ifdef USE_LITTLEFS
  File file = fs.open("ecu.cfg", FILE_READ);
  file.seek(EEPROM_CALIBRATION_O2_BINS);
  file.read(&o2Calibration_bins, sizeof(o2Calibration_bins));
  file.seek(EEPROM_CALIBRATION_O2_VALUES);
  file.read(&o2Calibration_values, sizeof(o2Calibration_values));

  file.seek(EEPROM_CALIBRATION_IAT_BINS);
  file.read(&iatCalibration_bins, sizeof(iatCalibration_bins));
  file.seek(EEPROM_CALIBRATION_IAT_VALUES);
  file.read(&iatCalibration_values, sizeof(iatCalibration_values));

  file.seek(EEPROM_CALIBRATION_CLT_BINS);
  file.read(&cltCalibration_bins, sizeof(cltCalibration_bins));
  file.seek(EEPROM_CALIBRATION_CLT_VALUES);
  file.read(&cltCalibration_values, sizeof(cltCalibration_values));
  file.close();
  #else
  EEPROM.get(EEPROM_CALIBRATION_O2_BINS, o2Calibration_bins);
  EEPROM.get(EEPROM_CALIBRATION_O2_VALUES, o2Calibration_values);
  
  EEPROM.get(EEPROM_CALIBRATION_IAT_BINS, iatCalibration_bins);
  EEPROM.get(EEPROM_CALIBRATION_IAT_VALUES, iatCalibration_values);

  EEPROM.get(EEPROM_CALIBRATION_CLT_BINS, cltCalibration_bins);
  EEPROM.get(EEPROM_CALIBRATION_CLT_VALUES, cltCalibration_values);
  #endif
}

/** Write calibration tables to EEPROM.
This takes the values in the 3 calibration tables (Coolant, Inlet temp and O2)
and saves them to the EEPROM.
*/
void writeCalibration(void)
{
  // If you modify this function be sure to also modify loadCalibration();
  // it should be a mirror image of this function.
  #ifdef USE_LITTLEFS
  File file = fs.open("ecu.cfg", FILE_WRITE);
  file.seek(EEPROM_CALIBRATION_O2_BINS);
  file.write(&o2Calibration_bins, sizeof(o2Calibration_bins));
  file.seek(EEPROM_CALIBRATION_O2_VALUES);
  file.write(o2Calibration_values, sizeof(o2Calibration_values));

  file.seek(EEPROM_CALIBRATION_IAT_BINS);
  file.write(&iatCalibration_bins, sizeof(iatCalibration_bins));
  file.seek(EEPROM_CALIBRATION_IAT_VALUES);
  file.write(iatCalibration_values, sizeof(iatCalibration_values));

  file.seek(EEPROM_CALIBRATION_CLT_BINS);
  file.write(&cltCalibration_bins, sizeof(cltCalibration_bins));
  file.seek(EEPROM_CALIBRATION_CLT_VALUES);
  file.write(&cltCalibration_values, sizeof(cltCalibration_values));
  file.close();
  #else
  EEPROM.put(EEPROM_CALIBRATION_O2_BINS, o2Calibration_bins);
  EEPROM.put(EEPROM_CALIBRATION_O2_VALUES, o2Calibration_values);
  
  EEPROM.put(EEPROM_CALIBRATION_IAT_BINS, iatCalibration_bins);
  EEPROM.put(EEPROM_CALIBRATION_IAT_VALUES, iatCalibration_values);

  EEPROM.put(EEPROM_CALIBRATION_CLT_BINS, cltCalibration_bins);
  EEPROM.put(EEPROM_CALIBRATION_CLT_VALUES, cltCalibration_values);
  #endif
}

void writeCalibrationPage(uint8_t pageNum)
{
  #ifdef USE_LITTLEFS
  File file = fs.open("ecu.cfg", FILE_WRITE);
  #endif
  if(pageNum == O2_CALIBRATION_PAGE)
  {
    #ifdef USE_LITTLEFS
    file.seek(EEPROM_CALIBRATION_O2_BINS);
    file.write(&o2Calibration_bins, sizeof(o2Calibration_bins));
    file.seek(EEPROM_CALIBRATION_O2_VALUES);
    file.write(&o2Calibration_values, sizeof(o2Calibration_values));
    #else
    EEPROM.put(EEPROM_CALIBRATION_O2_BINS, o2Calibration_bins);
    EEPROM.put(EEPROM_CALIBRATION_O2_VALUES, o2Calibration_values);
    #endif
  }
  else if(pageNum == IAT_CALIBRATION_PAGE)
  {
    #ifdef USE_LITTLEFS
    file.seek(EEPROM_CALIBRATION_IAT_BINS);
    file.write(&iatCalibration_bins, sizeof(iatCalibration_bins));
    file.seek(EEPROM_CALIBRATION_IAT_VALUES);
    file.write(&iatCalibration_values, sizeof(iatCalibration_values));
    #else
    EEPROM.put(EEPROM_CALIBRATION_IAT_BINS, iatCalibration_bins);
    EEPROM.put(EEPROM_CALIBRATION_IAT_VALUES, iatCalibration_values);
    #endif
  }
  else if(pageNum == CLT_CALIBRATION_PAGE)
  {
    #ifdef USE_LITTLEFS
    file.seek(EEPROM_CALIBRATION_CLT_BINS);
    file.write(&cltCalibration_bins, sizeof(cltCalibration_bins));
    file.seek(EEPROM_CALIBRATION_CLT_VALUES);
    file.write(&cltCalibration_values, sizeof(cltCalibration_values));
    #else
    EEPROM.put(EEPROM_CALIBRATION_CLT_BINS, cltCalibration_bins);
    EEPROM.put(EEPROM_CALIBRATION_CLT_VALUES, cltCalibration_values);
    #endif
  }
  #ifdef USE_LITTLEFS
  file.close();
  #endif
}

static eeprom_address_t compute_crc_address(uint8_t pageNum)
{
  return EEPROM_LAST_BARO-((getPageCount() - pageNum)*sizeof(uint32_t));
}

/** Write CRC32 checksum to EEPROM.
Takes a page number and CRC32 value then stores it in the relevant place in EEPROM
@param pageNum - Config page number
@param crcValue - CRC32 checksum
*/
void storePageCRC32(uint8_t pageNum, uint32_t crcValue)
{
  #ifdef USE_LITTLEFS
  File file = fs.open("ecu.cfg", FILE_WRITE);
  file.seek(compute_crc_address(pageNum));
  file.write(&crcValue, sizeof(crcValue));
  file.close();
  #else
  EEPROM.put(compute_crc_address(pageNum), crcValue);
  #endif
}

/** Retrieves and returns the 4 byte CRC32 checksum for a given page from EEPROM.
@param pageNum - Config page number
*/
uint32_t readPageCRC32(uint8_t pageNum)
{
  uint32_t crc32_val;
  #ifdef USE_LITTLEFS
  File file = fs.open("ecu.cfg", FILE_READ);
  file.seek(compute_crc_address(pageNum));
  file.read(&crc32_val, sizeof(crc32_val));
  file.close();
  return crc32_val;
  #else
  return EEPROM.get(compute_crc_address(pageNum), crc32_val);
  #endif
}

/** Same as above, but writes the CRC32 for the calibration page rather than tune data
@param calibrationPageNum - Calibration page number
@param calibrationCRC - CRC32 checksum
*/
void storeCalibrationCRC32(uint8_t calibrationPageNum, uint32_t calibrationCRC)
{
  uint16_t targetAddress;
  switch(calibrationPageNum)
  {
    case O2_CALIBRATION_PAGE:
      targetAddress = EEPROM_CALIBRATION_O2_CRC;
      break;
    case IAT_CALIBRATION_PAGE:
      targetAddress = EEPROM_CALIBRATION_IAT_CRC;
      break;
    case CLT_CALIBRATION_PAGE:
      targetAddress = EEPROM_CALIBRATION_CLT_CRC;
      break;
    default:
      targetAddress = EEPROM_CALIBRATION_CLT_CRC; //Obviously should never happen
      break;
  }
  #ifdef USE_LITTLEFS
  File file = fs.open("ecu.cfg", FILE_WRITE);
  file.seek(targetAddress);
  file.write(&calibrationCRC, sizeof(calibrationCRC));
  file.close();
  #else
  EEPROM.put(targetAddress, calibrationCRC);
  #endif
}

/** Retrieves and returns the 4 byte CRC32 checksum for a given calibration page from EEPROM.
@param calibrationPageNum - Config page number
*/
uint32_t readCalibrationCRC32(uint8_t calibrationPageNum)
{
  uint32_t crc32_val;
  uint16_t targetAddress;
  switch(calibrationPageNum)
  {
    case O2_CALIBRATION_PAGE:
      targetAddress = EEPROM_CALIBRATION_O2_CRC;
      break;
    case IAT_CALIBRATION_PAGE:
      targetAddress = EEPROM_CALIBRATION_IAT_CRC;
      break;
    case CLT_CALIBRATION_PAGE:
      targetAddress = EEPROM_CALIBRATION_CLT_CRC;
      break;
    default:
      targetAddress = EEPROM_CALIBRATION_CLT_CRC; //Obviously should never happen
      break;
  }

  #ifdef USE_LITTLEFS
  File file = fs.open("ecu.cfg", FILE_READ);
  file.seek(targetAddress);
  file.read(&crc32_val, sizeof(crc32_val));
  file.close();
  return crc32_val;
  #else
  EEPROM.get(targetAddress, crc32_val);
  return crc32_val;
  #endif
}

uint16_t getEEPROMSize(void)
{
  #ifdef USE_LITTLEFS
  return (uint16_t)CONFIG_FLASH_SIZE;
  #else
  return EEPROM.length();
  #endif
}

// Utility functions.
// By having these in this file, it prevents other files from calling EEPROM functions directly. This is useful due to differences in the EEPROM libraries on different devces
/// Read last stored barometer reading from EEPROM.
byte readLastBaro(void) { 
    #ifdef USE_LITTLEFS
    byte data;
    File file = fs.open("ecu.cfg", FILE_READ);
    file.seek(EEPROM_LAST_BARO);
    file.read(&data, sizeof(data));
    file.close();
    return data;
    #else
    return EEPROM.read(EEPROM_LAST_BARO);
    #endif
}
/// Write last acquired arometer reading to EEPROM.
void storeLastBaro(byte newValue) {
    #ifdef USE_LITTLEFS
    File file = fs.open("ecu.cfg", FILE_WRITE);
    file.seek(EEPROM_LAST_BARO);
    file.write(&newValue, sizeof(newValue));
    file.close();
    #else
    EEPROM.update(EEPROM_LAST_BARO, newValue);
    #endif
}
/// Read EEPROM current data format version (from offset EEPROM_DATA_VERSION).
byte readEEPROMVersion(void) {
    #ifdef USE_LITTLEFS
    byte data;
    File file = fs.open("ecu.cfg", FILE_READ);
    file.seek(EEPROM_DATA_VERSION);
    file.read(&data, sizeof(data));
    file.close();
    return data;
    #else
    return EEPROM.read(EEPROM_DATA_VERSION);
    #endif
}
/// Store EEPROM current data format version (to offset EEPROM_DATA_VERSION).
void storeEEPROMVersion(byte newVersion) {
    #ifdef USE_LITTLEFS
    File file = fs.open("ecu.cfg", FILE_WRITE);
    file.seek(EEPROM_DATA_VERSION);
    file.write(&newVersion, sizeof(newVersion));
    file.close();
    #else
    EEPROM.update(EEPROM_DATA_VERSION, newVersion);
    #endif
}
