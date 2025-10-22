// SPDX-License-Identifier: MPL-2.0
//
// Do NOT modify or remove this copyright and license
//
// Copyright (c) 2012-2025 Seagate Technology LLC and/or its Affiliates, All Rights Reserved
//
// This software is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.
//
// ******************************************************************************************
//
// \file buffer_test.c
// \brief This file defines the function calls for performing buffer/cabling tests

#include "bit_manip.h"
#include "code_attributes.h"
#include "common_types.h"
#include "error_translation.h"
#include "io_utils.h"
#include "math_utils.h"
#include "memory_safety.h"
#include "pattern_utils.h"
#include "precision_timer.h"
#include "string_utils.h"
#include "type_conversion.h"

#include "buffer_test.h"

static bool ata_Buffer_Commands_Supported(const tDevice* device)
{
    bool supported = false;
    if ((is_ATA_Identify_Word_Valid(le16_to_host(device->drive_info.IdentifyData.ata.Word082)) &&
         le16_to_host(device->drive_info.IdentifyData.ata.Word082) & BIT13 &&
         le16_to_host(device->drive_info.IdentifyData.ata.Word082) & BIT12) ||
        (is_ATA_Identify_Word_Valid(le16_to_host(device->drive_info.IdentifyData.ata.Word085)) &&
         le16_to_host(device->drive_info.IdentifyData.ata.Word085) & BIT13 &&
         le16_to_host(device->drive_info.IdentifyData.ata.Word085) & BIT12))
    {
        // PIO commands
        supported = true;
    }
    if ((is_ATA_Identify_Word_Valid(le16_to_host(device->drive_info.IdentifyData.ata.Word053)) &&
         le16_to_host(device->drive_info.IdentifyData.ata.Word053) & BIT1) /* this is a validity bit for field 69 */
        && (is_ATA_Identify_Word_Valid(le16_to_host(device->drive_info.IdentifyData.ata.Word069)) &&
            le16_to_host(device->drive_info.IdentifyData.ata.Word069) & BIT11 &&
            le16_to_host(device->drive_info.IdentifyData.ata.Word069) & BIT10))
    {
        // DMA commands
        supported = true;
    }
    return supported;
}

static bool scsi_Buffer_Commands_Supported(const tDevice* device)
{
    bool supported = false;
    // SCSI 2 + should support this.
    // SCSI 1 probably won't...but this is so old it may not be a problem
    // Only asking about read buffer command, since write buffer will likely be implemented for at least FWDL, so if
    // this is supported, the equivalent write buffer command should also be supported
    scsiOperationCodeInfoRequest readBufSupReq;
    safe_memset(&readBufSupReq, sizeof(scsiOperationCodeInfoRequest), 0, sizeof(scsiOperationCodeInfoRequest));
    readBufSupReq.operationCode      = READ_BUFFER_CMD;
    readBufSupReq.serviceActionValid = false;
    eSCSICmdSupport readBufSupport   = is_SCSI_Operation_Code_Supported(device, &readBufSupReq);
    if (readBufSupport == SCSI_CMD_SUPPORT_SUPPORTED_TO_SCSI_STANDARD)
    {
        supported = true;
    }
    else
    {
        // this means the command to ask about support didn't work, so we're just going to try asking the size of
        // the buffer and if that works, it is supported
        DECLARE_ZERO_INIT_ARRAY(uint8_t, supportedCommandData, 4);
        if (SUCCESS == scsi_Read_Buffer(device, SCSI_RB_DESCRIPTOR, 0, 0, 4, supportedCommandData))
        {
            supported = true;
        }
    }
    return supported;
}

static bool are_Buffer_Commands_Available(const tDevice* device)
{
    bool supported = false;
    // Check if read/write buffer commands are supported on SATA and SAS
    if (device->drive_info.drive_type == ATA_DRIVE)
    {
        supported = ata_Buffer_Commands_Supported(device);
    }
    else if (device->drive_info.drive_type == SCSI_DRIVE)
    {
        supported = scsi_Buffer_Commands_Supported(device);
    }
    return supported;
}

static eReturnValues get_Buffer_Size(const tDevice* device, uint32_t* bufferSize, uint8_t* offsetBoundary)
{
    eReturnValues ret = SUCCESS;
    if (!bufferSize || !offsetBoundary)
    {
        return BAD_PARAMETER;
    }
    *bufferSize = LEGACY_DRIVE_SEC_SIZE; // default to this size. Change this only if the drive reports a different size
    *offsetBoundary = 0x09;              // default to this size. Change this only if the drive reports a different size
    // get the size of the buffer for the drive.
    if (device->drive_info.drive_type == SCSI_DRIVE)
    {
        DECLARE_ZERO_INIT_ARRAY(uint8_t, bufferSizeData, 4);
        if (SUCCESS == scsi_Read_Buffer(device, SCSI_RB_DESCRIPTOR, 0, 0, 4, bufferSizeData))
        {
            *offsetBoundary = bufferSizeData[0]; // not sure if this is actually needed - TJE
            *bufferSize     = M_BytesTo4ByteValue(0, bufferSizeData[1], bufferSizeData[2], bufferSizeData[3]);
        }
        else
        {
            ret = FAILURE; // this shouldn't happen...
        }
    }
    return ret;
}

static eReturnValues send_Read_Buffer_Command(const tDevice* device, uint8_t* ptrData, uint32_t dataSize)
{
    if (device->drive_info.drive_type == ATA_DRIVE)
    {
        // return ata_Read_Buffer(device, ptrData, device->drive_info.ata_Options.readBufferDMASupported);
        // Switching to this new function since it will automatically try DMA mode if supported by the drive.
        // If the controller or driver don't like issuing DMA mode, this will detect it and retry the command with PIO
        // mode.
        return send_ATA_Read_Buffer_Cmd(device, ptrData);
    }
    else if (device->drive_info.drive_type == SCSI_DRIVE)
    {
        return scsi_Read_Buffer(device, SCSI_RB_DATA, 0, 0, dataSize, ptrData);
    }
    else
    {
        return NOT_SUPPORTED;
    }
}

static eReturnValues send_Write_Buffer_Command(const tDevice* device, uint8_t* ptrData, uint32_t dataSize)
{
    if (device->drive_info.drive_type == ATA_DRIVE)
    {
        // return ata_Write_Buffer(device, ptrData, device->drive_info.ata_Options.writeBufferDMASupported);
        // Switching to this new function since it will automatically try DMA mode if supported by the drive.
        // If the controller or driver don't like issuing DMA mode, this will detect it and retry the command with PIO
        // mode.
        return send_ATA_Write_Buffer_Cmd(device, ptrData);
    }
    else if (device->drive_info.drive_type == SCSI_DRIVE)
    {
        return scsi_Write_Buffer(device, SCSI_WB_DATA, 0, 0, 0, dataSize, ptrData, false, false, 0);
    }
    else
    {
        return NOT_SUPPORTED;
    }
}

static bool was_There_A_CRC_Error_On_Last_Command(const tDevice* device)
{
    bool    crc            = false;
    bool    checkSenseData = false;
    uint8_t senseKey       = UINT8_C(0);
    uint8_t asc            = UINT8_C(0);
    uint8_t ascq           = UINT8_C(0);
    uint8_t fru            = UINT8_C(0);
    if (device->drive_info.drive_type == ATA_DRIVE)
    {
        if (device->drive_info.lastCommandRTFRs.status & ATA_STATUS_BIT_ERROR) // error bit set
        {
            if (device->drive_info.lastCommandRTFRs.error & ATA_ERROR_BIT_INTERFACE_CRC)
            {
                crc = true;
            }
        }
        if (device->drive_info.ataSenseData.validData && !crc)
        {
            checkSenseData = true;
            senseKey       = device->drive_info.ataSenseData.senseKey;
            asc            = device->drive_info.ataSenseData.additionalSenseCode;
            ascq           = device->drive_info.ataSenseData.additionalSenseCodeQualifier;
        }
    }
    else if (device->drive_info.drive_type == SCSI_DRIVE)
    {
        checkSenseData = true;
        get_Sense_Key_ASC_ASCQ_FRU(device->drive_info.lastCommandSenseData, SPC3_SENSE_LEN, &senseKey, &asc, &ascq,
                                   &fru);
    }
    if (checkSenseData)
    {
        if (senseKey == SENSE_KEY_ABORTED_COMMAND) // INFORMATION UNIT iuCRC ERROR DETECTED
        {
            switch (asc)
            {
            case 0x08:
                if (ascq == 0x03) // LOGICAL UNIT COMMUNICATION CRC ERROR (ULTRA-DMA/32)
                {
                    crc = true;
                }
                break;
            case 0x10:
                if (ascq == 0x10) // ID CRC OR ECC ERROR
                {
                    crc = true;
                }
                break;
            case 0x11:
                if (ascq == 0x0D) // DE-COMPRESSION CRC ERROR
                {
                    crc = true;
                }
                break;
            case 0x47:
                switch (ascq)
                {
                case 0x01: // DATA PHASE CRC ERROR DETECTED
                case 0x03: // INFORMATION UNIT iuCRC ERROR DETECTED - SAT will translate a CRC error into this!
                           // Definitely need this one. Less sure about the others...-TJE
                case 0x05: // PROTOCOL SERVICE CRC ERROR
                    crc = true;
                    break;
                default:
                    break;
                }
                break;
            case 0x4B:
                if (ascq == 0x12) // PCIE ECRC CHECK FAILED
                {
                    crc = true;
                }
                break;
            default:
                break;
            }
        }
    }
    return crc;
}

M_NODISCARD_REASON("This function tells whether the test should continue or not. You must use this result to determine when to continue testing or exit.")
static M_INLINE bool set_cmd_results(eReturnValues result, ptrPatternTestResults testResults, const tDevice *device)
{
    bool continueTest = true;
    switch (result)
    {
    case OS_PASSTHROUGH_FAILURE:
    case NOT_SUPPORTED:
        continueTest = false;
        break;
    case OS_COMMAND_TIMEOUT:
        ++(testResults->totalCommandTimeouts);
        break;
    case SUCCESS:
        break;
    case ABORTED:
    case COMMAND_FAILURE:
    case FAILURE:
    default:
        if (was_There_A_CRC_Error_On_Last_Command(device))
        {
            ++(testResults->totalCommandCRCErrors);
        }
        break;
    }
    return continueTest;
}

M_NODISCARD static M_INLINE bool write_read_compare_pattern(const tDevice* device,
                                                        uint8_t*       patternBuffer,
                                                        uint32_t       deviceBufferSize,
                                                        ptrPatternTestResults testResults)
{
    bool success = true;
    eReturnValues wbResult = send_Write_Buffer_Command(device, patternBuffer, deviceBufferSize);
    ++(testResults->totalCommandsSent);
    if (!set_cmd_results(wbResult, testResults, device))
    {
        return false;
    }
    // now read back the pattern
    uint8_t*       returnBuffer = safe_calloc_aligned(deviceBufferSize, sizeof(uint8_t), device->os_info.minimumAlignment);
    if (returnBuffer == M_NULLPTR)
    {
        return false;
    }
    eReturnValues rbResult = send_Read_Buffer_Command(device, returnBuffer, deviceBufferSize);
    ++(testResults->totalCommandsSent);
    if (!set_cmd_results(rbResult, testResults, device))
    {
        success = false;
    }
    else
    {
        ++(testResults->totalBufferComparisons);
        // first check if the pattern matches or not
        if (memcmp(patternBuffer, returnBuffer, deviceBufferSize) != 0)
        {
            ++(testResults->totalBufferMiscompares);
        }
    }
    safe_free_aligned(&returnBuffer);
    return success;
}

// Function for simple byte pattern tests. take counter for number of times to try it?
static void perform_Byte_Pattern_Test(const tDevice*        device,
                                      uint32_t              pattern,
                                      uint32_t              deviceBufferSize,
                                      ptrPatternTestResults testResults)
{
    uint32_t numberOfTimesToTest = UINT32_C(5);
    uint8_t* patternBuffer =
        C_CAST(uint8_t*,
               safe_malloc_aligned(deviceBufferSize, device->os_info.minimumAlignment)); // only send this to the drive
    if (patternBuffer)
    {
        fill_Pattern_Buffer_Into_Another_Buffer(C_CAST(uint8_t*, &pattern), sizeof(uint32_t), patternBuffer,
                                                deviceBufferSize); // sets the pattern to write into memory
        DECLARE_SEATIMER(patternTimer);
        start_Timer(&patternTimer);
        for (uint32_t counter = UINT32_C(0); counter < numberOfTimesToTest; ++counter)
        {
            if (!write_read_compare_pattern(device, patternBuffer, deviceBufferSize, testResults))
            {
                break;
            }
        }
        stop_Timer(&patternTimer);
        testResults->totalTimeNS = get_Nano_Seconds(patternTimer);
    }
    safe_free_aligned(&patternBuffer);
}

typedef enum eRowBoatPatternEnum
{
    ROW_BOAT_PATTERN_00,
    ROW_BOAT_PATTERN_55,
    ROW_BOAT_PATTERN_AA,
    ROW_BOAT_PATTERN_FF
}eRowBoatPattern;

typedef struct s_rowBoatPatternSequence
{
    eRowBoatPattern pat1;
    eRowBoatPattern pat2;
    eRowBoatPattern pat3;
    eRowBoatPattern pat4;
}rowBoatPatternSequence;

rowBoatPatternSequence seq1 = {ROW_BOAT_PATTERN_55, ROW_BOAT_PATTERN_FF, ROW_BOAT_PATTERN_AA, ROW_BOAT_PATTERN_FF};
rowBoatPatternSequence seq2 = {ROW_BOAT_PATTERN_55, ROW_BOAT_PATTERN_00, ROW_BOAT_PATTERN_AA, ROW_BOAT_PATTERN_00};
rowBoatPatternSequence seq3 = {ROW_BOAT_PATTERN_FF, ROW_BOAT_PATTERN_55, ROW_BOAT_PATTERN_FF, ROW_BOAT_PATTERN_AA};
rowBoatPatternSequence seq4 = {ROW_BOAT_PATTERN_00, ROW_BOAT_PATTERN_55, ROW_BOAT_PATTERN_00, ROW_BOAT_PATTERN_AA};

M_NODISCARD static bool rowboat_wrc(const tDevice *device, uint32_t pattern, uint8_t *patternBuffer, uint32_t deviceBufferSize, ptrPatternTestResults    testResults)
{
    fill_Pattern_Buffer_Into_Another_Buffer(C_CAST(uint8_t*, &pattern), sizeof(uint32_t), patternBuffer,
                                        deviceBufferSize); // sets the pattern to write into memory
    if (!write_read_compare_pattern(device, patternBuffer, deviceBufferSize, testResults))
    {
        return false;
    }
    return true;
}

static uint32_t set_Rowboat_Pattern_From_Enum(eRowBoatPattern pat)
{
    uint32_t retpat = UINT32_C(0);
    switch (pat)
    {
    case ROW_BOAT_PATTERN_00:
        retpat = UINT32_C(0x00000000);
        break;
    case ROW_BOAT_PATTERN_55:
        retpat = UINT32_C(0x55555555);
        break;
    case ROW_BOAT_PATTERN_AA:
        retpat = UINT32_C(0xAAAAAAAA);
        break;
    case ROW_BOAT_PATTERN_FF:
        retpat = UINT32_C(0xFFFFFFFF);
        break;
    }
    return retpat;
}

static void perform_RowBoat_Pattern_Test(const tDevice*                 device,
                                         rowBoatPatternSequence   patternSequence,
                                         uint32_t                 deviceBufferSize,
                                         ptrPatternTestResults    testResults)
{
    uint32_t numberOfTimesToTest = UINT32_C(5);
    uint8_t* patternBuffer =
        C_CAST(uint8_t*,
               safe_malloc_aligned(deviceBufferSize, device->os_info.minimumAlignment)); // only send this to the drive
    if (patternBuffer)
    {
        DECLARE_SEATIMER(patternTimer);
        start_Timer(&patternTimer);
        for (uint32_t counter = UINT32_C(0); counter < numberOfTimesToTest; ++counter)
        {
            if (!rowboat_wrc(device, set_Rowboat_Pattern_From_Enum(patternSequence.pat1), patternBuffer, deviceBufferSize, testResults))
            {
                break;
            }
            if (!rowboat_wrc(device, set_Rowboat_Pattern_From_Enum(patternSequence.pat2), patternBuffer, deviceBufferSize, testResults))
            {
                break;
            }
            if (!rowboat_wrc(device, set_Rowboat_Pattern_From_Enum(patternSequence.pat3), patternBuffer, deviceBufferSize, testResults))
            {
                break;
            }
            if (!rowboat_wrc(device, set_Rowboat_Pattern_From_Enum(patternSequence.pat4), patternBuffer, deviceBufferSize, testResults))
            {
                break;
            }
        }
        stop_Timer(&patternTimer);
        testResults->totalTimeNS = get_Nano_Seconds(patternTimer);
    }
    safe_free_aligned(&patternBuffer);
}

static void fill_mark_pattern_in_buffer(uint8_t *patternBuffer, uint32_t deviceBufferSize)
{
    #define MARK_PATTERN_SEG_LEN (48)
    #define MARK_PATTERN_TOTAL_LEN (MARK_PATTERN_SEG_LEN * 2)
    uint32_t iter = UINT32_C(0);
    DECLARE_ZERO_INIT_ARRAY(uint8_t, mark0, MARK_PATTERN_SEG_LEN);
    DECLARE_ZERO_INIT_ARRAY(uint8_t, markF, MARK_PATTERN_SEG_LEN);
    uint8_t *mark = mark0;
    safe_memset(markF, MARK_PATTERN_SEG_LEN, 0xFF, MARK_PATTERN_SEG_LEN);
    while (iter < deviceBufferSize)
    {
        safe_memcpy(&patternBuffer[iter], deviceBufferSize - iter, mark, M_Min(MARK_PATTERN_SEG_LEN, deviceBufferSize - iter));
        iter += MARK_PATTERN_SEG_LEN;
        if (mark == mark0)
        {
            mark = markF;
        }
        else
        {
            mark = mark0;
        }
    }
}

static void perform_Mark_Pattern_Test(const tDevice* device, uint32_t deviceBufferSize, ptrPatternTestResults testResults)
{
    uint32_t numberOfTimesToTest = UINT32_C(10);
    uint8_t* patternBuffer =
        C_CAST(uint8_t*,
               safe_malloc_aligned(deviceBufferSize, device->os_info.minimumAlignment)); // only send this to the drive
    if (patternBuffer)
    {
        DECLARE_SEATIMER(patternTimer);
        start_Timer(&patternTimer);
        for (uint32_t counter = UINT32_C(0); counter < numberOfTimesToTest; ++counter)
        {
            fill_mark_pattern_in_buffer(patternBuffer, deviceBufferSize);
            if (!write_read_compare_pattern(device, patternBuffer, deviceBufferSize, testResults))
            {
                break;
            }
        }
        stop_Timer(&patternTimer);
        testResults->totalTimeNS = get_Nano_Seconds(patternTimer);
    }
    safe_free_aligned(&patternBuffer);
}

static bool fill_walking_test_pattern_in_buffer(uint8_t *patternBuffer, uint32_t deviceBufferSize, bool walkingZeros, uint32_t bitNumber, uint32_t *byteNumber)
{
    safe_memset(patternBuffer, deviceBufferSize, walkingZeros ? 0xFF : 0x00, deviceBufferSize);
    if (bitNumber > UINT32_C(7))
    {
        // this means we've shifted the bit through each bit of this byte, so offset to the next byte and start
        // again
        ++(*byteNumber);
        bitNumber = UINT32_C(0);
        if (*byteNumber >= deviceBufferSize)
        {
            return false;
        }
    }
    if (walkingZeros)
    {
        patternBuffer[*byteNumber] = clear_uint8_bit(patternBuffer[*byteNumber], bitNumber);
    }
    else
    {
        patternBuffer[*byteNumber] = set_uint8_bit(patternBuffer[*byteNumber], bitNumber);
    }
    return true;
}

// Function for Walking 1's/0's test
static void perform_Walking_Test(const tDevice*        device,
                                 bool                  walkingZeros,
                                 uint32_t              deviceBufferSize,
                                 ptrPatternTestResults testResults)
{
    uint8_t* patternBuffer = M_REINTERPRET_CAST(
        uint8_t*, safe_calloc_aligned(deviceBufferSize, sizeof(uint8_t),
                                      device->os_info.minimumAlignment)); // only send this to the drive
    if (patternBuffer)
    {
        DECLARE_SEATIMER(patternTimer);
        start_Timer(&patternTimer);
        for (uint32_t bitNumber = UINT32_C(0), byteNumber = UINT32_C(0); byteNumber < deviceBufferSize; ++bitNumber)
        {
            if (!fill_walking_test_pattern_in_buffer(patternBuffer, deviceBufferSize, walkingZeros, bitNumber,
                                                    &byteNumber))
            {
                break; // finished all bits in the buffer
            }
            if (!write_read_compare_pattern(device, patternBuffer, deviceBufferSize, testResults))
            {
                break;
            }
        }
        stop_Timer(&patternTimer);
        testResults->totalTimeNS = get_Nano_Seconds(patternTimer);
    }
    safe_free_aligned(&patternBuffer);
}
// Function for random data pattern test
static void perform_Random_Pattern_Test(const tDevice*        device,
                                        uint32_t              deviceBufferSize,
                                        ptrPatternTestResults testResults)
{
    uint32_t numberOfTimesToTest = UINT32_C(10);
    uint8_t* patternBuffer =
        C_CAST(uint8_t*,
               safe_malloc_aligned(deviceBufferSize, device->os_info.minimumAlignment)); // only send this to the drive
    if (patternBuffer)
    {
        DECLARE_SEATIMER(patternTimer);
        start_Timer(&patternTimer);
        for (uint32_t counter = UINT32_C(0); counter < numberOfTimesToTest; ++counter)
        {
            fill_Random_Pattern_In_Buffer(patternBuffer, deviceBufferSize); // set a new random pattern each time
            if (!write_read_compare_pattern(device, patternBuffer, deviceBufferSize, testResults))
            {
                break;
            }
        }
        stop_Timer(&patternTimer);
        testResults->totalTimeNS = get_Nano_Seconds(patternTimer);
    }
    safe_free_aligned(&patternBuffer);
}

// row boat test: device, inverting pattern, static pattern, bool startStatic
// start static to start with the static pattern or the alternating pattern

// row boat:
// 55->FF->AA->FF
// 55->00->AA->00
// FF->55->FF->AA
// 00->55->00->AA

// mark pattern: 48 F's, 48 0's, so on and so forth

// SATA Phy event counters: CRC = definitely bad
//                          R_ERR = multiple possible causes from bad connection to bad cable. Recommend redoing the
//                          connection or replacing cable.
// SATA Device statistics: CRC = definitely bad
//                         ASR events = bad cable as well. (asynchronous signal recovery)
// SAS SPL error counters: Invalid Dword = definitely bad
//                         Running disparity or loss of sync = multiple possible causes from bad connection to bad
//                         cable. Recommend redoing the connection or replacing cable.
// Slower interface speed = longer test time to get a confident result.

// master function for the whole test.
eReturnValues perform_Cable_Test(const tDevice* device, ptrCableTestResults testResults)
{
    eReturnValues ret = SUCCESS;
    DISABLE_NONNULL_COMPARE
    if (testResults == M_NULLPTR)
    {
        return BAD_PARAMETER;
    }
    RESTORE_NONNULL_COMPARE
    if (are_Buffer_Commands_Available(device))
    {
        uint8_t  offsetPO2  = UINT8_C(0); // This shouldn't actually be needed...but I have it here in case I do
        uint32_t bufferSize = UINT32_C(0);
        if (SUCCESS == get_Buffer_Size(device, &bufferSize, &offsetPO2) && bufferSize > 0)
        {
            DECLARE_SEATIMER(totalTestingTime);
            // drive supports the read/write buffer commands we need and we know what size the buffer is we can test
            // with. now we need to begin testing.
            safe_memset(testResults, sizeof(cableTestResults), 0, sizeof(cableTestResults));
            // first, lets do some simple data patterns (0's, F's, 5's, A's)
            start_Timer(&totalTestingTime);
            puts("Zeros test");
            for (uint8_t count = UINT8_C(0); count < ALL_0_TEST_COUNT; ++count)
            {
                perform_Byte_Pattern_Test(
                    device, UINT32_C(0x00000000), bufferSize,
                    &testResults->zerosTest[count]); // arbitrary number 10 was chosen since it sounded good for number
                                                     // of times to try this pattern
            }
            puts("F's test");
            for (uint8_t count = UINT8_C(0); count < ALL_F_TEST_COUNT; ++count)
            {
                perform_Byte_Pattern_Test(
                    device, UINT32_C(0xFFFFFFFF), bufferSize,
                    &testResults->fTest[count]); // arbitrary number 10 was chosen since it sounded good for number of
                                                 // times to try this pattern
            }
            puts("5's test");
            for (uint8_t count = UINT8_C(0); count < ALL_5_TEST_COUNT; ++count)
            {
                perform_Byte_Pattern_Test(
                    device, UINT32_C(0x55555555), bufferSize,
                    &testResults->fivesTest[count]); // arbitrary number 10 was chosen since it sounded good for number
                                                     // of times to try this pattern
            }
            puts("A's test");
            for (uint8_t count = UINT8_C(0); count < ALL_A_TEST_COUNT; ++count)
            {
                perform_Byte_Pattern_Test(
                    device, UINT32_C(0xAAAAAAAA), bufferSize,
                    &testResults->aTest[count]); // arbitrary number 10 was chosen since it sounded good for number of
                                                 // times to try this pattern
            }
            // checker board - byte
            puts("Checkerboard (Byte) test");
            for (uint8_t count = UINT8_C(0); count < CHECKER_BOARD_TEST_COUNT; ++count)
            {
                perform_Byte_Pattern_Test(
                    device, UINT32_C(0x55AA55AA), bufferSize,
                    &testResults->checkerBoardByte[count]); // arbitrary number 10 was chosen since it sounded good for number of
                                                 // times to try this pattern
            }
            // checker board - word
            puts("Checkerboard (Word) test");
            for (uint8_t count = UINT8_C(0); count < CHECKER_BOARD_TEST_COUNT; ++count)
            {
                perform_Byte_Pattern_Test(
                    device, UINT32_C(0x5555AAAA), bufferSize,
                    &testResults->checkerBoardWord[count]); // arbitrary number 10 was chosen since it sounded good for number of
                                                 // times to try this pattern
            }
            // mark
            puts("Mark test");
            for (uint8_t count = UINT8_C(0); count < MARK_TEST_COUNT; ++count)
            {
                perform_Mark_Pattern_Test(device, bufferSize, &testResults->mark[count]);
            }
            
            // Row Boat
            puts("Rowboat 00-FF-55-AA test");
            for (uint8_t count = UINT8_C(0); count < ZERO_F_5_A_TEST_COUNT; ++count)
            {
                rowBoatPatternSequence myRowBoat = {ROW_BOAT_PATTERN_00, ROW_BOAT_PATTERN_FF, ROW_BOAT_PATTERN_55, ROW_BOAT_PATTERN_AA};
                perform_RowBoat_Pattern_Test(device, myRowBoat, bufferSize, &testResults->zeroF5ATest[count]);
            }
            puts("Rowboat 1 test");
            for (uint8_t count = UINT8_C(0); count < ROW_BOAT_TEST_COUNT; ++count)
            {
                perform_RowBoat_Pattern_Test(device, seq1, bufferSize, &testResults->rowBoat1[count]);
            }
            puts("Rowboat 2 test");
            for (uint8_t count = UINT8_C(0); count < ROW_BOAT_TEST_COUNT; ++count)
            {
                perform_RowBoat_Pattern_Test(device, seq2, bufferSize, &testResults->rowBoat2[count]);
            }
            puts("Rowboat 3 test");
            for (uint8_t count = UINT8_C(0); count < ROW_BOAT_TEST_COUNT; ++count)
            {
                perform_RowBoat_Pattern_Test(device, seq3, bufferSize, &testResults->rowBoat3[count]);
            }
            puts("Rowboat 4 test");
            for (uint8_t count = UINT8_C(0); count < ROW_BOAT_TEST_COUNT; ++count)
            {
                perform_RowBoat_Pattern_Test(device, seq4, bufferSize, &testResults->rowBoat4[count]);
            }
            // now walking 1's
            puts("Walking 1's test");
            for (uint8_t count = UINT8_C(0); count < WALKING_1_TEST_COUNT; ++count)
            {
                perform_Walking_Test(device, false, bufferSize,
                                     &testResults->walking1sTest[count]); // arbitrary number 5 was chose since it
                                                                          // sounded good for trying this test
            }
            // walking 0's
            puts("Walking 0's test");
            for (uint8_t count = UINT8_C(0); count < WALKING_0_TEST_COUNT; ++count)
            {
                perform_Walking_Test(device, true, bufferSize,
                                     &testResults->walking0sTest[count]); // arbitrary number 5 was chose since it
                                                                          // sounded good for trying this test
            }
            // random data patterns
            puts("Random test");
            for (uint8_t count = UINT8_C(0); count < RANDOM_TEST_COUNT; ++count)
            {
                perform_Random_Pattern_Test(
                    device, bufferSize,
                    &testResults->randomTest[count]); // arbitrary number 10 was chosen since it sounded good for number
                                                      // of times to try this pattern
            }
            stop_Timer(&totalTestingTime);
            testResults->totalTestTimeNS = get_Nano_Seconds(totalTestingTime);
        }
        else
        {
            ret = NOT_SUPPORTED;
        }
    }
    else
    {
        ret = NOT_SUPPORTED;
    }
    return ret;
}

static void print_Individual_Test_Results(const char* testname, ptrPatternTestResults results, size_t maxcount)
{
    if (results != M_NULLPTR)
    {
        print_str(testname);
        print_str("\n");
        for (uint8_t count = UINT8_C(0); count < maxcount; ++count)
        {
            printf("    Run %" PRIu8 ":\n", count + UINT8_C(1));
            printf("        Total commands sent: %" PRIu32 "\n", results[count].totalCommandsSent);
            printf("        Number of command CRC errors: %" PRIu32 "\n",
                results[count].totalCommandCRCErrors);
            printf("        Number of command timeouts: %" PRIu32 "\n", results[count].totalCommandTimeouts);
            printf("        Number of buffer comparisons: %" PRIu32 "\n",
                results[count].totalBufferComparisons);
            printf("        Number of buffer miscompares: %" PRIu32 "\n",
                results[count].totalBufferMiscompares);
            print_str("        Test time: ");
            print_Command_Time(results[count].totalTimeNS);
            print_str("\n");
        }
    }
}

void print_Cable_Test_Results(cableTestResults testResults)
{
    print_str("Test Results:\n");
    print_str("=============\n");
    print_str("Total test time: ");
    print_Command_Time(testResults.totalTestTimeNS);
    printf("\n");
    print_Individual_Test_Results("00h Test Pattern", testResults.zerosTest, ALL_0_TEST_COUNT);
    print_Individual_Test_Results("FFh Test Pattern", testResults.fTest, ALL_F_TEST_COUNT);
    print_Individual_Test_Results("55h Test Pattern", testResults.fivesTest, ALL_5_TEST_COUNT);
    print_Individual_Test_Results("AAh Test Pattern", testResults.aTest, ALL_A_TEST_COUNT);
    print_Individual_Test_Results("Checkerboard (Byte) Test Pattern", testResults.checkerBoardByte, CHECKER_BOARD_TEST_COUNT);
    print_Individual_Test_Results("Checkerboard (Word) Test Pattern", testResults.checkerBoardWord, CHECKER_BOARD_TEST_COUNT);
    print_Individual_Test_Results("Mark Test Pattern", testResults.mark, MARK_TEST_COUNT);
    print_Individual_Test_Results("Rowboat 00FF55AAh Test Pattern", testResults.zeroF5ATest, ZERO_F_5_A_TEST_COUNT);
    print_Individual_Test_Results("Rowboat 1 Test Pattern", testResults.rowBoat1, ROW_BOAT_TEST_COUNT);
    print_Individual_Test_Results("Rowboat 2 Test Pattern", testResults.rowBoat2, ROW_BOAT_TEST_COUNT);
    print_Individual_Test_Results("Rowboat 3 Test Pattern", testResults.rowBoat3, ROW_BOAT_TEST_COUNT);
    print_Individual_Test_Results("Rowboat 4 Test Pattern", testResults.rowBoat4, ROW_BOAT_TEST_COUNT);
    print_Individual_Test_Results("Walking 1's Test Pattern", testResults.walking1sTest, WALKING_1_TEST_COUNT);
    print_Individual_Test_Results("Walking 0's Test Pattern", testResults.walking0sTest, WALKING_0_TEST_COUNT);
    print_Individual_Test_Results("Random Test Pattern", testResults.randomTest, RANDOM_TEST_COUNT);
}


