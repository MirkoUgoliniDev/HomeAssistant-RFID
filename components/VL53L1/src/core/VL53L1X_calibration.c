/*
* Copyright (c) 2017, STMicroelectronics - All Rights Reserved
*
* This file : part of VL53L1 Core and : dual licensed,
* either 'STMicroelectronics
* Proprietary license'
* or 'BSD 3-clause "New" or "Revised" License' , at your option.
*
********************************************************************************
*
* 'STMicroelectronics Proprietary license'
*
********************************************************************************
*
* License terms: STMicroelectronics Proprietary in accordance with licensing
* terms at www.st.com/sla0081
*
* STMicroelectronics confidential
* Reproduction and Communication of this document : strictly prohibited unless
* specifically authorized in writing by STMicroelectronics.
*
*
********************************************************************************
*
* Alternatively, VL53L1 Core may be distributed under the terms of
* 'BSD 3-clause "New" or "Revised" License', in which case the following
* provisions apply instead of the ones mentioned above :
*
********************************************************************************
*
* License terms: BSD 3-clause "New" or "Revised" License.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
* may be used to endorse or promote products derived from this software
* without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*
********************************************************************************
*
*/
/**
 * @file  vl53l1x_calibration.c
 * @brief Calibration functions implementation
 */
#include "VL53L1X_api.h"
#include "VL53L1X_calibration.h"
#include "esp_log.h"
#include <inttypes.h>

static const char *TAG = "VL53L1X_CAL";


#define ALGO__PART_TO_PART_RANGE_OFFSET_MM	0x001E
#define MM_CONFIG__INNER_OFFSET_MM			0x0020
#define MM_CONFIG__OUTER_OFFSET_MM 			0x0022


int8_t VL53L1X_CalibrateOffset(uint16_t dev, uint16_t TargetDistInMm, int16_t *offset){
	uint8_t i, tmp;
	int16_t AverageDistance = 0;
	uint16_t distance;
	VL53L1X_ERROR status = 0;

	ESP_LOGI(TAG, "Starting offset calibration for device 0x%02X to target %u mm", dev, TargetDistInMm);
	
	status = VL53L1_WrWord(dev, ALGO__PART_TO_PART_RANGE_OFFSET_MM, 0x0);
	status = VL53L1_WrWord(dev, MM_CONFIG__INNER_OFFSET_MM, 0x0);
	status = VL53L1_WrWord(dev, MM_CONFIG__OUTER_OFFSET_MM, 0x0);
	
	ESP_LOGI(TAG, "Reset current offsets, starting ranging");
	status = VL53L1X_StartRanging(dev);	/* Enable VL53L1X sensor */
	
	for (i = 0; i < 50; i++) {
		tmp = 0;
		while (tmp == 0){
			status = VL53L1X_CheckForDataReady(dev, &tmp);
		}
		status = VL53L1X_GetDistance(dev, &distance);
		status = VL53L1X_ClearInterrupt(dev);
		AverageDistance = AverageDistance + distance;
		
		ESP_LOGD(TAG, "Offset calibration sample %u: distance = %u mm", i, distance);
	}
	
	status = VL53L1X_StopRanging(dev);
	AverageDistance = AverageDistance / 50;
	*offset = TargetDistInMm - AverageDistance;
	
	ESP_LOGI(TAG, "Offset calibration complete: Average distance = %d mm, Calculated offset = %d mm", AverageDistance, *offset);
	        
	status = VL53L1_WrWord(dev, ALGO__PART_TO_PART_RANGE_OFFSET_MM, *offset*4);
	ESP_LOGI(TAG, "Wrote calibrated offset to device register: %d (scaled: %d)", *offset, *offset*4);
	
	return status;
}



int8_t VL53L1X_CalibrateXtalk(uint16_t dev, uint16_t TargetDistInMm, uint16_t *xtalk){
	uint8_t i, tmp;
	float AverageSignalRate = 0;
	float AverageDistance = 0;
	float AverageSpadNb = 0;
	uint16_t distance = 0, spadNum;
	uint16_t sr;
	VL53L1X_ERROR status = 0;
	uint32_t calXtalk;

	ESP_LOGI(TAG, "Starting crosstalk calibration for device 0x%02X at target %u mm", dev, TargetDistInMm);
	
	status = VL53L1_WrWord(dev, 0x0016, 0);
	ESP_LOGI(TAG, "Reset current crosstalk, starting ranging");
	
	status = VL53L1X_StartRanging(dev);
	for (i = 0; i < 50; i++) {
		tmp = 0;
		while (tmp == 0){
			status = VL53L1X_CheckForDataReady(dev, &tmp);
		}
		status = VL53L1X_GetSignalRate(dev, &sr);
		status = VL53L1X_GetDistance(dev, &distance);
		status = VL53L1X_ClearInterrupt(dev);
		status = VL53L1X_GetSpadNb(dev, &spadNum);
		
		AverageDistance = AverageDistance + distance;
		AverageSpadNb = AverageSpadNb + spadNum;
		AverageSignalRate = AverageSignalRate + sr;
		
		ESP_LOGD(TAG, "Xtalk calibration sample %u: distance = %u mm, signal rate = %u, SPAD count = %u", 
		        i, distance, sr, spadNum);
	}
	
	status = VL53L1X_StopRanging(dev);
	
	AverageDistance = AverageDistance / 50;
	AverageSpadNb = AverageSpadNb / 50;
	AverageSignalRate = AverageSignalRate / 50;
	
	ESP_LOGI(TAG, "Calibration measurements: Avg distance = %.2f mm, Avg signal rate = %.2f, Avg SPAD count = %.2f", 
	        AverageDistance, AverageSignalRate, AverageSpadNb);
	
	/* Calculate Xtalk value */
	calXtalk = (uint16_t)(512*(AverageSignalRate*(1-(AverageDistance/TargetDistInMm)))/AverageSpadNb);
	*xtalk = (uint16_t)((calXtalk*1000)>>9);
	
	ESP_LOGI(TAG, "Calculated crosstalk: raw value = %"PRIu32", calibrated value = %u", calXtalk, *xtalk);
	
	status = VL53L1_WrWord(dev, 0x0016, (uint16_t)calXtalk);
	ESP_LOGI(TAG, "Wrote calibrated crosstalk value to device register");
	
	return status;
}