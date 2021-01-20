#ifndef PLC_HPP
#define PLC_HPP

#include <string>

const std::string softmanbotStatusMasterTag = "PLC_REQUEST";
const std::string softmanbotStatusValidationTag = "SMB_GOT_REQUEST";
const std::string softmanbotStatusStepTag = "SMB_STEP";

typedef enum
{
	STATUS_REQUESTED,
	WORK_IN_PROGRESS,
	JOBS_DONE
}StepStatus;

const std::string watchdogMasterTag = "PLC_WATCHDOG";
const std::string watchdogValidateTag = "SMB_WATCHDOG";

void PLC_Init(void);
void PLC_WriteTag(const std::string &tagName, int valueToWrite);
int  PLC_ReadTag(const std::string &tagName);
void PLC_End(void);

#endif //PLC_HPP
