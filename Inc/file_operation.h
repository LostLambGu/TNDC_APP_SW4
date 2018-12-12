/*******************************************************************************
* File Name          : file_operation.h
* Author             : Yangjie Gu
* Description        : This file provides all the file_operation functions.

* History:
*  10/25/2017 : file_operation V1.00
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FILE_OPERATION_H__
#define __FILE_OPERATION_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "fatfs.h"
#include "prot.h"

#define MAX_FILE_NAME_LEN (31 + 1)

typedef struct FileList_t FileListTypeDef;
	
struct FileList_t
{
    FileListTypeDef *pAhead;
    uint8_t FileNum;
    uint8_t FileIndex;
    TCHAR FileName[MAX_FILE_NAME_LEN];
    FIL *File;
    FileListTypeDef *pNext;
};

/* Variables -----------------------------------------------------------------*/
extern const BYTE OpenMode[OEM_FSI_FILE_OPEN_SHARE + 1];
extern const OemValFsiResultT FileOperationRet[FR_INVALID_PARAMETER + 1];
extern FileListTypeDef FileListHead;

/* Exported functions --------------------------------------------------------*/
extern OemValFsiResultT FileOpen(OemValFsiHandleT *FileP, const char *FileNameP, OemValFsiFileOpenModeT Mode);
extern OemValFsiResultT FileClose(OemValFsiHandleT File);
extern OemValFsiResultT FileRead(void *BufferP, uint32_t ItemSize, uint32_t *ItemNumP, OemValFsiHandleT File);
extern OemValFsiResultT FileWrite(void *BufferP, uint32_t ItemSize, uint32_t *ItemNumP, OemValFsiHandleT File);
extern OemValFsiResultT FileSeek(OemValFsiHandleT File, OemValFsiFileSeekTypeT SeekFrom, int32_t MoveDistance);
extern OemValFsiResultT FileTell(OemValFsiHandleT File, uint32_t *PosP);
extern OemValFsiResultT FileRemove(const char *NameP);
extern OemValFsiResultT FileRename(const char *OldNameP, const char *NewNameP);
extern OemValFsiResultT FileGetLength(const char *NameP, uint32_t *FileLengthP);
// extern OemValFsiResultT FileVerify(char *NameP);

#ifdef __cplusplus
}
#endif

#endif /* __FILE_OPERATION_H__ */

/*******************************************************************************
    Copyrights (C) Asiatelco Technologies Co., 2003-2017. All rights reserved
                                End Of The File
*******************************************************************************/
