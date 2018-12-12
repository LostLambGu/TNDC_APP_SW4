/*******************************************************************************
* File Name          : file_operation.c
* Author             : Yangjie Gu
* Description        : This file provides all the file_operation functions.

* History:
*  10/25/2017 : file_operation V1.00
*******************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include <string.h>

#include "file_operation.h"
#include "uart_api.h"

#define FILE_OP_PRINT DebugPrintf

/* Variables -----------------------------------------------------------------*/
const BYTE OpenMode[OEM_FSI_FILE_OPEN_SHARE + 1] =
    {
            [OEM_FSI_FILE_OPEN_READ_EXIST] = FA_READ | FA_OPEN_EXISTING,
            [OEM_FSI_FILE_OPEN_WRITE_EXIST] = FA_READ | FA_WRITE | FA_OPEN_EXISTING,
            [OEM_FSI_FILE_OPEN_CREATE_NEW] = FA_CREATE_NEW,
            [OEM_FSI_FILE_OPEN_CREATE_ALWAYS] = FA_WRITE |FA_CREATE_ALWAYS,
            [OEM_FSI_FILE_OPEN_WRITE_ALWAYS] = FA_WRITE | FA_OPEN_ALWAYS,
            [OEM_FSI_FILE_OPEN_SHARE] = FA_READ | FA_WRITE | FA_OPEN_ALWAYS,
};

const OemValFsiResultT FileOperationRet[FR_INVALID_PARAMETER + 1] =
    {
            // FR_OK = 0,				/* (0) Succeeded */
            [FR_OK] = OEM_FSI_SUCCESS,
            // FR_DISK_ERR,			/* (1) A hard error occurred in the low level disk I/O layer */
            [FR_DISK_ERR] = OEM_FSI_ERR_UNKNOWN,
            // FR_INT_ERR,				/* (2) Assertion failed */
            [FR_INT_ERR] = OEM_FSI_ERR_UNKNOWN,
            // FR_NOT_READY,			/* (3) The physical drive cannot work */
            [FR_NOT_READY] = OEM_FSI_ERR_UNKNOWN,
            // FR_NO_FILE,				/* (4) Could not find the file */
            [FR_NO_FILE] = OEM_FSI_ERR_NOTEXIST,
            // FR_NO_PATH,				/* (5) Could not find the path */
            [FR_NO_PATH] = OEM_FSI_ERR_UNKNOWN,
            // FR_INVALID_NAME,		/* (6) The path name format is invalid */
            [FR_INVALID_NAME] = OEM_FSI_ERR_UNKNOWN,
            // FR_DENIED,				/* (7) Access denied due to prohibited access or directory full */
            [FR_DENIED] = OEM_FSI_ERR_UNKNOWN,
            // FR_EXIST,				/* (8) Access denied due to prohibited access */
            [FR_EXIST] = OEM_FSI_ERR_ACCESS_DENY,
            // FR_INVALID_OBJECT,		/* (9) The file/directory object is invalid */
            [FR_INVALID_OBJECT] = OEM_FSI_ERR_UNKNOWN,
            // FR_WRITE_PROTECTED,		/* (10) The physical drive is write protected */
            [FR_WRITE_PROTECTED] = OEM_FSI_ERR_WRITE,
            // FR_INVALID_DRIVE,		/* (11) The logical drive number is invalid */
            [FR_INVALID_DRIVE] = OEM_FSI_ERR_UNKNOWN,
            // FR_NOT_ENABLED,			/* (12) The volume has no work area */
            [FR_NOT_ENABLED] = OEM_FSI_ERR_UNKNOWN,
            // FR_NO_FILESYSTEM,		/* (13) There is no valid FAT volume */
            [FR_NO_FILESYSTEM] = OEM_FSI_ERR_FORMAT,
            // FR_MKFS_ABORTED,		/* (14) The f_mkfs() aborted due to any parameter error */
            [FR_MKFS_ABORTED] = OEM_FSI_ERR_UNKNOWN,
            // FR_TIMEOUT,				/* (15) Could not get a grant to access the volume within defined period */
            [FR_TIMEOUT] = OEM_FSI_ERR_TIMEOUT,
            // FR_LOCKED,				/* (16) The operation is rejected according to the file sharing policy */
            [FR_LOCKED] = OEM_FSI_ERR_UNKNOWN,
            // FR_NOT_ENOUGH_CORE,		/* (17) LFN working buffer could not be allocated */
            [FR_NOT_ENOUGH_CORE] = OEM_FSI_ERR_UNKNOWN,
            // FR_TOO_MANY_OPEN_FILES,	/* (18) Number of open files > _FS_SHARE */
            [FR_TOO_MANY_OPEN_FILES] = OEM_FSI_ERR_UNKNOWN,
            // FR_INVALID_PARAMETER	/* (19) Given parameter is invalid */
            [FR_INVALID_PARAMETER] = OEM_FSI_ERR_PARAMETER,
};

FileListTypeDef FileListHead = {0};
uint8_t FileSystemMountFlag = 0;

static int8_t FileListInsert(const char *FileNameP)
{
    FileListTypeDef *ptmp = NULL;
    FileListTypeDef *pRec = NULL;
    int8_t ret = 0;

    if (FileNameP == NULL)
    {
        return -1;
    }
    else
    {
        taskENTER_CRITICAL();
        if ((FileListHead.FileNum == 0) || (FileListHead.File == NULL))
        {
            FileListHead.pAhead = NULL;
            FileListHead.FileIndex = 0;
            memset(FileListHead.FileName, 0, MAX_FILE_NAME_LEN);
            strncpy(FileListHead.FileName, FileNameP, 
                (strlen(FileNameP) >= (MAX_FILE_NAME_LEN - 1)) ? (MAX_FILE_NAME_LEN - 1) : strlen(FileNameP));
            memset(&EMMCFile, 0, sizeof(EMMCFile));
            FileListHead.File = &EMMCFile;
            // FileListHead.pNext = NULL;
            FileListHead.FileNum++;
            ret = 0;
        }
        else
        {
            pRec = &FileListHead;
            ptmp = pvPortMalloc(sizeof(FileListTypeDef));
            if (ptmp == NULL)
            {
                taskEXIT_CRITICAL();
                return -2;
            }
            memset(ptmp, 0, sizeof(FileListTypeDef));
            ptmp->File = pvPortMalloc(sizeof(FIL));
            if (ptmp->File == NULL)
            {
                if (ptmp != NULL)
                    vPortFree(ptmp);
                taskEXIT_CRITICAL();
                return -3;
            }
            memset(ptmp->File, 0, sizeof(FIL));
            strncpy(ptmp->FileName, FileNameP, 
                (strlen(FileNameP) >= (MAX_FILE_NAME_LEN - 1)) ? (MAX_FILE_NAME_LEN - 1) : strlen(FileNameP));

            while (pRec->pNext != NULL)
            {
                if ((pRec->FileIndex + 1) == pRec->pNext->FileIndex)
                {
                    pRec = pRec->pNext;
                }
                else
                {
                    break;
                }
            }
            ptmp->FileIndex = pRec->FileIndex + 1;
            ptmp->pAhead = pRec;
            ptmp->pNext = pRec->pNext;
            pRec->pNext = ptmp;
            if (ptmp->pNext != NULL)
            {
                ptmp->pNext->pAhead = ptmp;
            }
            FileListHead.FileNum++;
            ret = ptmp->FileIndex;
        }
        taskEXIT_CRITICAL();
        return ret;
    }
}

static void FileListDelete(uint8_t FileIndex)
{
    FileListTypeDef *pRec = NULL;
    pRec = &FileListHead;

    if (pRec->FileNum != 0)
    {
        taskENTER_CRITICAL();

        while (pRec->FileIndex != FileIndex)
        {
            if (pRec->pNext != NULL)
            {
                pRec = pRec->pNext;
            }
            else
            {
                taskEXIT_CRITICAL();
                return;
            }
        }
        if (pRec->pAhead != NULL)
        {
            pRec->pAhead->pNext = pRec->pNext;
        }
        if (FileIndex != 0)
        {
            if (pRec->pNext != NULL)
            {
                pRec->pNext->pAhead = pRec->pAhead;
            }
        }

        if (pRec->File == &EMMCFile)
        {
            memset(&EMMCFile, 0, sizeof(EMMCFile));
            FileListHead.File = NULL;
        }
        else if (pRec->File != NULL)
        {
            vPortFree(pRec->File);
        }
        else
        {

        }

        if (pRec == &FileListHead)
        {
            memset(FileListHead.FileName, 0, MAX_FILE_NAME_LEN);
        }
        else
        {
            vPortFree(pRec);
        }
        FileListHead.FileNum--;
        if (FileListHead.FileNum == 0)
        {
            FileListHead.pNext = NULL;
        }
        
        taskEXIT_CRITICAL();
    }
    else
    {
        return;
    }
}

static FIL *FileListGetFIL(uint8_t FileIndex)
{
    FileListTypeDef *pRec = NULL;
    pRec = &FileListHead;

    if (pRec->FileNum != 0)
    {
        taskENTER_CRITICAL();

        while (pRec->FileIndex != FileIndex)
        {
            if (pRec->pNext != NULL)
            {
                pRec = pRec->pNext;
            }
            else
            {
                taskEXIT_CRITICAL();
                return NULL;
            }
        }
        taskEXIT_CRITICAL();
        return pRec->File;
    }
    else
    {
        return NULL;
    }
}

static FIL *FileListGetFILName(const char *FileNameP)
{
    FileListTypeDef *pRec = NULL;
    pRec = &FileListHead;

    if (pRec->FileNum != 0)
    {
        taskENTER_CRITICAL();

        while ((strncmp(pRec->FileName, FileNameP, 
            (strlen(FileNameP) >= (MAX_FILE_NAME_LEN - 1)) ? 
            (MAX_FILE_NAME_LEN - 1) : strlen(FileNameP))) != 0)
        {
            if (pRec->pNext != NULL)
            {
                pRec = pRec->pNext;
            }
            else
            {
                taskEXIT_CRITICAL();
                return NULL;
            }
        }
        taskEXIT_CRITICAL();
        return pRec->File;
    }
    else
    {
        return NULL;
    }
}

/* Function prototypes -------------------------------------------------------*/
OemValFsiResultT FileOpen(OemValFsiHandleT *FileP, const char *FileNameP, OemValFsiFileOpenModeT Mode)
{
    FRESULT res;
    int8_t file;

    if ((FileP == NULL) || (FileNameP == NULL) || (Mode > OEM_FSI_FILE_OPEN_SHARE))
        return OEM_FSI_ERR_PARAMETER;

    file = FileListInsert(FileNameP);
    *FileP = (file < 0) ? 0xffff : file;
    if (*FileP == 0xffff)
    {
        FILE_OP_PRINT(DbgCtl.FileInfoEn, "\r\nFile allocate fail!");
        return OEM_FSI_ERR_MOUNTED;
    }

    if (FileSystemMountFlag == 0)
    {
        if (0 == FATFS_GetAttachedDriversNbr())
            MX_FATFS_Init();

        res = f_mount(&EMMCFatFS, EMMCPath, 1);

        if (res == FR_OK)
        {
            FileSystemMountFlag = 1;
        }
        else
        {
            FileListDelete(*FileP);
            FILE_OP_PRINT(DbgCtl.FileInfoEn, "\r\nEMMC FatFs mount failed:%d", res);
            *FileP = 0xffff;
            return FileOperationRet[res];
        }
    }

    res = f_open(FileListGetFIL(*FileP), FileNameP, OpenMode[Mode]);
    if (res == FR_OK)
    {
//        FILE_OP_PRINT(DbgCtl.FileInfoEn, "\r\nEMMC FatFs open success %s!", FileNameP);
    }
    else
    {
        FileListDelete(*FileP);
        FILE_OP_PRINT(DbgCtl.FileInfoEn, "\r\nEMMC FatFs open failed! %s!", FileNameP);
        *FileP = 0xffff;
        return FileOperationRet[res];
    }

    return FileOperationRet[res];
}

OemValFsiResultT FileClose(OemValFsiHandleT File)
{
    FRESULT res;

    if (File >= 0xff)
    {
        return OEM_FSI_ERR_PARAMETER;
    }

    res = f_close(FileListGetFIL(File));
    if (res == FR_OK)
    {
        FileListDelete(File);
    //    FILE_OP_PRINT(DbgCtl.FileInfoEn, "\r\nEMMC FatFs close success!");
    }
    else
    {
       FILE_OP_PRINT(DbgCtl.FileInfoEn, "\r\nEMMC FatFs close failed(%d)!", res);
    }

    return FileOperationRet[res];
}

OemValFsiResultT FileRead(void *BufferP, uint32_t ItemSize, uint32_t *ItemNumP, OemValFsiHandleT File)
{
    FRESULT res;
    uint32_t rcount = 0;

    if ((BufferP == NULL) || (ItemSize == 0) || (ItemNumP == NULL) || (*ItemNumP == 0))
        return OEM_FSI_ERR_PARAMETER;

    res = f_read(FileListGetFIL(File), BufferP, ItemSize * (*ItemNumP), &rcount);
	*ItemNumP = rcount;

    return FileOperationRet[res];
}

OemValFsiResultT FileWrite(void *BufferP, uint32_t ItemSize, uint32_t *ItemNumP, OemValFsiHandleT File)
{
    FRESULT res;
    uint32_t wcount = 0;

    if ((BufferP == NULL) || (ItemSize == 0) || (ItemNumP == NULL) || (*ItemNumP == 0))
        return OEM_FSI_ERR_PARAMETER;

    res = f_write(FileListGetFIL(File), BufferP, ItemSize * (*ItemNumP), &wcount);
    if(res != 0 ) FILE_OP_PRINT(DbgCtl.FileInfoEn, "\r\nEMMC FatFs write error %d!", res);
    return FileOperationRet[res];
}

OemValFsiResultT FileSeek(OemValFsiHandleT File, OemValFsiFileSeekTypeT SeekFrom, int32_t MoveDistance)
{
    DWORD size = 0;
    FRESULT res;

    if ((SeekFrom != OEM_FSI_FILE_SEEK_START) && (SeekFrom != OEM_FSI_FILE_SEEK_END) && (SeekFrom != OEM_FSI_FILE_SEEK_CURRENT))
        return OEM_FSI_ERR_PARAMETER;

    if (SeekFrom == OEM_FSI_FILE_SEEK_START)
    {
        if (MoveDistance < 0)
            return OEM_FSI_ERR_PARAMETER;

        size = 0 + MoveDistance;

        if (size > f_size(FileListGetFIL(File)))
            return OEM_FSI_ERR_PARAMETER;
    }
    else if (SeekFrom == OEM_FSI_FILE_SEEK_END)
    {
        if (MoveDistance > 0)
            return OEM_FSI_ERR_PARAMETER;

        if (((int32_t)f_size(FileListGetFIL(File)) + MoveDistance) < 0)
            return OEM_FSI_ERR_PARAMETER;

        size = f_size(FileListGetFIL(File)) + MoveDistance;
    }
    else
    {
        if (((int32_t)(f_tell(FileListGetFIL(File)) + MoveDistance) < 0) || (((int32_t)(f_tell(FileListGetFIL(File)) + MoveDistance) > f_size(FileListGetFIL(File)))))
            return OEM_FSI_ERR_PARAMETER;

        size = f_tell(FileListGetFIL(File)) + MoveDistance;
    }

    res = f_lseek(FileListGetFIL(File), size);

    return FileOperationRet[res];
}

OemValFsiResultT FileTell(OemValFsiHandleT File, uint32_t *PosP)
{
    if (PosP == NULL)
        return OEM_FSI_ERR_PARAMETER;

    *PosP = f_tell(FileListGetFIL(File));

    return OEM_FSI_SUCCESS;
}

OemValFsiResultT FileRemove(const char *NameP)
{
    FRESULT res;

    if (NameP == NULL)
        return OEM_FSI_ERR_PARAMETER;

    res = f_unlink(NameP);

    return FileOperationRet[res];
}

OemValFsiResultT FileRename(const char *OldNameP, const char *NewNameP)
{
    FRESULT res;

    if ((OldNameP == NULL) || (NewNameP == NULL))
        return OEM_FSI_ERR_PARAMETER;

    // FILE_OP_PRINT(DbgCtl.FileInfoEn, "\r\nOldNameP(%s) NewNameP(%s)", OldNameP, NewNameP);
    res = f_rename(OldNameP, NewNameP);

    return FileOperationRet[res];
}

OemValFsiResultT FileGetLength(const char *NameP, uint32_t *FileLengthP)
{
    if ((NameP == NULL) || (FileLengthP == NULL))
        return OEM_FSI_ERR_PARAMETER;

    *FileLengthP = f_size(FileListGetFILName(NameP));

    return OEM_FSI_SUCCESS;
}

// OemValFsiResultT FileVerify(char *NameP)
// {
//     char *ptmp = NameP;
//     char path[64] = {"0:/"};
//     FILINFO fno;
//     FRESULT res = FR_OK;

//     if ((NameP == NULL) || (strlen(NameP) == 0) || (strlen(NameP) > (sizeof(path) - 4)))
//         return OEM_FSI_ERR_PARAMETER;

//     while (1)
//     {
//         ptmp = strstr(ptmp, "/");
//         if (ptmp == NULL)
//         {
//             break;
//         }
//         else
//         {
//             ptmp++;
//             NameP = ptmp;
//         }
//     }
    
//     strcpy(path + 3, NameP);

//     res = f_stat(NameP, &fno);
//     if (res == FR_OK)
//     {
//         if (fno.fsize == 0)
//         {
//             return OEM_FSI_ERR_NOTEXIST;
//         }
//     }

//     return FileOperationRet[res];
// }

/*******************************************************************************
    Copyrights (C) Asiatelco Technologies Co., 2003-2017. All rights reserved
                                End Of The File
*******************************************************************************/
