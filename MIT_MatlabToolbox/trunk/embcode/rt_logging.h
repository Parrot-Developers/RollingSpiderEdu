/* Copyright 2012-2014 The MathWorks, Inc. */

#ifndef rt_logging_h
#define rt_logging_h

#if defined(_MSC_VER)
# pragma once
#endif
#if defined(__GNUC__) && (__GNUC__ > 3 || (__GNUC__ == 3 && __GNUC_MINOR__ > 3))
# pragma once
#endif

#if !defined(MAT_FILE) || (defined(MAT_FILE) && MAT_FILE == 1)

#include <stddef.h>                     /* size_t */
#include "rtwtypes.h"
#include "builtin_typeid_types.h"
#include "multiword_types.h"
#include "rt_mxclassid.h"
#include "rtw_matlogging.h"

#ifndef TMW_NAME_LENGTH_MAX
#define TMW_NAME_LENGTH_MAX 64
#endif
#define mxMAXNAM  TMW_NAME_LENGTH_MAX	/* maximum name length */

/*=========*
 * Defines *
 *=========*/

/*
 * Logging related functions and data structures
 */
typedef double MatReal;                /* "real" data type used in model.mat  */
typedef struct LogVar_Tag LogVar;
typedef struct StructLogVar_Tag StructLogVar;

typedef struct MatrixData_Tag {
  char_T         name[mxMAXNAM];     /* Name of the variable                  */
  int_T          nRows;              /* number of rows                        */
  int_T          nCols;              /* number of columns                     */
  int_T          nDims;              /* number of dimensions                  */
  int_T          _dims[2];           /* most cases, dimensions are 2          */
  int_T          *dims;              /* dimensions of the log variable we 
                                        write to at each simulation time step.
                                        E.g: (1) Non-frame data - 
                                                 Signal dimension = [2 X 3]
                                                 numDims = 2
                                                 dims[0] = 2, dims[1] = 3
                                             (2) Frame data - 
                                                 Signal dimension = [2 X 3]
                                                 numDims = 1
                                                 dims[0] = 3                  */
  void           *re;                /* pointer to real part of the data      */
  void           *im;                /* pointer to imaginary part, if complex */
  DTypeId        dTypeID;            /* data type id                          */
  size_t         elSize;             /* element size in bytes                 */

  RTWLogDataTypeConvert dataTypeConvertInfo;

  mxClassID      mxID;               /* mxId corresponding to this dTypeID    */
  uint32_T       logical;            /* is this a logical array ?             */
  uint32_T       complex;            /* is this a complex matrix?             */
  uint32_T       frameData;          /* is this data frame based?             */
  uint32_T       frameSize;          /* is this data frame based?             */
} MatrixData;

typedef struct ValDimsData_Tag {
  char_T         name[mxMAXNAM];     /* Name of the variable                  */
  int_T          nRows;              /* number of rows                        */
  int_T          nCols;              /* number of columns                     */
  void           **currSigDims;      /* dimensions of current output          */
  int_T          *currSigDimsSize;   /* size in bytes of current sig dims     */
  real_T         *dimsData;          /* pointer to the value of dimension     */
} ValDimsData;

struct LogVar_Tag {
    MatrixData  data;                 /* Container for name, data etc.,       */
    ValDimsData *valDims;             /* field of valueDimensions
                                       1. If all logging signals are fixed-size,
                                          then we set this field to NULL;
                                       2. If any logging signal is variable-size,
                                          then this field will be needed:
                                         1) For fixed-size signal, this field is
                                            an empty matrix;
                                         2) Otherwise, it contains the dimension
                                            information of the logging signal.
                                      */
    int_T      rowIdx;                /* current row index                    */
    int_T      wrapped;               /* number of times the circular buffer
                                       * has wrapped around                   */
    int_T     nDataPoints;            /* total number of data points logged   */
    int_T     usingDefaultBufSize;    /* used to print a message at end       */
    int_T     okayToRealloc;          /* reallocate during sim?               */
    int_T     decimation;             /* decimation factor                    */
    int_T     numHits;                /* decimation hit count                 */

    int_T     *coords;
    int_T     *strides;
    int_T     *currStrides;           /* coords, strides and currStrides will be
                                         needed when logging variable-size 
                                         signal to calculate whether the 
                                         currently logging value is in the range.
                                         If the current signal is fixed-size,
                                         these pointers will be set to NULLs;
                                         otherwise, we allocate memory for them.
                                         (the size will be nDims in this case)
                                      */

    LogVar    *next;
};

typedef struct SignalsStruct_Tag {
    int_T        numActiveFields;   /* number of active fields                */
    const char_T *fieldNames;
    int_T        numSignals;
    LogVar       *values;
    MatrixData   *dimensions;
    MatrixData   *labels;
    MatrixData   *plotStyles;
    MatrixData   *titles;
    MatrixData   *blockNames;
    MatrixData   *stateNames;
    MatrixData   *crossMdlRef;

    boolean_T    logValueDimensions; /* If there's any variable-size signal 
                                         we also should log 'valueDimensions'
                                         field                                 */
    boolean_T    *isVarDims;         /* is this signal a variable-size signal? */
} SignalsStruct;

struct StructLogVar_Tag {
    char_T        name[mxMAXNAM];    /* Name of the ML Struct variable         */
    int_T         numActiveFields;   /* number of active fields                */
    boolean_T     logTime;
    void          *time;
    SignalsStruct signals;
    MatrixData    *blockName;

    StructLogVar  *next;
};

#define matUNKNOWN                  0
#define	matINT8                     1
#define	matUINT8                    2
#define	matINT16                    3
#define	matUINT16                   4
#define	matINT32                    5
#define	matUINT32                   6
#define	matFLOAT                    7
#define	matDOUBLE                   9
#define matINT64                   12
#define matUINT64                  13
#define	matMATRIX                  14

/* status of logging "valueDimensions" field */
/* 
  NO_LOGVALDIMS: 
                 No need to log valueDimensions: 
                 All signals are fixed-sized.

  LOGVALDIMS_EMPTYMX: 
                 Signals with mixed dimension modes,
                 and the signal logged currently
                 is fixed-sized. So set valueDimensions
                 field to an empty matrix.

  LOGVALDIMS_VARDIMS:
                 Signal logged currently is variable-sized.
*/
typedef enum {
    NO_LOGVALDIMS,      
    LOGVALDIMS_EMPTYMX,
    LOGVALDIMS_VARDIMS
} LogValDimsStat;



#ifdef __cplusplus
extern "C" {
#endif

extern mxClassID rt_GetMxIdFromDTypeIdForRSim(BuiltInDTypeId dTypeID);

extern mxClassID rt_GetMxIdFromDTypeId(BuiltInDTypeId dTypeID);

extern LogVar *rt_CreateLogVarWithConvert(
    RTWLogInfo        *li,
    const real_T      startTime,
    const real_T      finalTime,
    const real_T      inStepSize,
    const char_T      **errStatus,
    const char_T      *varName,
    BuiltInDTypeId    inpDataTypeID,
    const RTWLogDataTypeConvert *pDataTypeConvertInfo,
    int_T             logical,
    int_T             complex,
    int_T             frameData,
    int_T             nCols,
    int_T             nDims,
    const int_T       *dims,
    LogValDimsStat    logValDimsStat,
    void              **currSigDims,
    int_T             *currSigDimsSize,
    int_T             maxRows,
    int_T             decimation,
    real_T            sampleTime,
    int_T             appendToLogVarsList);


extern LogVar *rt_CreateLogVar(RTWLogInfo        *li,
                               const real_T      startTime,
                               const real_T      finalTime,
                               const real_T      inStepSize,
                               const char_T      **errStatus,
                               const char_T      *varName,
                               BuiltInDTypeId    inpDataTypeID,
                               int_T             logical,
                               int_T             complex,
                               int_T             frameData,
                               int_T             nCols,
                               int_T             nDims,
                               const int_T       *dims,
                               LogValDimsStat    logValDimsStat,
                               void              **currSigDims,
                               int_T             *currSigDimsSize,
                               int_T             maxRows,
                               int_T             decimation,
                               real_T            sampleTime,
                               int_T             appendToLogVarsList);

extern StructLogVar *rt_CreateStructLogVar(RTWLogInfo              *li,
                                           const real_T            startTime,
                                           const real_T            finalTime,
                                           const real_T            inStepSize,
                                           const char_T            **errStatus,
                                           const char_T            *varName,
                                           boolean_T               logTime,
                                           int_T                   maxRows,
                                           int_T                   decimation,
                                           real_T                  sampleTime,
                                           const RTWLogSignalInfo  *sigInfo,
                                           const char_T            *blockName);


extern const char_T *rt_StartDataLoggingWithStartTime(RTWLogInfo   *li,
                                                      const real_T startTime,
                                                      const real_T finalTime,
                                                      const real_T stepSize,
                                                      const char_T **errStatus);

extern const char_T *rt_StartDataLogging(RTWLogInfo   *li,
                                         const real_T finalTime,
                                         const real_T stepSize,
                                         const char_T **errStatus);

extern void rt_UpdateLogVar(LogVar *var, const void *data, boolean_T isVarDims);

extern void rt_UpdateStructLogVar(StructLogVar *var, const real_T *t, const void *data);

extern const char_T *rt_UpdateTXYLogVars(RTWLogInfo *li, time_T *tPtr);

extern void rt_StopDataLoggingImpl(const char_T *file, RTWLogInfo *li, boolean_T isRaccel);

extern void rt_StopDataLogging(const char_T *file, RTWLogInfo *li);


#ifdef __cplusplus
}
#endif


#else /*!defined(MAT_FILE) || (defined(MAT_FILE) && MAT_FILE == 1)*/

#define rt_StartDataLogging(li, finalTime, stepSize, errStatus) NULL /* do nothing */
#define rt_UpdateTXYLogVars(li, tPtr) NULL /* do nothing */
#define rt_StopDataLogging(file, li); /* do nothing */

#endif /*!defined(MAT_FILE) || (defined(MAT_FILE) && MAT_FILE == 1)*/

#endif /* rt_logging_h */

/* LocalWords:  curr LOGVALDIMS EMPTYMX VARDIMS
 */
