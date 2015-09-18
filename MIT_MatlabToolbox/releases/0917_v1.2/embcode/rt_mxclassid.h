/* Copyright 1994-2004 The MathWorks, Inc.
 *
 * File    : rt_mxclassid.h
 * Abstract:
 *   Definition of enum mxClassID
 *
 */


#ifndef __rt_mxclassid_h__
#define __rt_mxclassid_h__

typedef enum {
	mxUNKNOWN_CLASS = 0,
	mxCELL_CLASS = 1,
	mxSTRUCT_CLASS,
	mxLOGICAL_CLASS,
	mxCHAR_CLASS,
	mxRESERVED5_CLASS, /* mxVOID_CLASS */
	mxDOUBLE_CLASS,
	mxSINGLE_CLASS,
	mxINT8_CLASS,
	mxUINT8_CLASS,
	mxINT16_CLASS,
	mxUINT16_CLASS,
	mxINT32_CLASS,
	mxUINT32_CLASS,
	mxINT64_CLASS,		/* place holder - future enhancements */
	mxUINT64_CLASS,		/* place holder - future enhancements */
	mxRESERVED16_CLASS, /* mxFUNCTION_CLASS */
	mxRESERVED17_CLASS, /* mxOPAQUE_CLASS */
	mxOBJECT_CLASS
} mxClassID;

#endif /* __rt_mxclassid_h__ */

/* [EOF] rt_mxclass_id.h */
