//------------------------------------------------------------------------------
// <copyright file="dset_internal.h" company="Atheros">
//    Copyright (c) 2004-2007 Atheros Corporation.  All rights reserved.
// 
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License version 2 as
// published by the Free Software Foundation;
//
// Software distributed under the License is distributed on an "AS
// IS" basis, WITHOUT WARRANTY OF ANY KIND, either express or
// implied. See the License for the specific language governing
// rights and limitations under the License.
//
//
//------------------------------------------------------------------------------
//==============================================================================
// Author(s): ="Atheros"
//==============================================================================


#ifndef __DSET_INTERNAL_H__
#define __DSET_INTERNAL_H__

/*
 * Internal dset definitions, common for DataSet layer.
 */

#define DSET_TYPE_STANDARD      0
#define DSET_TYPE_BPATCHED      1
#define DSET_TYPE_COMPRESSED    2

/* Dataset descriptor */

typedef struct dset_descriptor_s {
  struct dset_descriptor_s  *next;         /* List link. NULL only at the last
                                              descriptor */
  A_UINT16                   id;           /* Dset ID */
  A_UINT16                   size;         /* Dset size. */
  void                      *DataPtr;      /* Pointer to raw data for standard
                                              DataSet or pointer to original
                                              dset_descriptor for patched
                                              DataSet */
  A_UINT32                   data_type;    /* DSET_TYPE_*, above */

  void                      *AuxPtr;       /* Additional data that might
                                              needed for data_type. For
                                              example, pointer to patch
                                              Dataset descriptor for BPatch. */
} dset_descriptor_t;

#endif /* __DSET_INTERNAL_H__ */
