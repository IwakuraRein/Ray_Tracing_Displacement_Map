/*
 * Copyright (c) 2021, NVIDIA CORPORATION.  All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION
 * SPDX-License-Identifier: Apache-2.0
 */

 //-------------------------------------------------------------------------------------------------
 // The Closest-Hit shader only returns the information of the hit. The shading will be done in 
 // the Ray-Generation shader or Ray-Query (compute)

#version 460
#extension GL_GOOGLE_include_directive : enable
#extension GL_EXT_ray_tracing : require  // This is about ray tracing

#include "globals.glsl"
layout(location = 0) rayPayloadInEXT PtPayload prd;
hitAttributeEXT vec2 bary;

void main()
{
  //prd.seed;
  prd.hitT                = gl_HitTEXT;
  prd.primitiveID         = gl_PrimitiveID;
  prd.instanceID          = gl_InstanceID;
  prd.instanceCustomIndex = gl_InstanceCustomIndexEXT;
  prd.baryCoord           = bary;
  prd.objectToWorld       = gl_ObjectToWorldEXT;
  prd.worldToObject       = gl_WorldToObjectEXT;
}
