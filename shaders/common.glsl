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
// Various common functions.


#ifndef RAYCOMMON_GLSL
#define RAYCOMMON_GLSL

#define HIT_KIND_TRIANGLE 0
#define HIT_KIND_DISPLACED_TRIANGLE 1
#define HIT_KIND_DISPLACED_CUBE 2


//-----------------------------------------------------------------------
// Debugging
//-----------------------------------------------------------------------
vec3 IntegerToColor(uint val)
{
  const vec3 freq = vec3(1.33333f, 2.33333f, 3.33333f);
  return vec3(sin(freq * val) * .5 + .5);
}

// utility for temperature
float fade(float low, float high, float value)
{
  float mid   = (low + high) * 0.5;
  float range = (high - low) * 0.5;
  float x     = 1.0 - clamp(abs(mid - value) / range, 0.0, 1.0);
  return smoothstep(0.0, 1.0, x);
}

// Return a cold-hot color based on intensity [0-1]
vec3 temperature(float intensity)
{
  const vec3 blue   = vec3(0.0, 0.0, 1.0);
  const vec3 cyan   = vec3(0.0, 1.0, 1.0);
  const vec3 green  = vec3(0.0, 1.0, 0.0);
  const vec3 yellow = vec3(1.0, 1.0, 0.0);
  const vec3 red    = vec3(1.0, 0.0, 0.0);

  vec3 color = (fade(-0.25, 0.25, intensity) * blue    //
                + fade(0.0, 0.5, intensity) * cyan     //
                + fade(0.25, 0.75, intensity) * green  //
                + fade(0.5, 1.0, intensity) * yellow   //
                + smoothstep(0.75, 1.0, intensity) * red);
  return color;
}

//-----------------------------------------------------------------------
// Return the UV in a lat-long HDR map
//-----------------------------------------------------------------------
vec2 GetSphericalUv(vec3 v)
{
  float gamma = asin(-v.y);
  float theta = atan(v.z, v.x);

  vec2 uv = vec2(theta * M_1_OVER_PI * 0.5, gamma * M_1_OVER_PI) + 0.5;
  return uv;
}


//-----------------------------------------------------------------------
// Return the tangent and binormal from the incoming normal
//-----------------------------------------------------------------------
void CreateCoordinateSystem(in vec3 N, out vec3 Nt, out vec3 Nb)
{
  // http://www.pbr-book.org/3ed-2018/Geometry_and_Transformations/Vectors.html#CoordinateSystemfromaVector
  //if(abs(N.x) > abs(N.y))
  //  Nt = vec3(-N.z, 0, N.x) / sqrt(N.x * N.x + N.z * N.z);
  //else
  //  Nt = vec3(0, N.z, -N.y) / sqrt(N.y * N.y + N.z * N.z);
  //Nb = cross(N, Nt);

  Nt = normalize(((abs(N.z) > 0.99999f) ? vec3(-N.x * N.y, 1.0f - N.y * N.y, -N.y * N.z) :
                                          vec3(-N.x * N.z, -N.y * N.z, 1.0f - N.z * N.z)));
  Nb = cross(Nt, N);
}


//-------------------------------------------------------------------------------------------------
// Avoiding self intersections (see Ray Tracing Gems, Ch. 6)
//-----------------------------------------------------------------------
vec3 OffsetRay(in vec3 p, in vec3 n)
{
  const float intScale   = 256.0f;
  const float floatScale = 1.0f / 65536.0f;
  const float origin     = 1.0f / 32.0f;

  ivec3 of_i = ivec3(intScale * n.x, intScale * n.y, intScale * n.z);

  vec3 p_i = vec3(intBitsToFloat(floatBitsToInt(p.x) + ((p.x < 0) ? -of_i.x : of_i.x)),
                  intBitsToFloat(floatBitsToInt(p.y) + ((p.y < 0) ? -of_i.y : of_i.y)),
                  intBitsToFloat(floatBitsToInt(p.z) + ((p.z < 0) ? -of_i.z : of_i.z)));

  return vec3(abs(p.x) < origin ? p.x + floatScale * n.x : p_i.x,  //
              abs(p.y) < origin ? p.y + floatScale * n.y : p_i.y,  //
              abs(p.z) < origin ? p.z + floatScale * n.z : p_i.z);
}

vec3 getBaryCoord(vec2 p, vec2 a, vec2 b, vec2 c)
{
  vec2 v0 = b - a, v1 = c - a, v2 = p - a;
  float denom = 1.0 / (v0.x * v1.y - v1.x * v0.y);
  vec3 outVec;
  outVec.y = (v2.x * v1.y - v1.x * v2.y) * denom;
  outVec.z = (v0.x * v2.y - v2.x * v0.y) * denom;
  outVec.x = 1.0 - outVec.y - outVec.z;
  return outVec;
}

vec3 rayPlaneIntersect(Ray ray, vec3 p0, vec3 p1, vec3 p2) {
  vec3 N = cross(p1-p0, p2-p0);
  vec3 X = ray.origin + ray.direction * dot(p0 - ray.origin, N) / dot(ray.direction, N);

  return X;
}

bool rayTrigIntersect(Ray r, vec3 vert0, vec3 vert1, vec3 vert2, out vec2 baryPosition, out float hitT)
{
  // find vectors for two edges sharing vert0
  vec3 edge1 = vert1 - vert0;
  vec3 edge2 = vert2 - vert0;

	// begin calculating determinant - also used to calculate U parameter
	vec3 p = cross(r.direction, edge2);

	// if determinant is near zero, ray lies in plane of triangle
	float det = dot(edge1, p);
  vec3 Perpendicular = vec3(0);

  if(det > EPS)
  {
    // calculate distance from vert0 to ray origin
    vec3 dist = r.origin - vert0;

    // calculate U parameter and test bounds
    baryPosition.x = dot(dist, p);
    if(baryPosition.x < 0 || baryPosition.x > det)
      return false;

    // prepare to test V parameter
    Perpendicular = cross(dist, edge1);

    // calculate V parameter and test bounds
    baryPosition.y = dot(r.direction, Perpendicular);
    if((baryPosition.y < 0) || ((baryPosition.x + baryPosition.y) > det))
      return false;
  }
  else if(det < -EPS)
  {
    // calculate distance from vert0 to ray origin
    vec3 dist = r.origin - vert0;

    // calculate U parameter and test bounds
    baryPosition.x = dot(dist, p);
    if((baryPosition.x > 0) || (baryPosition.x < det))
      return false;

    // prepare to test V parameter
    Perpendicular = cross(dist, edge1);

    // calculate V parameter and test bounds
    baryPosition.y = dot(r.direction, Perpendicular);
    if((baryPosition.y > 0) || (baryPosition.x + baryPosition.y < det))
      return false;
  }
  else
    return false; // ray is parallel to the plane of the triangle

  float inv_det = 1.0 / det;

  // calculate distance, ray intersects triangle
  hitT = dot(edge2, Perpendicular) * inv_det;
  baryPosition *= inv_det;

  return true;
}
bool rayAABBIntersect(vec3 Min, vec3 Max, Ray r, out float dist) {
  vec3 invDir = 1.0 / r.direction;
  float tmin = 0.0, tmax = INFINITY;
  float t1 = (Min.x - r.origin.x) * invDir.x;
  float t2 = (Max.x - r.origin.x) * invDir.x;

  tmin = max(tmin, min(t1, t2));
  tmax = min(tmax, max(t1, t2));

  t1 = (Min.y - r.origin.y) * invDir.y;
  t2 = (Max.y - r.origin.y) * invDir.y;

  tmin = max(tmin, min(t1, t2));
  tmax = min(tmax, max(t1, t2));

  t1 = (Min.z - r.origin.z) * invDir.z;
  t2 = (Max.z - r.origin.z) * invDir.z;

  tmin = max(tmin, min(t1, t2));
  tmax = min(tmax, max(t1, t2));

  dist = tmin;
  return tmax >= tmin;
}

#endif  // RAYCOMMON_GLSL
