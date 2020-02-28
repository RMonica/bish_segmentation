/*
 * Copyright (c) 2018, Riccardo Monica
 *   RIMLab, Department of Engineering and Architecture, University of Parma, Italy
 *   http://www.rimlab.ce.unipr.it/
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted
 * provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions
 * and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or other materials provided with
 * the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
 * FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <bish_segmentation/shaders/surfel_fragment_shader.h>

static const std::string FRAGMENT_SHADER = R"763d539980(

uniform vec4 intrinsics; //cx, cy, fx, fy
uniform float maxDepth;
uniform float minDepth;
uniform bool enable_lighting;

in vec4 vPositionIsFrontel;
in vec4 vNormalRadius;
in vec4 vColor;
in vec3 vGlobalPos;
in float vSqrPointSize;
flat in uint vIndex;

#if WITH_COLOR
layout(location = 0) out vec4 image;
#else // WITH_COLOR
layout(location = 0) out float image;
layout(location = 2) out uint out_index;
#endif // WITH_COLOR

#if WITH_DEPTH
layout(location = 1) out float out_depth;
#endif


#if WITH_SPHERE_FILTER
uniform vec4 sphere_filter_data;
#endif

#if WITH_BBOX_FILTER
uniform vec3 bbox_filter_data_min;
uniform vec3 bbox_filter_data_max;
#endif

void main()
{
    vec3 l = normalize(vec3((vec2(gl_FragCoord) - intrinsics.xy) / intrinsics.zw, 1.0f));

    if (dot(vNormalRadius.xyz,vNormalRadius.xyz) < (0.05 * 0.05))
      discard;
    if (abs(dot(l,normalize(vNormalRadius.xyz))) < 0.05)
      discard;

    vec3 corrected_pos = (dot(vPositionIsFrontel.xyz, vNormalRadius.xyz) / dot(l, vNormalRadius.xyz)) * l;

    float sqrRad = vNormalRadius.w;
    vec3 diff = corrected_pos - vPositionIsFrontel.xyz;

    if(dot(diff, diff) > sqrRad)  //cerchio
    {
        discard;
    }

    float isFrontel = vPositionIsFrontel.w;
    //float z = corrected_pos.z;        //corrected_pos FORSE DA TOGLIERE
    float z = vPositionIsFrontel.z;

    bool in_filter = true;
    if (z < minDepth)
      in_filter = false;

#if WITH_DEPTH
    if (in_filter)
      out_depth = z;
    else
      out_depth = 0.0;
#endif

    float value = 1.0;

    if (isFrontel > 0.5)
      value = -value; // unknown

#if WITH_COLOR
    float color_dim = (!enable_lighting) ? 1.0f :
                      (0.4f + 0.6f * dot(normalize(vNormalRadius.xyz),-normalize(vPositionIsFrontel.xyz)));
    image.xyz = vColor.xyz * color_dim;
    if (!in_filter)
      image.w = 0.0;
    else
      image.w = value;
#else // WITH_COLOR
    if (!in_filter)
      image = 0.0;
    else
      image = value;
    
    out_index = vIndex;
#endif // WITH_COLOR

    gl_FragDepth = (corrected_pos.z / (2 * maxDepth)) + 0.5f;
}

                                          )763d539980";

std::string GetFragmentShaderCode(const bool with_color,
                                  const bool with_depth)
{
  std::string result;
  result += "#version 330 core\n";

  result += "#define WITH_COLOR ";
  result += (with_color ? "1" : "0");
  result += "\n";

  result += "#define WITH_DEPTH ";
  result += (with_depth ? "1" : "0");
  result += "\n";

  result += FRAGMENT_SHADER;

  return result;
}
