/*
 * Copyright (c) 2023, DF Automation & Robotics Sdn. Bhd.
 * All rights reserved.
 *
 * Author: Patrick Chin
 */

import * as _ from 'lodash-es';
import * as THREE from 'three';

// custom shaders
var vertexShader = `
uniform sampler2D map;
uniform sampler2D map2;
uniform sampler2D map3;

uniform float width;
uniform float height;

uniform float cx;
uniform float cy;
uniform float fx;
uniform float fy;

uniform mat4 depthToColor;
uniform float color_cx;
uniform float color_cy;
uniform float color_fx;
uniform float color_fy;

uniform float pointSize;
uniform int mode;
uniform bool detecting;

varying vec4 color;

vec4 jet(float x) {
  float r = x < 0.7 ? 4.0 * x - 1.5 : 4.5 - 4.0 * x;
  float g = x < 0.5 ? 4.0 * x - 0.5 : 3.5 - 4.0 * x;
  float b = x < 0.3 ? 4.0 * x + 0.5 : 2.5 - 4.0 * x;
  r = clamp(r, 0.0, 1.0);
  g = clamp(g, 0.0, 1.0);
  b = clamp(b, 0.0, 1.0);
  return vec4(r, g, b, 1.0);
}

vec4 skyGroundSeg(float z) {
  if (z > 0.02) {
    return vec4(0.0, 0.3, 1.0, 1.0);
  } else if (z > 0.01) {
    return vec4(0.0, 0.65, 0.65, 1.0);
  } else if (z > -0.01) {
    return vec4(0.3, 1.0, 0.3, 1.0);
  } else if (z > -0.02) {
    return vec4(0.65, 0.5, 0.0, 1.0);
  }
  return vec4(0.8, 0.0, 0.1, 1.0);
}

void main() {
  vec2 uv = vec2((position.x + 0.5) / width, (position.y + 0.5) / height);
  vec4 p;
  if (mode == 5 && detecting) {
    p = texture(map3, uv);
    color = vec4(0.0, 0.3, 1.0, 1.0);
    if (p.r == 0.0 && p.a == 0.0) {
      p = texture(map2, uv);
      color = vec4(0.3, 1.0, 0.3, 1.0);
    }
    if (p.r == 0.0 && p.a == 0.0) {
      p = texture(map, uv);
      color = vec4(1.0, 1.0, 1.0, 1.0);
    }
  } else {
    p = texture(map, uv);
  }
  float depth = (p.r * 65536.0 + p.a * 256.0) * 0.001;

  vec4 pos = vec4(
    (position.x - cx) * depth / fx,
    (position.y - cy) * depth / fy,
    depth,
    1.0
  );

  vec4 worldPos = modelMatrix * pos;

  if (depth == 0.0) {
    color = vec4(0.0, 0.0, 0.0, 0.0);
  } else if (mode == 0) {
    float s = pos.z / 8.0;
    color = jet(s);
  } else if (mode == 1) {
    float s = worldPos.y / 8.0 + 0.5;
    color = jet(s);
  } else if (mode == 2) {
    float s = (worldPos.z + 0.5) / 2.0;
    color = jet(s);
  } else if (mode == 3) {
    color = skyGroundSeg(worldPos.z);
  } else if (mode == 4) {
    vec4 pos = depthToColor * pos;
    uv = vec2(
      pos.x * color_fx / pos.z + color_cx,
      pos.y * color_fy / pos.z + color_cy
    );
    if (uv.x < 0.0 || uv.x > 1.0 || uv.y < 0.0 || uv.y > 1.0) {
      color = vec4(1.0, 1.0, 1.0, 1.0);
    } else {
      color = texture(map2, uv);
      if (color.a == 0.0) {
        color = vec4(1.0, 1.0, 1.0, 1.0);
      }
    }
  } else if (mode == 5 && detecting) {
    // pass
  } else {
    color = vec4(1.0, 1.0, 1.0, 1.0);
  }

  gl_Position = projectionMatrix * viewMatrix * worldPos;
  gl_PointSize = pointSize;
}
`;

var fragmentShader = `
varying vec4 color;

void main() {
  if (color.a == 0.0) {
    discard;
  }
  gl_FragColor = color;
}
`;

export default function (scene) {
  var _mode = 0;
  var _detecting = false;
  var texture;
  var texture2;
  var texture3;
  var mat;
  var geom;
  var cloud;
  var _info;
  var _colorInfo;
  var _depthToColor;

  function setup(img, info) {
    // cleanup current resources
    if (cloud) {
      scene.remove(cloud);
      cloud = null;
    }
    if (geom) {
      geom.dispose();
      geom = null;
    }
    if (mat) {
      mat.dispose();
      mat = null;
    }
    if (texture3) {
      texture3.dispose();
      texture3 = null;
    }
    if (texture2) {
      texture2.dispose();
      texture2 = null;
    }
    if (texture) {
      texture.dispose();
      texture = null;
    }

    texture = new THREE.Texture(img);
    texture.magFilter = THREE.NearestFilter;
    texture.minFilter = THREE.NearestFilter;
    texture.generateMipmaps = false;
    texture.flipY = false;

    texture2 = new THREE.Texture();
    texture2.magFilter = THREE.NearestFilter;
    texture2.minFilter = THREE.NearestFilter;
    texture2.generateMipmaps = false;
    texture2.flipY = false;

    texture3 = new THREE.Texture();
    texture3.magFilter = THREE.NearestFilter;
    texture3.minFilter = THREE.NearestFilter;
    texture3.generateMipmaps = false;
    texture3.flipY = false;

    let colorInfo = _colorInfo || {
      width: 1.0,
      height: 1.0,
      cx: 0.0,
      cy: 0.0,
      fx: 0.0,
      fy: 0.0
    };
    let depthToColor = _depthToColor || new THREE.Matrix4();

    mat = new THREE.ShaderMaterial({
      uniforms: {
        map: {
          value: texture
        },
        map2: {
          value: texture2
        },
        map3: {
          value: texture3
        },
        width: {
          value: info.width
        },
        height: {
          value: info.height
        },
        cx: {
          value: info.cx
        },
        cy: {
          value: info.cy
        },
        fx: {
          value: info.fx
        },
        fy: {
          value: info.fy
        },
        depthToColor: {
          value: depthToColor
        },
        color_cx: {
          value: (colorInfo.cx + 0.5) / colorInfo.width
        },
        color_cy: {
          value: (colorInfo.cy + 0.5) / colorInfo.height
        },
        color_fx: {
          value: colorInfo.fx / colorInfo.width
        },
        color_fy: {
          value: colorInfo.fy / colorInfo.height
        },
        pointSize: {
          value: 1.5
        },
        mode: {
          value: _mode
        },
        detecting: {
          value: _detecting
        }
      },
      vertexShader: vertexShader,
      fragmentShader: fragmentShader
    });

    var vertices = new Float32Array(info.width * info.height * 3);
    for (var y = 0, idx = -1; y < info.height; ++y) {
      for (var x = 0; x < info.width; ++x, ++idx) {
        vertices[++idx] = x;
        vertices[++idx] = y;
      }
    }
    geom = new THREE.BufferGeometry();
    geom.setAttribute('position', new THREE.BufferAttribute(new Float32Array(vertices), 3));

    cloud = new THREE.Points(geom, mat);
    scene.add(cloud);
    _info = info;
  }

  function updateDepthImage(img, info, pose) {
    if (!_.isEqual(_info, info)) {
      setup(img, info);
    }
    texture.needsUpdate = true;
    cloud.position.set(pose.p[0], pose.p[1], pose.p[2]);
    cloud.quaternion.set(pose.q[0], pose.q[1], pose.q[2], pose.q[3]);
  }

  function update2(img, info, pose) {
    if (mat) {
      let infoChanged = info && !_.isEqual(_colorInfo, info);
      if (texture2) {
        if (infoChanged) {
          texture2.dispose();
          texture2 = new THREE.Texture(img);
          texture2.magFilter = THREE.NearestFilter;
          texture2.minFilter = THREE.NearestFilter;
          texture2.generateMipmaps = false;
          texture2.flipY = false;
          mat.uniforms.map2.value = texture2;
        }
        texture2.image = img;
        texture2.needsUpdate = true;
      }

      if (infoChanged) {
        mat.uniforms.color_cx.value = (info.cx + 0.5) / info.width;
        mat.uniforms.color_cy.value = (info.cy + 0.5) / info.height;
        mat.uniforms.color_fx.value = info.fx / info.width;
        mat.uniforms.color_fy.value = info.fy / info.height;

        if (pose) {
          var position = new THREE.Vector3(pose.p[0], pose.p[1], pose.p[2]);
          var quaternion = new THREE.Quaternion(pose.q[0], pose.q[1], pose.q[2], pose.q[3]);
          var scale = new THREE.Vector3(1, 1, 1);
          var invColor = new THREE.Matrix4();
          invColor.compose(position, quaternion, scale);
          invColor.invert();

          var matrix = new THREE.Matrix4();
          matrix.compose(cloud.position, cloud.quaternion, cloud.scale);
          matrix.premultiply(invColor);

          mat.uniforms.depthToColor.value = matrix;
          _depthToColor = matrix;
        }
      }
    }
    _colorInfo = info;
  }

  function update3(img) {
    if (texture3) {
      texture3.image = img;
      texture3.needsUpdate = true;
    }
  }

  function updateColorMode(mode) {
    _mode = mode;
    if (mat) {
      mat.uniforms.mode.value = mode;

      if (texture2) {
        texture2.dispose();
        texture2 = new THREE.Texture();
        texture2.magFilter = THREE.NearestFilter;
        texture2.minFilter = THREE.NearestFilter;
        texture2.generateMipmaps = false;
        texture2.flipY = false;
        mat.uniforms.map2.value = texture2;
      }

      if (texture3) {
        texture3.dispose();
        texture3 = new THREE.Texture();
        texture3.magFilter = THREE.NearestFilter;
        texture3.minFilter = THREE.NearestFilter;
        texture3.generateMipmaps = false;
        texture3.flipY = false;
        mat.uniforms.map3.value = texture3;
      }
    }
  }

  function updateDetecting(detecting) {
    _detecting = detecting;
    if (mat) {
      mat.uniforms.detecting.value = detecting;
    }
  }

  return {
    update: updateDepthImage,
    update2,
    update3,
    updateColorMode,
    updateDetecting
  };
}
