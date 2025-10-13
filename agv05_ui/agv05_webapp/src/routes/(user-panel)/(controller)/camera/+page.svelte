<script>
  import { cameraChannel } from 'stores/sock/index.js';
  import { onDestroy, onMount } from 'svelte';
  import base64 from 'base64-js';
  import threeViz from '$lib/shared/three/viz';
  import cashDom from 'cash-dom';
  import * as THREE from 'three';

  import MarkerType from './MarkerType.svelte';

  var scope_fps = -1;
  var topics = [];
  var topic = '';
  var colorTopic = '';
  var flip = false;
  var rpy = null;
  var applyRotation = false;
  var depth = false;
  var depthColorMode = 0;
  var imageUrl = '';
  var imageUrl2 = '';
  var imageUrl3 = '';
  var sensor = '';
  var imgDom;
  var imgDom2;
  var imgDom3;

  const depthColorModeFullOptions = [
    [4, 'RGB'],
    [0, 'X-axis'],
    [1, 'Y-axis'],
    [2, 'Z-axis'],
    [3, 'Calibration'],
    [5, 'Pallet Detection']
  ];
  const depthColorModeBasicOptions = depthColorModeFullOptions.slice(1, 5);

  $: depthColorModeOptions = topic.startsWith('/camera')
    ? depthColorModeFullOptions
    : depthColorModeBasicOptions;
  $: updateDepthColorMode(depthColorMode);

  var _infoTopics = {}; // topic to infoTopic mapping
  function reload() {
    clear();
    fetch('/stream.html')
      .then(function (response) {
        return response.text();
      })
      .then(function (response) {
        var str = response;
        var _topics = [];
        var infoTopics = {};

        var infoTopic;
        var topic;
        var start;
        var end = 0;
        var groupEnd;

        // with camera info
        while ((start = str.indexOf('<li>/', end)) > 0) {
          end = str.indexOf('<ul>', start);
          infoTopic = str.substring(start + 4, end);
          groupEnd = str.indexOf('</ul>', end);

          while ((start = str.indexOf('<a href="/stream_viewer?topic=', end)) > 0) {
            if (start > groupEnd) {
              break;
            }
            end = str.indexOf('">', start);
            topic = str.substring(start + 30, end);
            _topics.push(topic);
            infoTopics[topic] = infoTopic;
          }
          end = groupEnd;
        }

        // without camera info
        while ((start = str.indexOf('<a href="/stream_viewer?topic=', end)) > 0) {
          end = str.indexOf('">', start);
          _topics.push(str.substring(start + 30, end));
        }

        topics = _topics.sort();
        _infoTopics = infoTopics;
      });
  }

  var _depth;
  function stream(_topic) {
    imageUrl = getStreamUrl(_topic);
    topic = _topic;

    var infoTopic = _infoTopics[_topic];
    _depth = infoTopic && _topic.indexOf('depth') >= 0;
    if (infoTopic) {
      fps += 1; // throttle next publish
      cameraChannel.publish({
        id: 'info',
        topic: infoTopic
      });
    }
    if (_depth) {
      let m = topic.match(/^\/(camera[^\/]*)\//);
      if (m) {
        sensor = m[1];
      }
    }
  }

  function getStreamUrl(_topic) {
    return `/stream?topic=${_topic}&type=ros_compressed&t=${Date.now()}`;
  }

  function clear() {
    imageUrl = '';
    imageUrl2 = '';
    imageUrl3 = '';
    sensor = '';
    topics = [];
    topic = '';
    colorTopic = '';
    flip = false;
    rpy = null;
    applyRotation = false;
    depth = false;
    depthColorMode = 0;
    _infoTopics = {};
    _depth = false;
  }

  /* WebGL canvas */
  var canvas;
  var img;
  var viz;

  var isFirefox = typeof InstallTrigger !== 'undefined';
  var _update;

  function vizInit() {
    canvas = cashDom('#live-camera-view');
    img = canvas.siblings('img').attr('crossOrigin', 'anonymous');
    img.on('load', function () {
      if (this === img[0]) {
        update();
      } else if (this === img[1]) {
        update2();
      } else if (this === img[2]) {
        if (_imgReady(img[2])) viz.update3(img[2]);
      }
    });
    if (!isFirefox) {
      _update = setInterval(update, 60);
    }
    viz = threeViz(canvas[0]);
  }

  function update() {
    fps += 1;
    flip = false;
    rpy = null;
    depth = depthColorMode === 4; // in RGB mode, image occasionally not ready when synchronizing
    if (!viz) {
      return;
    }
    if (!_imgReady(img[0])) {
      return;
    }

    var infoTopic = _infoTopics[topic];
    if (!infoTopic) {
      return;
    }

    var width = img[0].naturalWidth;
    var height = img[0].naturalHeight;

    if (
      !_info ||
      !_pose ||
      _info.topic !== infoTopic ||
      _info.width !== width ||
      _info.height !== height ||
      !_pose[_info.frame_id]
    ) {
      // throttle
      if (fps === 1) {
        cameraChannel.publish({
          id: 'info',
          topic: infoTopic
        });
      }
      return;
    }

    if (!_depth) {
      let q = _pose[_info.frame_id].q;

      // rotate Y vector & get Z component
      if (q[1] * q[2] + q[3] * q[0] > 0) {
        flip = true;
      }

      // find relative rotation from the normal camera frame: [-0.5, 0.5, -0.5, 0.5]
      q = new THREE.Quaternion(q[0], q[1], q[2], q[3]);
      q.premultiply(new THREE.Quaternion(0.5, -0.5, 0.5, 0.5));
      let euler = new THREE.Euler();
      euler.setFromQuaternion(q, 'YXZ');
      rpy = [
        (-euler.x * 180.0) / Math.PI, // left-handed coordinate on screen
        0, // remove pitch (yaw in robot frame)
        (euler.z * 180.0) / Math.PI
      ];
      return;
    }

    viz.update(img[0], _info, _pose[_info.frame_id]);
    if (!isFirefox) {
      update2();
      if (_imgReady(img[2])) viz.update3(img[2]);
    }
    depth = true;
  }

  function update2() {
    fps2 += 1;
    if (!_imgReady(img[1])) {
      return;
    }
    if (depthColorMode === 4) {
      var infoTopic = _infoTopics[colorTopic];
      var width = img[1].naturalWidth;
      var height = img[1].naturalHeight;

      if (
        !_colorInfo ||
        !_pose ||
        _colorInfo.topic !== infoTopic ||
        _colorInfo.width !== width ||
        _colorInfo.height !== height ||
        !_pose[_colorInfo.frame_id]
      ) {
        // throttle
        if (fps2 === 1) {
          cameraChannel.publish({
            id: 'info',
            topic: infoTopic
          });
        }
        return;
      }
      viz.update2(img[1], _colorInfo, _pose[_colorInfo.frame_id]);
    } else {
      viz.update2(img[1]);
    }
  }

  function _imgReady(img) {
    return img.complete && img.naturalWidth && img.naturalHeight;
  }

  function updateDepthColorMode() {
    if (!viz) return;
    imageUrl2 = '';
    imageUrl3 = '';

    if (depthColorMode === 4) {
      if (sensor) {
        let _topic = `/${sensor}/color/image_raw`;
        let infoTopic = _infoTopics[_topic];
        if (infoTopic) {
          imageUrl2 = getStreamUrl(_topic);
          colorTopic = _topic;
          cameraChannel.publish({
            id: 'info',
            topic: infoTopic
          });
        } else {
          depthColorMode = 0;
        }
      } else {
        depthColorMode = 0;
      }
    } else if (depthColorMode === 5) {
      if (sensor) {
        if (detecting) {
          imageUrl2 = getStreamUrl('/depth_voxel');
          imageUrl3 = getStreamUrl('/depth_vertical');
        }
      } else {
        depthColorMode = 0;
      }
    }
    viz.updateColorMode(depthColorMode);
  }

  function updateDetecting() {
    if (viz) {
      if (detecting) {
        imageUrl2 = getStreamUrl('/depth_voxel');
        imageUrl3 = getStreamUrl('/depth_vertical');
      } else if (depthColorMode === 5) {
        imageUrl2 = '';
        imageUrl3 = '';
      }
      viz.updateDetecting(detecting);
    }
  }

  var wantResize = 0;
  function vizResize(internal) {
    var p = canvas.parent().parent();
    var s = p.children().css('display', 'none');

    var rc = p[0].getBoundingClientRect();
    s.css('display', null);

    viz.resize(rc.width, rc.height);

    if (!internal) {
      wantResize = 2; // schedule vizResize after resize animation ends
    }
  }

  /* measure fps */
  var fps = 0;
  var fps2 = 0;
  var _fps;

  function updateFps() {
    scope_fps = isFirefox ? fps : -1;
    fps = 0;
    fps2 = 0;

    if (wantResize > 0) {
      --wantResize;
      vizResize(true);
    }

    // synchronize depth and color images
    if (depthColorMode === 4) {
      if (topic) imageUrl = getStreamUrl(topic);
      if (colorTopic) imageUrl2 = getStreamUrl(colorTopic);
    }
  }

  /* callback */
  var _info;
  var _colorInfo;
  var _pose;
  var markerType = '';

  function cameraCallback(data) {
    if (data.id === 'info') {
      var infoTopic = _infoTopics[topic];
      if (infoTopic === data.topic) {
        _info = data.info;
      } else {
        infoTopic = _infoTopics[colorTopic];
        if (infoTopic === data.topic) {
          _colorInfo = data.info;
        }
      }
    } else if (data.id === 'pose') {
      _pose = data.pose;
    } else if (data.id === 'marker_type') {
      markerType = data.marker_type;
    } else if (data.id === 'cloud_flatten') {
      data = data.cloud_flatten;
      viz.updateCloudFlatten(
        base64.toByteArray(data.cloud).buffer,
        detecting && !!data.frame_id
      );
    } else if (data.id === 'cloud_colored') {
      data = data.cloud_colored;
      viz.updateCloudColored(
        base64.toByteArray(data.cloud).buffer,
        detecting && !!data.frame_id
      );
    }
  }

  $: detecting =
    depthColorMode === 5 &&
    sensor &&
    !!markerType.match(new RegExp(`^pallet[0-9]?__${sensor}($|_)`));
  $: updateDetecting(detecting);

  onMount(() => {
    vizInit();
    const resizeListener = () => vizResize();
    window.addEventListener('resize', resizeListener);
    //   $log.debug($scope.controllerName, '$ionicView.enter');
    vizResize();
    reload();
    _fps = setInterval(updateFps, 1000);
    cameraChannel.subscribe(cameraCallback);

    return () => {
      window.removeEventListener('resize', resizeListener);
    };
  });

  onDestroy(() => {
    clear();
    cameraChannel.unsubscribe(cameraCallback);
    if (viz) {
      viz.stop();
      viz.dispose();
    }
    clearInterval(_fps);
    clearInterval(_update);

    // Note: onDestroy might not rerender img tag when navigate away.
    imgDom.src = '';
    imgDom2.src = '';
    imgDom3.src = '';
  });
  //   $log.debug($scope.controllerName, 'initialized.');
</script>

<div class="flex h-full flex-col">
  <div class="mb-2">
    <strong>Camera:</strong>
    <button type="button" class="variant-filled-tertiary btn btn-sm" on:click={reload}>
      {#if topic}
        <i class="fa-solid fa-chevron-left mr-1"></i> Show List
      {:else}
        <i class="fa-solid fa-rotate-right mr-1"></i> Refresh List
      {/if}
    </button>
    {#if topic}
      <span class="variant-ghost chip">{topic}</span>
      {#if scope_fps >= 0}
        <span class="variant-ghost chip">FPS: {scope_fps}</span>
      {/if}
      {#if depth}
        <span class="text-sm font-bold">Color Mode:</span>
        <select class="chip text-left" bind:value={depthColorMode}>
          {#each depthColorModeOptions as [value, label]}
            <option {value}>{label}</option>
          {/each}
        </select>
        {#if depthColorMode === 5}
          <MarkerType
            {sensor}
            {markerType}
            {detecting}
            publish={(m) =>
              cameraChannel.publish({
                id: 'marker_type',
                marker_type: m
              })} />
        {/if}
      {:else if rpy}
        <span
          class="chip {applyRotation ? 'variant-filled-warning' : 'variant-ghost'}"
          on:click={() => {
            applyRotation = !applyRotation;
          }}>
          Rotate Image
        </span>
      {/if}
    {/if}
  </div>
  <div class="flex h-full flex-col">
    <div class="card divide-y divide-solid divide-black px-2" class:hidden={topic}>
      {#each topics as x}
        <div class="py-1">
          <!-- svelte-ignore a11y-click-events-have-key-events -->
          <!-- svelte-ignore a11y-no-static-element-interactions -->
          <div
            class="rounded-xl p-2 hover:shadow-lg"
            style="cursor: pointer;"
            on:click={() => stream(x)}>
            {x}
            <i class="fa-solid fa-play float-right"></i>
          </div>
        </div>
      {/each}
    </div>
    <div
      class="flex h-full items-center justify-center overflow-hidden"
      class:hidden={!topic}
      class:p-[10%]={rpy && applyRotation}
      style="perspective:800px">
      <canvas id="live-camera-view" class:hidden={!depth} />
      <img
        bind:this={imgDom}
        src={imageUrl}
        class="outline-solid max-h-full max-w-full bg-black outline-1 outline-gray-300"
        class:hidden={depth}
        class:rotate-180={flip && !applyRotation}
        style={rpy && applyRotation
          ? `transform: rotateX(${rpy[0]}deg) rotateZ(${rpy[2]}deg)`
          : ''}
        alt="loading..." />
      <img bind:this={imgDom2} src={imageUrl2} class="hidden" alt="hidden" />
      <img bind:this={imgDom3} src={imageUrl3} class="hidden" alt="hidden" />
    </div>
  </div>
</div>
