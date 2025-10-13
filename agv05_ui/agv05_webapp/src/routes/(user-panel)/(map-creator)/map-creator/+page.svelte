<script>
  import base64 from 'base64-js';
  import cashDom from 'cash-dom';
  import { onDestroy, onMount, getContext } from 'svelte';
  import { ProgressRadial } from '@skeletonlabs/skeleton';
  import { SlideToggle } from '@skeletonlabs/skeleton';
  import { get } from 'svelte/store';

  import { mapXChannel as mapChannel } from 'stores/sock/index.js';
  import mapViz from '$lib/shared/mapx/viz';
  import VirtualJoystick from 'components/VirtualJoystick.svelte';

  const navKey = [
    'KeyW',
    'KeyA',
    'KeyS',
    'KeyD',
    'ArrowUp',
    'ArrowLeft',
    'ArrowDown',
    'ArrowRight',
    'Space'
  ];

  const user = getContext('user');
  const mainController = getContext('mainController');
  let moduleManager = mainController.moduleService.getManager();
  let mapCreator = mainController.moduleService.getSub('map-creator');

  var element;
  var viz;
  var destroyed = false;
  var started = false;
  var screenRotate = true;
  var robotRadius = 1;
  var speed = {
    data: 0.5,
    noos: false
  };
  var enableManualControl = true;
  var enableVirtualKeypad = false;
  var enableLateralControl = false;
  var status;
  var mapChannelStatus;
  var mapCreatorStatus;
  var lastSaved = null;

  function moduleManagerCallback(data) {
    if (data.command === 'active_module') {
      if (data.module_id === 'map-creator') {
        started = true;
        mapCreatorStarted();
      } else {
        started = false;
        mapCreatorStopped();
      }
    }
  }

  function mapCreatorStarted() {
    mapCreator.publish({
      command: 'status'
    });
    mapChannel.publish({
      id: 'retrieve_all'
    });
  }

  function mapCreatorStopped() {}

  $: startMappingDisabled = status !== 'idle';

  function startMapping() {
    if (startMappingDisabled) {
      return;
    }
    mapCreator.publish({
      command: 'start_mapping'
    });
    mapChannel.publish({
      id: 'start_mapping'
    });
  }

  $: saveMapDisabled = status !== 'mapping';
  function saveMap() {
    if (saveMapDisabled) {
      return;
    }
    lastSaved = 0; // display saving in progress.
    mapChannel.publish({
      id: 'save_map'
    });
  }

  $: stopMappingDisabled = status !== 'mapping';

  function stopMapping() {
    if (stopMappingDisabled) {
      return;
    }
    lastSaved = 0; // display saving in progress.
    mapChannel.publish({
      id: 'stop_mapping'
    });
    mapCreator.publish({
      command: 'stop_mapping'
    });
  }

  let isOmniDrive = false;
  let hasHumanFollower = false;

  function mapCreatorCallback(data) {
    if (data.command === 'status') {
      mapCreatorStatus = data.status;
      if (mapCreatorStatus === mapChannelStatus) {
        status = data.status;
      } else {
        status = 'idle';
      }
      if (status === 'idle') {
        // clear map and reset robot pose
        viz.scene.clearRawMap();
        viz.scene.updateRobotPose({
          x: 0,
          y: 0,
          theta: 0
        });
        viz.zoom.zoomFit();
        lastSaved = null;
      }
      isOmniDrive = data.is_omni_drive;
    } else if (data.command === 'human_follower') {
      hasHumanFollower = !!(data.ready || data.running);
      enableHumanFollower = !!data.running;
    }
  }

  function mapCallback(data) {
    if (data.id === 'laser_pose') {
      viz.scene.updateLaserPose(data.laser_pose);
    } else if (data.id === 'laser_scan') {
      data.laser_scan.ranges = new Float32Array(
        base64.toByteArray(data.laser_scan.ranges).buffer
      );
      if (data.laser_scan.intensities) {
        data.laser_scan.intensities = new Float32Array(
          base64.toByteArray(data.laser_scan.intensities).buffer
        );
      }
      viz.scene.updateLaserScan(data.laser_scan);
    } else if (data.id === 'robot_svg') {
      if (data?.robot_svg?.robot_radius !== undefined && !isNaN(data.robot_svg.robot_radius)) {
        robotRadius = data.robot_svg.robot_radius;
      } else {
        console.error(
          'The robot_radius is missing or invalid. Using the default value of 1 meter.'
        );
      }
      viz.scene.updateRobotSvg(data.robot_svg);
    } else if (data.id === 'robot_pose') {
      if (status === 'mapping') {
        viz.scene.updateRobotPose(data.robot_pose);
      }
    } else if (data.id === 'raw_map') {
      if (status === 'mapping') {
        viz.scene.updateRawMap(data.raw_map);
      }
    } else if (data.id === 'save_map') {
      lastSaved = new Date(data.timestamp * 1000).toLocaleString('en-GB');
    } else if (data.id === 'status') {
      mapChannelStatus = data.status;
      if (mapCreatorStatus === mapChannelStatus) {
        status = data.status;
      } else {
        status = 'idle';
      }
      if (status === 'idle') {
        // clear map and reset robot pose
        viz.scene.clearRawMap();
        viz.scene.updateRobotPose({
          x: 0,
          y: 0,
          theta: 0
        });
        viz.zoom.zoomFit();
        lastSaved = null;
      }
    }
  }

  /* manual control */
  var _driving;
  var _bindings = [
    [1, 1, 1],
    [1, 0, 0],
    [1, -1, -1],
    [0, 1, 1],
    [0, 0, 0],
    [0, -1, -1],
    [-1, 1, -1],
    [-1, 0, 0],
    [-1, -1, 1]
  ];
  var _icons = [
    '',
    'fa-arrow-up',
    '',
    'fa-arrow-left',
    'fa-hand',
    'fa-arrow-right',
    '',
    'fa-arrow-down',
    ''
  ];

  function cellP(event, cell) {
    // To handle race condition of this function being called after onDestroy()
    if (destroyed) {
      return;
    }
    /* cell pressed */
    if (!Number.isInteger(cell) || cell < 0 || cell >= 9) {
      return;
    }
    var linearSpeed = speed.data;
    var angularSpeed = linearSpeed / robotRadius;
    var cmd = {
      command: 'force_drive',
      vx: _bindings[cell][0] * linearSpeed,
      vy: enableLateralControl ? _bindings[cell][1] * linearSpeed : 0,
      vth: enableLateralControl ? 0 : _bindings[cell][2] * angularSpeed,
      noos: speed.noos
    };

    if (_driving) {
      clearInterval(_driving);
      _driving = null;
    }
    _driving = setInterval(mapCreator.publish.bind(mapCreator, cmd), 100);
    mapCreator.publish(cmd);
    event.preventDefault();
  }

  function cellR(event, _cell) {
    /* cell released */
    if (_driving) {
      clearInterval(_driving);
      _driving = null;
      speed.noos = false;
    }
    event.preventDefault();
  }

  var joystick = {
    data: {
      x: 0.0,
      y: 0.0
    }
  };

  function onJoystick() {
    joystickPress();
    if (joystick.data.x === 0.0 && joystick.data.y === 0.0) {
      joystickRelease();
    }
  }

  function joystickPress() {
    // To handle race condition of this function being called after onDestroy()
    if (destroyed) {
      return;
    }
    var linearSpeed = speed.data;
    var lateralSpeed = linearSpeed;
    var angularSpeed = linearSpeed / robotRadius;
    linearSpeed = joystick.data.y * linearSpeed;

    if (enableLateralControl) {
      lateralSpeed = -joystick.data.x * lateralSpeed;
      angularSpeed = 0;
    } else {
      lateralSpeed = 0;
      angularSpeed = -joystick.data.x * angularSpeed;
      angularSpeed = joystick.data.y < 0 ? -angularSpeed : angularSpeed;
    }

    var cmd = {
      command: 'force_drive',
      vx: linearSpeed,
      vy: lateralSpeed,
      vth: angularSpeed,
      noos: speed.noos
    };

    if (_driving) {
      clearInterval(_driving);
      _driving = null;
    }
    _driving = setInterval(mapCreator.publish.bind(mapCreator, cmd), 100);
    mapCreator.publish(cmd);
  }

  function joystickRelease() {
    if (_driving) {
      clearInterval(_driving);
      _driving = null;
      speed.noos = false;
    }
  }

  /* keyboard control */
  let pressedKeys = new Set();
  let hasKeysPressed = false;
  let pressedKeyCell = -1;

  function getKeyboardCell() {
    let vx = 0,
      vy = 0;

    if (pressedKeys.has('Space')) {
      return 4;
    }

    if (pressedKeys.has('KeyW') || pressedKeys.has('ArrowUp')) {
      vx += 1;
    }
    if (pressedKeys.has('KeyS') || pressedKeys.has('ArrowDown')) {
      vx += -1;
    }

    if (pressedKeys.has('KeyA') || pressedKeys.has('ArrowLeft')) {
      vy += 1;
    }
    if (pressedKeys.has('KeyD') || pressedKeys.has('ArrowRight')) {
      vy += -1;
    }

    if (vx > 0) {
      if (vy > 0) {
        return 0;
      } else if (vy === 0) {
        return 1;
      } else {
        return 2;
      }
    } else if (vx === 0) {
      if (vy > 0) {
        return 3;
      } else if (vy === 0) {
        return 4;
      } else {
        return 5;
      }
    } else if (vx < 0) {
      if (vy > 0) {
        return 6;
      } else if (vy === 0) {
        return 7;
      } else {
        return 8;
      }
    }
    return 4; // Default to stop cell
  }

  function handleKeyDown(event) {
    if (navKey.indexOf(event.code) < 0) {
      return;
    }

    if (!enableManualControl || destroyed || enableHumanFollower) {
      if (pressedKeys.size > 0) {
        pressedKeys.clear();
        handlePressedKeysChanged();
      }
      return;
    }

    pressedKeys.add(event.code);

    handlePressedKeysChanged();
  }

  function handleKeyUp(event) {
    if (navKey.indexOf(event.code) < 0) {
      return;
    }

    if (!enableManualControl || destroyed || enableHumanFollower){
      if (pressedKeys.size > 0) {
        pressedKeys.clear();
        handlePressedKeysChanged();
      }
      return;
    }

    pressedKeys.delete(event.code);

    handlePressedKeysChanged();
  }

  function handlePressedKeysChanged() {
    const cell = getKeyboardCell();
    if (pressedKeys.size === 0) {
      cellR(event, cell);
    } else {
      cellP(event, cell);
    }

    hasKeysPressed = pressedKeys.size > 0;
    pressedKeyCell = hasKeysPressed ? cell : -1;
    if (hasKeysPressed) {
      enableVirtualKeypad = true;
    }
  }

  /* human follower */
  let enableHumanFollower = false;

  $: if (!enableManualControl && enableHumanFollower) toggleHumanFollower();

  function toggleHumanFollower() {
    mapCreator.publish({
      command: 'human_follower',
      enable: !enableHumanFollower
    });
  }

  const resizeListener = () => {
    vizResize();
  };

  function vizInit() {
    var u = get(user);

    element = cashDom('#map-creator-view');

    /* Re-initialize svg element. See comment below. */
    viz = mapViz(element[0], {
      grid: true,
      mapLayout: true,
      laserDownSample: u.is_agv_panel()
    });
    viz.zoom.zoomFit();

    window.addEventListener('resize', resizeListener);
    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);
    window.dispatchEvent(new Event('resize'));
  }

  function vizResize() {
    var p = element.parent();
    var rc = p[0].getBoundingClientRect();
    if (viz) {
      viz.resized(rc.width, rc.height);
    }
  }

  onMount(() => {
    // $log.debug($scope.controllerName, '$ionicView.enter');

    vizInit();

    // subscribe
    moduleManager.subscribe(moduleManagerCallback);
    mapCreator.subscribe(mapCreatorCallback);
    mapChannel.subscribe(mapCallback);

    // allow time for subscribe to happen first.
    setTimeout(function () {
      mapCreator.startModule();
    }, 100);

    return () => {
      window.removeEventListener('resize', resizeListener);
      window.removeEventListener('keydown', handleKeyDown);
      window.removeEventListener('keyup', handleKeyUp);
    };
  });

  onDestroy(() => {
    // $log.debug($scope.controllerName, '$ionicView.leave');
    destroyed = true;
    if (_driving) {
      clearInterval(_driving);
      _driving = null;
    }
    pressedKeys.clear();
    mapChannel.unsubscribe(mapCallback);
    mapCreator.unsubscribe(mapCreatorCallback);
    moduleManager.unsubscribe(moduleManagerCallback);

    /**
     * Remove svg element.
     * This is a bug fix for Firefox. The pattern fill in svg
     * does not render properly when the pattern id is
     * duplicated across multiple svgs.
     */
    // element.html('');
    viz = null;

    // $log.debug($scope.controllerName, '$destroy.');
  });

  //   $log.debug($scope.controllerName, 'initialized.');
</script>

<div class="flex h-full flex-col space-y-2">
  {#if !started}
    Starting Map Creator...
    <ProgressRadial width="w-4" />
  {:else}
    <div class="flex flex-row place-items-center space-x-4">
      <strong>Map Creator</strong>
      <div>
        <button
          type="button"
          class="variant-filled-warning btn btn-sm"
          on:click={startMapping}
          disabled={startMappingDisabled}>
          Start
        </button>
        <button
          type="button"
          class="variant-filled-primary btn btn-sm"
          on:click={saveMap}
          disabled={saveMapDisabled}>
          Save
        </button>
        <button
          type="button"
          class="variant-filled-primary btn btn-sm"
          on:click={stopMapping}
          disabled={stopMappingDisabled}>
          Save &amp; Stop
        </button>
      </div>
      {#if lastSaved}
        <span>Map saved at {lastSaved}</span>
      {/if}
      {#if lastSaved === 0}
        <span>Saving...</span>
      {/if}
    </div>
  {/if}
  <div class="relative grow">
    <svg
      style="width: 100%; height: 100%;border-radius: 1rem"
      xmlns="http://www.w3.org/2000/svg"
      id="map-creator-view"></svg>
    {#if $user.username !== 'Guest'}
      <div class="manual-control-overlay p-4">
        <div class="flex h-full flex-col items-end space-y-2">
          <div class="flex items-center justify-center align-middle">
            <SlideToggle
              name="enableManualControl"
              size="sm"
              class="slide-toggle"
              bind:checked={enableManualControl}>Manual Control</SlideToggle>
          </div>
          {#if enableManualControl}
            <div>
              <button
                type="button"
                class="btn w-14"
                tip-title="Rotate manual control"
                class:variant-filled-error={screenRotate}
                on:click={() => (screenRotate = !screenRotate)}>
                <i class="icon fa fa-undo"></i>
              </button>
              <button
                type="button"
                class="btn w-14"
                class:variant-filled-error={enableVirtualKeypad}
                tip-title="Toggle keypad display"
                on:click={() => (enableVirtualKeypad = !enableVirtualKeypad)}>
                <i class="fa-solid fa-table-cells"></i>
              </button>
              {#if isOmniDrive}
                <button
                  type="button"
                  class="btn w-14"
                  class:variant-filled-error={enableLateralControl}
                  tip-title="Toggle lateral control"
                  on:click={() => (enableLateralControl = !enableLateralControl)}>
                  <i class="fa-solid fa-arrows-up-down-left-right"></i>
                </button>
              {/if}
              <button
                type="button"
                class="btn w-14"
                tip-title="Enable/Disable obstacle sensor"
                class:variant-filled-error={speed.noos}
                on:click={() => (speed.noos = !speed.noos)}>
                <i class="fa-solid {speed.noos ? 'fa-eye-slash' : 'fa-eye'}"></i>
              </button>
              {#if hasHumanFollower}
                <button
                  type="button"
                  class="btn w-14"
                  class:variant-filled-error={enableHumanFollower}
                  tip-title="Follow me"
                  on:click={toggleHumanFollower}>
                  <i class="fa-solid fa-person-walking"></i>
                </button>
              {/if}
            </div>
            {#if enableHumanFollower}
              <div>
                <img
                  class="outline-solid max-h-32 bg-black outline-1 outline-gray-300"
                  alt="loading..."
                  crossorigin="anonymous"
                  src="/stream?topic=/human_follower/result&type=ros_compressed&t={Date.now()}" />
              </div>
            {:else if hasKeysPressed}
              <div
                class="screen-rotate col-span-3 mr-8 grid h-64 w-64 grid-cols-3 place-items-stretch gap-4"
                class:screen-rotate-active={screenRotate}>
                {#each { length: 9 } as _, i}
                  <div
                    class="rounded-xl flex items-center justify-center text-black"
                    class:bg-yellow-300={i !== 4 && i !== pressedKeyCell}
                    class:bg-red-400={i === 4 && i !== pressedKeyCell}
                    class:bg-yellow-500={i !== 4 && i === pressedKeyCell}
                    class:bg-red-600={i === 4 && i === pressedKeyCell}>
                    <i class="fa-solid {_icons[i]}"></i>
                  </div>
                {/each}
              </div>
            {:else if enableVirtualKeypad}
              <div
                class="screen-rotate col-span-3 mr-8 grid h-64 w-64 grid-cols-3 place-items-stretch gap-4"
                class:screen-rotate-active={screenRotate}>
                {#each { length: 9 } as _, i}
                  <!-- svelte-ignore a11y-mouse-events-have-key-events -->
                  <button
                    type="button"
                    class="variant-filled-warning rounded-xl"
                    class:variant-filled-error={i === 4}
                    on:mousedown={(event) => cellP(event, i)}
                    on:mouseup={(event) => cellR(event, i)}
                    on:mouseout={(event) => cellR(event, i)}
                    on:touchstart={(event) => cellP(event, i)}
                    on:touchend={(event) => cellR(event, i)}
                    on:dragstart={(event) => cellR(event, i)}>
                    <i class="fa-solid {_icons[i]}"></i>
                  </button>
                {/each}
              </div>
            {:else}
              <VirtualJoystick
                class="h-64 w-64 mr-8"
                data={joystick.data}
                onChange={onJoystick}
                {screenRotate}
              />
            {/if}
          {/if}
        </div>
      </div>
    {/if}
  </div>
</div>

<style>
  @import '$lib/styles/viz.css';
  @import '$lib/styles/map-viz.css';
</style>
