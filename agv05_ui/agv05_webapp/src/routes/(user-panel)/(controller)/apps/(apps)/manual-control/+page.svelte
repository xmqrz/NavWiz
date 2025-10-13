<script>
  import base64 from 'base64-js';
  import * as d3 from 'd3';
  import { getContext, onDestroy } from 'svelte';
  import { getModalStore, SlideToggle, RangeSlider } from '@skeletonlabs/skeleton';
  import { goto } from '$app/navigation';

  import AppLayout from 'components/AppLayout.svelte';
  import { amclChannel, mapXChannel as mapChannel } from 'stores/sock/index.js';
  import mapViz from '$lib/shared/mapx/viz';
  import VirtualJoystick from 'components/VirtualJoystick.svelte';
  import Pose2D from 'mapx-layout-editor/models/pose2d';

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

  const modalStore = getModalStore();
  const user = getContext('user');
  const mainController = getContext('mainController');
  let manualControl = mainController.moduleService.getSub('manual-control');

  let destroyed = false;
  let element;
  let viz;
  let lastCostmapTime = new Date();
  let settingPose = false;

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
    } else if (data.id === 'reflectors') {
      viz.scene.updateReflectors(data.reflectors);
    } else if (data.id === 'docking_reflectors') {
      viz.scene.updateDockingReflectors(data.docking_reflectors);
    } else if (data.id === 'docking_cloud') {
      data.docking_cloud.cloud = new Float32Array(
        base64.toByteArray(data.docking_cloud.cloud).buffer
      );
      viz.scene.updateDockingCloud(data.docking_cloud);
    } else if (data.id === 'target_pose') {
      viz.scene.updateMarkerPose(data.marker_pose);
      viz.scene.updateTargetPose(data.target_pose);
    } else if (data.id === 'docking_plan') {
      viz.scene.updateDockingPlan(data.docking_plan);
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
      viz.scene.updateRobotPose(getCorrectedPose(data.robot_pose));
    } else if (data.id === 'robot_path') {
      viz.scene.updateRobotPath(data.robot_path);
    } else if (data.id === 'robot_motion') {
      viz.scene.updateRobotMotion(data.robot_motion);
    } else if (data.id === 'raw_map') {
      viz.scene.updateRawMap(data.raw_map);
    } else if (data.id === 'costmap') {
      viz.scene.updateCostmap(data.costmap);
      lastCostmapTime = new Date();
    } else if (data.id === 'map_layout') {
      try {
        viz.models.load(JSON.parse(data.map_layout));
      } catch (e) {
        console.log('fail to parse map layout in manual control.');
      }
    }

    if (new Date() - lastCostmapTime > 3000) {
      // clear stale costmap (>3s)
      viz.scene.clearCostmap();
    }
  }

  function amclCallback(data) {
    if (data.id === 'particle_cloud') {
      viz.scene.updateParticleCloud(data.particle_cloud);
    }
  }

  let robotRadius = 1;
  let speed = {
    data: 0.3,
    previous: 0.3,
    reverse: false,
    noos: false
  };
  let motor = {
    free: false,
    previous: false
  };
  let enableManualControl = true;
  let enableSimpleGoal = false;
  let enableVirtualKeypad = false;
  var enableLateralControl = false;

  function speedUpdated() {
    if (speed.data > 1.5) {
      speed.data = 1.5;
    } else if (speed.data < 0.1) {
      speed.data = 0.1;
    }
    modalStore.trigger({
      type: 'confirm',
      title: `Confirm speed change to ${speed.data}?`,
      response: (r) => {
        if (r) {
          speed.previous = speed.data;
        } else {
          speed.data = speed.previous;
        }
      }
    });
  }

  function freeMotorUpdated() {
    modalStore.trigger({
      type: 'confirm',
      title: motor.free ? 'Confirm set motor to free?' : 'Confirm set motor to locked?',
      response: (r) => {
        if (r) {
          manualControl.publish({
            command: 'free_motor',
            free_motor: motor.free
          });
          motor.previous = motor.free;
        } else {
          motor.free = motor.previous;
        }
      }
    });
  }

  /* set pose */
  let robotPose;
  let correction;

  $: canSetPose = !!robotPose;

  function startSettingPose() {
    settingPose = true;
    correction = new Pose2D();
  }

  let _correcting;
  let _holding_scale = 0;
  function poseP(action) {
    // To handle race condition of this function being called after onDestroy()
    if (destroyed) {
      return;
    }
    /* button pressed */
    updateCorrection(action, 1);
    _correcting = setInterval(function () {
      _holding_scale += 1;
      let correction_scale = 5;
      if (_holding_scale > 15) {
        correction_scale = 50;
      } else if (_holding_scale > 10) {
        correction_scale = 20;
      } else if (_holding_scale > 5) {
        correction_scale = 10;
      }
      updateCorrection.bind(null, action, correction_scale)();
    }, 250);
  }

  function poseR() {
    /* button released */
    _holding_scale = 0;
    if (_correcting) {
      clearInterval(_correcting);
      _correcting = null;
    }
  }

  function setPose(data) {
    settingPose = false;
    if (data === true) {
      let newPose = new Pose2D(robotPose).add(correction);
      manualControl.publish({
        command: 'set_pose',
        set_pose: newPose
      });
    }
  }

  function updateCorrection(action, scale) {
    let newPose = new Pose2D(robotPose).add(correction);
    if (action === 'Up') {
      newPose.x += 0.01 * scale;
    } else if (action === 'Down') {
      newPose.x -= 0.01 * scale;
    } else if (action === 'Left') {
      newPose.y += 0.01 * scale;
    } else if (action === 'Right') {
      newPose.y -= 0.01 * scale;
    } else if (action === 'RL') {
      newPose.theta += ((0.5 * Math.PI) / 180.0) * scale;
    } else if (action === 'RR') {
      newPose.theta -= ((0.5 * Math.PI) / 180.0) * scale;
    }
    correction = newPose.sub2(robotPose);
  }

  function getCorrectedPose(data) {
    robotPose = data;
    if (!settingPose) {
      return data;
    }
    return new Pose2D(robotPose).add(correction);
  }

  /* dynamic path planning */
  let serverGoal = null;
  let clientGoal = null;
  let tagGoal = null;
  let trackedLine = [];
  let noTrackGoal = d3.track(false);
  let trackGoal = d3
    .track(true)
    .on('trackstart', trackGoalStarted)
    .on('track', trackedGoal)
    .on('trackend', trackGoalEnded);

  function trackGoalStarted(e) {
    /* jshint validthis: true */
    e.stopPropagation();
    let coord = viz.zoom.pixelToCoord(this.p0);
    trackedLine = [
      {
        x: coord[0],
        y: coord[1]
      }
    ];
    trackedLine.push(trackedLine[0]);
    drawClientGoal(trackedLine);
  }

  function trackedGoal() {
    /* jshint validthis: true */
    if (!this.p1) {
      return;
    }
    let coord = viz.zoom.pixelToCoord(this.p1);
    trackedLine[1] = {
      x: coord[0],
      y: coord[1]
    };
    drawClientGoal(trackedLine);
  }

  function trackGoalEnded() {
    /* jshint validthis: true */
    trackedGoal.call(this);

    viz.call(noTrackGoal);
    viz.style('cursor', 'default');
    viz.zoom.enable(true);
    trackedLine = [];

    manualControl.publish({
      command: 'nav',
      nav: clientGoal,
      speed: speed.reverse ? -speed.data : speed.data
    });
  }

  function drawClientGoal(r) {
    let dx = r[1].x - r[0].x;
    let dy = r[1].y - r[0].y;
    clientGoal = {
      x: r[0].x,
      y: r[0].y,
      theta: dx * dx + dy * dy > 0.1 ? Math.atan2(dy, dx) : null
    };
    drawGoal(clientGoal);
  }

  function drawGoal(goal) {
    if (!tagGoal) {
      tagGoal = viz.scene.append('g').attr('class', 'construct');
      tagGoal.append('use').attr({
        class: 'junction active',
        'xlink:href': '#junction'
      });
      tagGoal.station = tagGoal.append('use').attrs({
        class: 'station active',
        'xlink:href': '#station'
      });
    }
    if (goal.theta !== null) {
      tagGoal.station.attr('xlink:href', '#station');
      tagGoal.attr(
        'transform',
        `translate(${goal.x},${goal.y})rotate(${(goal.theta * 180.0) / Math.PI})`
      );
    } else {
      tagGoal.station.attr('xlink:href', '#station-headless');
      tagGoal.attr('transform', `translate(${goal.x},${goal.y})`);
    }
  }

  function simpleGoal(enable) {
    enableSimpleGoal = enable;
    viz.call(noTrackGoal);
    viz.style('cursor', 'default');
    viz.zoom.enable(true);
    trackedLine = [];
    clientGoal = null;

    if (!enable) {
      if (tagGoal) {
        tagGoal.remove();
        tagGoal = null;
      }
    }
  }

  function onSimpleGoal() {
    if (enableSimpleGoal) {
      if (serverGoal) {
        manualControl.publish({
          command: 'nav',
          nav: null
        });
      } else {
        simpleGoal(false);
      }
    } else {
      enableSimpleGoal = true;
      viz.call(trackGoal);
      viz.style('cursor', 'crosshair');
      viz.zoom.disablePan();
    }
  }

  /* manual control */
  let _driving;
  let _bindings = [
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
  let _icons = [
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
    let linearSpeed = speed.data;
    let angularSpeed = linearSpeed / robotRadius;
    let cmd = {
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
    _driving = setInterval(manualControl.publish.bind(manualControl, cmd), 100);
    manualControl.publish(cmd);
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

  let joystick = {
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
    let linearSpeed = speed.data;
    var lateralSpeed = linearSpeed;
    let angularSpeed = linearSpeed / robotRadius;
    linearSpeed = joystick.data.y * linearSpeed;
    if (enableLateralControl) {
      lateralSpeed = -joystick.data.x * lateralSpeed;
      angularSpeed = 0;
    } else {
      lateralSpeed = 0;
      angularSpeed = -joystick.data.x * angularSpeed;
      angularSpeed = joystick.data.y < 0 ? -angularSpeed : angularSpeed;
    }

    let cmd = {
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
    _driving = setInterval(manualControl.publish.bind(manualControl, cmd), 100);
    manualControl.publish(cmd);
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

    if (!enableManualControl || motor.free || settingPose || destroyed || enableHumanFollower) {
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

    if (!enableManualControl || motor.free || settingPose || destroyed || enableHumanFollower){
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
    manualControl.publish({
      command: 'human_follower',
      enable: !enableHumanFollower
    });
  }

  let dynamicPathPlanning = false;
  let isOmniDrive = false;
  let hasHumanFollower = false;

  function manualControlCallback(data) {
    if (data.command === 'status') {
      motor.free = data.is_motor_free;
      dynamicPathPlanning = data.dynamic_path_planning;
      isOmniDrive = data.is_omni_drive;
      if (motor.free) {
        simpleGoal(false);
      }
    } else if (data.command === 'nav') {
      if (data.nav) {
        if (serverGoal === null) {
          simpleGoal(true);
        }
      } else if (serverGoal) {
        simpleGoal(false);
      }

      serverGoal = data.nav;
      if (serverGoal) {
        drawGoal(serverGoal);
      }
    } else if (data.command === 'human_follower') {
      hasHumanFollower = !!(data.ready || data.running);
      enableHumanFollower = !!data.running;
    }
  }

  function vizInit(dom) {
    element = dom;

    viz = mapViz(element, {
      grid: true,
      mapLayout: true,
      laserDownSample: $user.is_agv_panel()
    });
    viz.zoom.zoomFit();

    // subscribe
    mapChannel.subscribe(mapCallback);
    amclChannel.subscribe(amclCallback);
    manualControl.subscribe(manualControlCallback);
    window.addEventListener('resize', vizResize);
    window.addEventListener('keydown', handleKeyDown);
    window.addEventListener('keyup', handleKeyUp);

    vizResize();

    setTimeout(function () {
      manualControl.publish({
        command: 'is_motor_free'
      });
      mapChannel.publish({
        id: 'retrieve_all'
      });
      amclChannel.publish({
        id: 'retrieve_all'
      });
    }, 100);

    return {
      destroy() {
        mapChannel.unsubscribe(mapCallback);
        amclChannel.unsubscribe(amclCallback);
        manualControl.unsubscribe(manualControlCallback);
        window.removeEventListener('resize', vizResize);
        window.removeEventListener('keydown', handleKeyDown);
        window.removeEventListener('keyup', handleKeyUp);
      }
    };
  }

  function vizResize() {
    if (!element) {
      return;
    }
    let p = element.parentNode;
    let rc = p.getBoundingClientRect();
    if (viz) {
      viz.resized(rc.width, rc.height);
    }
  }

  function shutdown() {
    const modal = {
      type: 'confirm',
      title: 'Confirm stopping module<br/>"Manual Control" ?',
      response: (r) => {
        if (r) {
          manualControl.stopModule();
          goto('/apps', { replaceState: true });
        }
      }
    };
    modalStore.trigger(modal);
  }

  onDestroy(() => {
    destroyed = true;
    if (_driving) {
      clearInterval(_driving);
      _driving = null;
    }
    if (_correcting) {
      clearInterval(_correcting);
      _correcting = null;
    }
    pressedKeys.clear();
  });
</script>

<AppLayout title="Manual Control" moduleID="manual-control" on:close={shutdown}>
  <div class="flex h-full w-full flex-col">
    <div class="grid grid-cols-1 gap-4 md:grid-cols-5">
      <div class="col-span-3 p-2 px-4">
        <RangeSlider
          name="speed.data"
          bind:value={speed.data}
          on:change={speedUpdated}
          min={0.1}
          max={1.5}
          step={0.1}
          ticked>
          <div class="flex items-center justify-between">
            <div class="font-bold">Speed</div>
            <div class="text-md">{parseFloat(speed.data).toFixed(1)} m/s</div>
          </div>
        </RangeSlider>
      </div>
      <div class="flex flex-row">
        {#if !settingPose}
          <button
            type="button"
            class="btn"
            on:click={startSettingPose}
            disabled={!canSetPose}
            tip-title="Set Pose">
            <i class="fa-lg fa-solid fa-location-crosshairs"></i>
          </button>
        {:else}
          <div class="flex h-full flex-col place-content-evenly">
            <div class="grid grid-cols-4">
              <button
                type="button"
                class="btn btn-sm"
                on:mousedown={() => poseP('RL')}
                on:mouseup={() => poseR()}
                on:touchstart={() => poseP('RL')}
                on:touchend={() => poseR()}
                on:touchcancel={() => poseR()}>
                <i class="fa fa-undo"></i>
              </button>
              <button
                type="button"
                class="btn btn-sm"
                on:mousedown={() => poseP('Up')}
                on:mouseup={() => poseR()}
                on:touchstart={() => poseP('Up')}
                on:touchend={() => poseR()}
                on:touchcancel={() => poseR()}>
                <i class="fa-solid fa-arrow-up"></i>
              </button>
              <button
                type="button"
                class="btn btn-sm"
                on:mousedown={() => poseP('RR')}
                on:mouseup={() => poseR()}
                on:touchstart={() => poseP('RR')}
                on:touchend={() => poseR()}
                on:touchcancel={() => poseR()}>
                <i class="fa-solid fa-rotate-right"></i>
              </button>
              <button
                type="button"
                class="variant-filled-error btn btn-sm"
                on:click={() => setPose(false)}
                tip-title="Cancel">
                <i class="fa-solid fa-xmark"></i>
              </button>
            </div>
            <div class="grid grid-cols-4">
              <button
                type="button"
                class="btn btn-sm"
                on:mousedown={() => poseP('Left')}
                on:mouseup={() => poseR()}
                on:touchstart={() => poseP('Left')}
                on:touchend={() => poseR()}
                on:touchcancel={() => poseR()}>
                <i class="fa-solid fa-arrow-left"></i>
              </button>
              <button
                type="button"
                class="btn btn-sm"
                on:mousedown={() => poseP('Down')}
                on:mouseup={() => poseR()}
                on:touchstart={() => poseP('Down')}
                on:touchend={() => poseR()}
                on:touchcancel={() => poseR()}>
                <i class="fa-solid fa-arrow-down"></i>
              </button>
              <button
                type="button"
                class="btn btn-sm"
                on:mousedown={() => poseP('Right')}
                on:mouseup={() => poseR()}
                on:touchstart={() => poseP('Right')}
                on:touchend={() => poseR()}
                on:touchcancel={() => poseR()}>
                <i class="fa-solid fa-arrow-right"></i>
              </button>
              <button
                type="button"
                class="variant-filled-success btn btn-sm"
                on:click={() => setPose(true)}
                tip-title="Update pose">
                <i class="fa-solid fa-check"></i>
              </button>
            </div>
          </div>
        {/if}
        {#if dynamicPathPlanning && (enableSimpleGoal || !motor.free)}
          <button
            type="button"
            class="btn m-auto mx-3 h-10"
            class:variant-filled-error={enableSimpleGoal}
            on:click={() => onSimpleGoal()}
            tip-title="Go To">
            <i class="fa-solid fa-location-arrow"></i>
          </button>
        {/if}
      </div>
      <div class="flex items-center justify-end p-2 px-4 align-middle">
        <SlideToggle
          name="free-motor"
          size="sm"
          class="w-full min-w-min max-w-[174px] p-2"
          bind:checked={motor.free}
          on:change={freeMotorUpdated}>Free Motor</SlideToggle>
      </div>
    </div>
    <div class="relative grow">
      <svg style="width: 100%; height: 100%" xmlns="http://www.w3.org/2000/svg" use:vizInit>
      </svg>
      {#if !motor.free}
        <div class="manual-control-overlay absolute inset-0 p-2">
          <div class="flex h-full flex-col items-end space-y-2">
            <div class="flex w-full max-w-[180px] items-center justify-end">
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
                  class:variant-filled-error={speed.reverse}
                  on:click={() => (speed.reverse = !speed.reverse)}>
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
                  class:screen-rotate-active={!speed.reverse}>
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
                  class:screen-rotate-active={!speed.reverse}>
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
                  class="mr-8 h-64 w-64"
                  data={joystick.data}
                  onChange={onJoystick}
                  screenRotate={!speed.reverse} />
              {/if}
            {/if}
          </div>
        </div>
      {/if}
    </div>
  </div>
</AppLayout>

<style>
  @import '$lib/styles/viz.css';
  @import '$lib/styles/map-viz.css';
</style>
